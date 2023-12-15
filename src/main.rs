
#![no_std]
#![no_main]

use core::ops::{Add, Sub};
use rp_pico::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use fugit::{ RateExtU32};

use rp_pico as bsp;
use embedded_hal::digital::v2::{OutputPin};
use bsp::hal as p_hal;
use numtoa::NumToA;
use arraystring::{ArrayString, typenum::{U40} };

use rv3028c7_rtc::{RV3028, DateTimeAccess, Duration, NaiveDateTime, NaiveDate, NaiveTime, Datelike, Timelike, Weekday};
use embedded_hal::blocking::i2c::{Write, Read, WriteRead};

use p_hal::{
  Clock,
  clocks::{init_clocks_and_plls},
  pac::{self},
  sio::Sio,
  watchdog::Watchdog,
  rtc::{self, DayOfWeek},
  gpio::{FunctionI2C, PullUp},

};

// use rp2040_hal as p_hal;
// use p_hal::{
//   pac::{self, interrupt},
//   sio::Sio,
//   clocks::{ClockGate, Clock, init_clocks_and_plls},
//   // rtc::{self}
//   watchdog::Watchdog,
//   rtc::{self, DayOfWeek}
//
// };



use embedded_graphics::{
  prelude::*,
  mono_font::MonoTextStyleBuilder,
  text::{Alignment, Baseline, Text, TextStyleBuilder}
};

use embedded_graphics_core::{
  geometry::OriginDimensions,
  pixelcolor::BinaryColor,
};


use epd_waveshare::{
  prelude::*,
  epd2in9bc::*,
  graphics::{DisplayRotation},

};
use rp_pico::hal::gpio::Pin;

type DisplayType = Display2in9bc;


// fn generate_datetime() -> rtc::DateTime {
//   // compile time UTC
//   let compile_dt = compile_time::datetime!();
//   // cheezy Pacific time offset from UTC
//   let orig_dt = compile_dt.saturating_sub(time::Duration::hours(8));
//
//   rtc::DateTime {
//     year: orig_dt.year() as u16,
//     month: orig_dt.month() as u8,
//     day: orig_dt.day() as u8,
//     day_of_week: DayOfWeek::Wednesday,
//     hour: orig_dt.hour(),
//     minute: orig_dt.minute(),
//     second: orig_dt.second(),
//   }
// }


fn rpico_weekday_from_naive(val: Weekday) -> rtc::DayOfWeek {
  match val {
    Weekday::Sun => DayOfWeek::Sunday,
    Weekday::Mon => DayOfWeek::Monday,
    Weekday::Tue => DayOfWeek::Tuesday,
    Weekday::Wed => DayOfWeek::Wednesday,
    Weekday::Thu => DayOfWeek::Thursday,
    Weekday::Fri => DayOfWeek::Friday,
    Weekday::Sat => DayOfWeek::Saturday,
  }
}

fn naive_datetime_to_rpico(dt: &NaiveDateTime) -> rtc::DateTime {
  rtc::DateTime {
    year: dt.year().try_into().unwrap(),
    month: dt.month().try_into().unwrap(),
    day: dt.day().try_into().unwrap(),
    day_of_week: rpico_weekday_from_naive(dt.weekday()),
    hour: dt.hour().try_into().unwrap(),
    minute: dt.minute().try_into().unwrap(),
    second: dt.second().try_into().unwrap(),
  }
}

#[entry]
fn main() -> ! {

  info!("Program start");
  let mut pac = pac::Peripherals::take().unwrap();
  let core = pac::CorePeripherals::take().unwrap();
  let mut watchdog = Watchdog::new(pac.WATCHDOG);
  let sio = Sio::new(pac.SIO);

  // External high-speed crystal on the pico board is 12Mhz
  let external_xtal_freq_hz = 12_000_000u32;
  let clocks = init_clocks_and_plls(
    external_xtal_freq_hz,
    pac.XOSC,
    pac.CLOCKS,
    pac.PLL_SYS,
    pac.PLL_USB,
    &mut pac.RESETS,
    &mut watchdog,
  )
    .ok()
    .unwrap();

  // Only leave the rtc's clock enabled while in deep sleep.
  // let mut config = ClockGate::default();
  // config.set_rtc_rtc(true);
  // clocks.configure_sleep_enable(config);


  // let sys_dt = generate_datetime();
  // let mut sys_rtc = rtc::RealTimeClock::new(
  //   pac.RTC,
  //   clocks.rtc_clock,
  //   &mut pac.RESETS,
  //   sys_dt,
  // ).unwrap();

  let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

  let pins = bsp::Pins::new(
    pac.IO_BANK0,
    pac.PADS_BANK0,
    sio.gpio_bank0,
    &mut pac.RESETS,
  );


  // Configure two pins as being I²C, not GPIO
  let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio4.reconfigure();
  let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio5.reconfigure();

  // Create the I²C drive, using the two pre-configured pins. This will fail
  // at compile time if the pins are in the wrong mode, or if this I²C
  // peripheral isn't available on these pins!
  let i2c = p_hal::I2C::i2c0(
    pac.I2C0,
    sda_pin,
    scl_pin,
    400.kHz(),
    &mut pac.RESETS,
    &clocks.system_clock,
  );

  // Create a new instance of the RV3028 driver
  let mut rtc = RV3028::new(i2c);

  // Set from external RTC to internal (since internal is known to be bogus)
  let rtc_dt = rtc.datetime().unwrap();
  let sys_dt = naive_datetime_to_rpico(&rtc_dt);
  let mut sys_rtc = p_hal::rtc::RealTimeClock::new(
    pac.RTC,
    clocks.rtc_clock,
    &mut pac.RESETS,
    sys_dt,
  ).unwrap();

  let mut led_pin = pins.led.into_push_pull_output();

   let spi0_sck_pin = pins.gpio6.into_function::<bsp::hal::gpio::FunctionSpi>();
  let spi0_do_pin = pins.gpio7.into_function::<bsp::hal::gpio::FunctionSpi>();

  // rst_pin is used to reset the display during power-on sequence
  let mut rst_pin = pins.gpio10.into_push_pull_output();
  rst_pin.set_high().unwrap();
  delay.delay_us(200);
  rst_pin.set_low().unwrap();

  let busy_in = pins.gpio11.into_floating_input();

  let dc_pin  = pins.gpio8.into_push_pull_output();
  let cs_pin = pins.gpio9.into_push_pull_output();

  // (Tx, Sck) (MOSI, SCK)
  let spi0 = p_hal::Spi::<_, _, _, 8>::new(pac.SPI0, (spi0_do_pin, spi0_sck_pin) );

  println!("create SPI...");
  // Exchange the uninitialized SPI driver for an initialized one
  let mut spi = spi0.init(
    &mut pac.RESETS,
    clocks.peripheral_clock.freq(),
    16u32.MHz(),
    //&embedded_hal::spi::MODE_0,
    &embedded_hal::spi::MODE_3,
  );

  println!("create EPD...");
  let mut epd = Epd2in9bc::new(
    &mut spi,
    cs_pin,
    busy_in,
    dc_pin,
    rst_pin,
    &mut delay,
  ).unwrap();

  // epd.set_background_color(TriColor::White);
  epd.set_border_color(&mut spi, TriColor::Chromatic).unwrap();

  println!("create display...");
  // Use display graphics from embedded-graphics
  let mut display = DisplayType::default();
  display.set_rotation(DisplayRotation::Rotate90);
  let disp_size = display.size();
  println!("size : {} x {}", display.size().width,display.size().height);
  display.clear_buffer(Color::White);

  println!("create styles...");
  let fg_char_style = MonoTextStyleBuilder::new()
    .font(&embedded_graphics::mono_font::ascii::FONT_10X20)
    .text_color(BinaryColor::On)
    .background_color(BinaryColor::Off )
    .build();

  let align_style = TextStyleBuilder::new()
    .baseline(Baseline::Middle)
    .alignment(Alignment::Center)
    .build();

  let ctr_point =  Point::new( (disp_size.height/2) as i32, (disp_size.width/2) as i32);
  let time_point = ctr_point.sub(Point::new(0,10));
  let date_point = ctr_point.add(Point::new(0,10));

  println!("enter loop...");
  let mut last_minute = 0;
  let mut text_buf =  ArrayString::<U40>::new();
  loop {
    if let Ok(dt) = sys_rtc.now() {
      if dt.minute == last_minute {
        // don't refresh until the next minute
        delay.delay_ms(250);
        continue;
      }
      last_minute = dt.minute;

      let _ = led_pin.set_high();
      // display.clear_buffer(Color::White);
      epd.wake_up(&mut spi, &mut delay).unwrap();
      epd.clear_frame(&mut spi, &mut delay).unwrap();

      format_time(&dt,&mut text_buf);
      let time_text =
        Text::with_text_style(
          &text_buf,
          time_point,
          fg_char_style, align_style);
      time_text.draw(&mut display).unwrap();

      format_date(&dt,&mut text_buf);
      let date_text =
        Text::with_text_style(
          &text_buf,
          date_point,
          fg_char_style, align_style);
      date_text.draw(&mut display).unwrap();
      let draw_dt =  sys_rtc.now().unwrap();

      // push changes to display
      epd.update_and_display_frame(&mut spi, &display.buffer(), &mut delay).unwrap();
      let final_dt = sys_rtc.now().unwrap();
      println!("update+display delta {}", final_dt.second - draw_dt.second);

      println!("sleeping: {:02}:{:02}:{:02}", final_dt.hour, final_dt.minute, final_dt.second);
      // put display into low power mode
      epd.sleep(&mut spi, &mut delay).unwrap();
      let _ = led_pin.set_low();
      // wait until the Pico RTC wakes us up
      // sys_rtc.schedule_alarm(rtc::DateTimeFilter::default().second(0));
      // cortex_m::asm::wfi();
      // sys_rtc.clear_interrupt();
      // println!("awake");
    }

  }

  // #[allow(non_snake_case)]
  // #[interrupt]
  // fn RTC_IRQ() {
  //   critical_section::with(|cs| {
  //     // borrow the content of the Mutexed RefCell.
  //     let mut maybe_rtc = SHARED_RTC.borrow_ref_mut(cs);
  //
  //     // borrow the content of the Option
  //     if let Some(rtc) = maybe_rtc.as_mut() {
  //       // clear the interrupt flag so that it stops firing for now and can be triggered again.
  //       rtc.clear_interrupt();
  //     }
  //   });
  // }

  fn format_time(dt: &rtc::DateTime, text_buf: &mut ArrayString::<U40>) {
    let mut num_buffer = [0u8; 20];
    text_buf.clear();
    if dt.hour < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.hour.numtoa_str(10, &mut num_buffer));
    text_buf.push_str(":");
    if dt.minute < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.minute.numtoa_str(10, &mut num_buffer));
    text_buf.push_str(":");
    if dt.second < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.second.numtoa_str(10, &mut num_buffer));
  }

  fn format_date(dt: &rtc::DateTime, text_buf: &mut ArrayString::<U40>) {
    let mut num_buffer = [0u8; 20];
    text_buf.clear();
    text_buf.push_str(dt.year.numtoa_str(10, &mut num_buffer));
    text_buf.push_str("-");
    if dt.month < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.month.numtoa_str(10, &mut num_buffer));
    text_buf.push_str("-");
    if dt.day < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.day.numtoa_str(10, &mut num_buffer));
  }


}


