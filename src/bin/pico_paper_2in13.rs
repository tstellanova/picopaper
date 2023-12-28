
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
// use embedded_hal::blocking::i2c::{Write, Read, WriteRead};

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

use epd_waveshare::{prelude::*,
                    graphics::{DisplayRotation},
                    // color::Color,
                    // epd2in13bc::{Epd2in13bc, Display2in13bc},
                    epd2in13_v2::{Epd2in13, Display2in13},
};

use rp_pico::hal::gpio::Pin;

type DisplayType = Display2in13;


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

fn draw_text(display: &mut DisplayType, text: &str, x: i32, y: i32) {
  let text_color = epd_waveshare::color::Black;
  let bg_color = epd_waveshare::color::White;
  let style = MonoTextStyleBuilder::new()
    .font(&embedded_graphics::mono_font::ascii::FONT_6X10)
    .text_color(text_color)
    .background_color(bg_color)
    .build();

  let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();
  let _ = Text::with_text_style(text, Point::new(x, y), style, text_style).draw(display);
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

  let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

  let pins = bsp::Pins::new(
    pac.IO_BANK0,
    pac.PADS_BANK0,
    sio.gpio_bank0,
    &mut pac.RESETS,
  );

  // Configure two pins as being I²C, not GPIO
  let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio2.reconfigure();
  let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio3.reconfigure();

  // Create the I²C drive, using the two pre-configured pins. This will fail
  // at compile time if the pins are in the wrong mode, or if this I²C
  // peripheral isn't available on these pins!
  let i2c = p_hal::I2C::i2c1(
    pac.I2C1,
    sda_pin,
    scl_pin,
    400.kHz(),
    &mut pac.RESETS,
    &clocks.system_clock,
  );

  // Create a new instance of the RV3028 driver
  let mut rtc = RV3028::new(i2c);

  // Set from external RTC to internal (since internal is known to be bogus)
  let rtc_dt = {
    let testo = rtc.datetime();
    if let Ok(val) = testo {
      val
    }
    else {
      println!("ext rtc fail");
      NaiveDateTime::default()
    }
  } ;

  // let mut rtc_dt = NaiveDateTime::default();
  let sys_dt = naive_datetime_to_rpico(&rtc_dt);
  let sys_rtc = p_hal::rtc::RealTimeClock::new(
    pac.RTC,
    clocks.rtc_clock,
    &mut pac.RESETS,
    sys_dt,
  ).unwrap();

  let mut led_pin = pins.led.into_push_pull_output();
  led_pin.set_high().unwrap();

  let mut dc_pin  = pins.gpio8.into_push_pull_output(); // D/C -- pin 11
  let mut cs_pin = pins.gpio9.into_push_pull_output(); // SPI1 CS -- pin 12
  let spi1_sck_pin = pins.gpio10.into_function::<bsp::hal::gpio::FunctionSpi>(); //SCK -- pin 14
  let spi1_do_pin = pins.gpio11.into_function::<bsp::hal::gpio::FunctionSpi>(); //MOSI -- pin 15
  // rst_pin is used to reset the display during power-on sequence
  let mut rst_pin = pins.gpio12.into_push_pull_output(); //RST - pin 16
  let busy_in = pins.gpio13.into_floating_input(); //BUSY pin 17

  dc_pin.set_high().unwrap();
  cs_pin.set_high().unwrap();
  rst_pin.set_high().unwrap();

  // pico paper pins for 2.13 inch b/w display with partial refresh
  // DC - pin 11 (data/command, high for data, low for command)
  // CS - pin 12 (chip select, low active)
  // GND - pin 13
  // CLK - pin 14 (SCK)
  // DIN - pin 15 (MOSI)
  // RST - pin 16 (external reset, low active)
  // BUSY - pin 17 (display busy status output)
  //

  // (Tx, Sck) (MOSI, SCK)
  let spi1_periph = p_hal::Spi::<_, _, _, 8>::new(pac.SPI1, (spi1_do_pin, spi1_sck_pin) );

  println!("create SPI...");
  // Exchange the uninitialized SPI driver for an initialized one
  let mut spi = spi1_periph.init(
    &mut pac.RESETS,
    clocks.peripheral_clock.freq(),
    4u32.MHz(),
    embedded_hal::spi::MODE_0
  );

  println!("create EPD...");
  let mut epd = Epd2in13::new(
    &mut spi,
    cs_pin,
    busy_in,
    dc_pin,
    rst_pin,
    &mut delay,
  ).expect("epaper new error");

  epd.wake_up(&mut spi, &mut delay).unwrap();
  epd.clear_frame(&mut spi, &mut delay).unwrap();

  println!("create display...");
  // Use display graphics from embedded-graphics
  let mut display = DisplayType::default();
  display.clear_buffer(Color::White);

  let raw_disp_dim = display.size();
  // should be 122 w x 250 h
  println!("raw_disp_dim : w {} x h {}", raw_disp_dim.width, raw_disp_dim.height);
  let disp_width = raw_disp_dim.width.max(raw_disp_dim.height) as i32;
  let disp_height = raw_disp_dim.width.min(raw_disp_dim.height) as i32;
  let min_dim = disp_height / 2;

  // epd.set_refresh(&mut spi, &mut delay, RefreshLut::Quick).unwrap();

  display.set_rotation(DisplayRotation::Rotate0);
  draw_text(&mut display, "0!", min_dim, min_dim);

  display.set_rotation(DisplayRotation::Rotate90);
  draw_text(&mut display, "90!", min_dim, min_dim);

  display.set_rotation(DisplayRotation::Rotate180);
  draw_text(&mut display, "180!", min_dim, min_dim);

  display.set_rotation(DisplayRotation::Rotate270);
  draw_text(&mut display, "270!", min_dim, min_dim);

  epd.update_and_display_frame(&mut spi, display.buffer(), &mut delay).expect("update fail");

  for _i in 0..10 {
    delay.delay_ms(200);
  }

  led_pin.set_low().unwrap();

  let fg_char_style = MonoTextStyleBuilder::new()
    .font(&embedded_graphics::mono_font::ascii::FONT_10X20)
    .text_color(BinaryColor::On)
    .background_color(BinaryColor::Off )
    .build();

  let align_style = TextStyleBuilder::new()
    .baseline(Baseline::Middle)
    .alignment(Alignment::Center)
    .build();

  const TEXT_FONT_HEIGHT: i32 = 20;
  let ctr_point =  Point::new(disp_width/2 , disp_height/2);
  let time_point = ctr_point.sub(Point::new(0,TEXT_FONT_HEIGHT));
  let wd_point = time_point.add(Point::new(0,TEXT_FONT_HEIGHT));
  let date_point = wd_point.add(Point::new(0,TEXT_FONT_HEIGHT));

  // use Landscape mode, with the RPico pin 1 "up"
  display.set_rotation(DisplayRotation::Rotate90);

  // sys_rtc.disable_alarm();
  // unsafe {
  //   pac::NVIC::unmask(pac::Interrupt::RTC_IRQ);
  // }


  println!("enter loop...");
  let mut last_minute = 99;
  let mut text_buf =  ArrayString::<U40>::new();
  loop {
    if let Ok(dt) = sys_rtc.now() {
      delay.delay_ms(1000);
      if dt.minute == last_minute {
        // println!("wait..{:02}:{:02}", dt.minute, dt.second);
        // don't refresh until the next minute
        delay.delay_ms(250);
        continue;
      }
      last_minute = dt.minute;

      // rtc_dt = rtc_dt
      //   .with_hour(dt.hour as u32).unwrap()
      //   .with_minute(dt.minute as u32).unwrap()
      //   .with_second(dt.second as u32).unwrap();

      // TODO get time from external RTC
      let rtc_dt = rtc.datetime().unwrap();
      let local_dt = rtc_dt.sub(Duration::hours(8));

      let _ = led_pin.set_high();
      display.clear_buffer(Color::White);
      // epd.wake_up(&mut spi, &mut delay).unwrap();
      //epd.clear_frame(&mut spi, &mut delay).unwrap();

      format_time(&local_dt,&mut text_buf);
      let time_text =
        Text::with_text_style(
          &text_buf,
          time_point,
          fg_char_style, align_style);
      time_text.draw(&mut display).unwrap();

      format_weekday(&local_dt, &mut text_buf);
      let wd_text =
        Text::with_text_style(
          &text_buf,
          wd_point,
          fg_char_style, align_style);
      wd_text.draw(&mut display).unwrap();

      format_date(&local_dt,&mut text_buf);
      let date_text =
        Text::with_text_style(
          &text_buf,
          date_point,
          fg_char_style, align_style);
      date_text.draw(&mut display).unwrap();

      let draw_dt =  sys_rtc.now().unwrap();

      // Demonstrating how to use the partial refresh feature of the screen.
      // Real animations can be used.
      // epd.set_refresh(&mut spi, &mut delay, RefreshLut::Quick).unwrap();
      // epd.clear_frame(&mut spi, &mut delay).unwrap();

      // push changes to display
      epd.wake_up(&mut spi, &mut delay).unwrap();
      epd.set_refresh(&mut spi, &mut delay, RefreshLut::Quick).unwrap();
      epd.update_and_display_frame(&mut spi, &display.buffer(), &mut delay).unwrap();
      let final_dt = sys_rtc.now().unwrap();
      println!("tdelta {}", final_dt.second - draw_dt.second);

      println!("sleeping: {:02}:{:02}:{:02}", rtc_dt.time().hour() , rtc_dt.time().minute(), rtc_dt.time().second());
      // put display into low power mode
      epd.sleep(&mut spi, &mut delay).unwrap();
      let _ = led_pin.set_low();

      // // wait until the Pico RTC wakes us up
      // sys_rtc.schedule_alarm(rtc::DateTimeFilter::default().second(33));
      // // Let the alarm trigger an interrupt in the NVIC.
      // cortex_m::asm::wfi();
      // sys_rtc.clear_interrupt();
      // println!("awake");
    }



  }



 use crate::pac::interrupt;

  #[allow(non_snake_case)]
  #[interrupt]
  fn RTC_IRQ() {
    println!("hiya");

    // critical_section::with(|cs| {
    //   // borrow the content of the Mutexed RefCell.
    //   // let mut maybe_rtc = SHARED_RTC.borrow_ref_mut(cs);
    //
    //   println!("hiya");
    //   // // borrow the content of the Option
    //   // if let Some(rtc) = maybe_rtc.as_mut() {
    //   //   // clear the interrupt flag so that it stops firing for now and can be triggered again.
    //   //   rtc.clear_interrupt();
    //   // }
    // });
  }


  fn format_time(dt: &NaiveDateTime, text_buf: &mut ArrayString::<U40>) {
    let mut num_buffer = [0u8; 20];
    text_buf.clear();
    if dt.time().hour() < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.time().hour().numtoa_str(10, &mut num_buffer));
    text_buf.push_str(":");
    if dt.time().minute() < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.time().minute().numtoa_str(10, &mut num_buffer));
    text_buf.push_str(":");
    if dt.time().second() < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.time().second().numtoa_str(10, &mut num_buffer));
  }


  fn format_date(dt: &NaiveDateTime, text_buf: &mut ArrayString::<U40>) {
    let mut num_buffer = [0u8; 20];
    text_buf.clear();
    text_buf.push_str(dt.date().year().numtoa_str(10, &mut num_buffer));
    text_buf.push_str("-");
    if dt.date().month() < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.date().month().numtoa_str(10, &mut num_buffer));
    text_buf.push_str("-");
    if dt.date().day() < 10 { text_buf.push_str("0");}
    text_buf.push_str(dt.date().day().numtoa_str(10, &mut num_buffer));
  }

  fn format_weekday(dt: &NaiveDateTime,  text_buf: &mut ArrayString::<U40>) {
    text_buf.clear();
    match dt.weekday() {
      Weekday::Sun => {
        text_buf.push_str("Sunday");
      }
      Weekday::Mon => {
        text_buf.push_str("Monday");
      }
      Weekday::Tue => {
        text_buf.push_str("Tuesday");
      }
      Weekday::Wed => {
        text_buf.push_str("Wednesday");
      }
      Weekday::Thu => {
        text_buf.push_str("Thursday");
      }
      Weekday::Fri => {
        text_buf.push_str("Friday");
      }
      Weekday::Sat => {
        text_buf.push_str("Saturday");
      }
    }
  }

}


