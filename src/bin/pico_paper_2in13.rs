
#![no_std]
#![no_main]

use core::ops::{Add, Sub};
use rp_pico::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use fugit::{ RateExtU32};

use rp_pico as bsp;
use embedded_hal::{
  digital::v2::{OutputPin},
  PwmPin,
  blocking::i2c::{Write, Read, WriteRead}
};

use bsp::hal as p_hal;
use numtoa::NumToA;
use arraystring::{ArrayString, typenum::{U40} };

use rv3028c7_rtc::{RV3028, DateTimeAccess, Duration, NaiveDateTime, NaiveDate, NaiveTime, Datelike, Timelike, Weekday};

use p_hal::{
  Clock,
  clocks::{init_clocks_and_plls},
  pac::{self},
  sio::Sio,
  watchdog::Watchdog,
  rtc::{self, DayOfWeek},
  gpio::{FunctionI2C, PullUp},
  pwm::{FreeRunning, Pwm3, Slice},

};
use cortex_m::delay::Delay;


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

  println!("setup PWM audio");
  let mut pwm_slices = p_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

  // Configure PWM3
  let mut pwm = &mut pwm_slices.pwm3;
  pwm.set_ph_correct();
  pwm.enable();

  // Create PWM driver on GPIO22 / "GP22" pin
  let channel = &mut pwm.channel_a;
  channel.output_to(pins.gpio22);
  channel.set_duty(0);//initially off

  pwm.set_div_int(AUDIO_PWM_DIVISOR); // To set integer part of clock divider
  pwm.set_div_frac(0u8); // To set fractional part of clock divider

  println!("test notes");
  // play_tune(&DO_RE_MI_TUNE, &mut pwm, &mut delay);
  play_tune(&HALF_HOUR_CHIME , &mut pwm, &mut delay);

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
    if let Ok(rtc_dt) = rtc.datetime() {
      if rtc_dt.minute() == last_minute {
        // println!("wait..{:02}:{:02}", dt.minute, dt.second);
        // don't refresh until the next minute
        delay.delay_ms(200);
        continue;
      }
      let _ = led_pin.set_high();
      last_minute = rtc_dt.minute();

      // get time from external RTC
      // let rtc_dt = rtc.datetime().unwrap();
      let local_dt = rtc_dt.sub(Duration::hours(8));

      println!("check: {:02}:{:02}:{:02}",
               local_dt.time().hour() , local_dt.time().minute(), local_dt.time().second());

      if local_dt.day() == 1 && local_dt.month() == 1 &&
        local_dt.hour() == 0 && local_dt.minute() == 0 {
        play_tune(&AULD_LANG_SYNE_VERSE1, &mut pwm, &mut delay);
        play_tune(&AULD_LANG_SYNE_VERSE2, &mut pwm, &mut delay);
        play_tune(&AULD_LANG_SYNE_VERSE2, &mut pwm, &mut delay);
      }
      else if (local_dt.minute() == 0) {
        play_tune(&HOURLY_CHIME, &mut pwm, &mut delay);
      }
      else if (local_dt.minute() % 30 == 0) {
        play_tune(&HALF_HOUR_CHIME, &mut pwm, &mut delay);
      }


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

      // let draw_dt =  sys_rtc.now().unwrap();

      // Demonstrating how to use the partial refresh feature of the screen.
      // Real animations can be used.
      // epd.set_refresh(&mut spi, &mut delay, RefreshLut::Quick).unwrap();
      // epd.clear_frame(&mut spi, &mut delay).unwrap();

      // push changes to display
      epd.wake_up(&mut spi, &mut delay).unwrap();
      epd.set_refresh(&mut spi, &mut delay, RefreshLut::Quick).unwrap();
      epd.update_and_display_frame(&mut spi, &display.buffer(), &mut delay).unwrap();
      let final_dt = sys_rtc.now().unwrap();
      // println!("tdelta {}", final_dt.second - draw_dt.second);

      // put display into low power mode
      epd.sleep(&mut spi, &mut delay).unwrap();
      let _ = led_pin.set_low();
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


//===== Definitions for PWM audio nonsense ===

const AUDIO_PWM_DIVISOR: u8 = 64;

fn calc_note(freq: f32) -> u16 {
  const SPLAT_FACTOR: f32 = 12_000_000 as f32 / AUDIO_PWM_DIVISOR as f32;
  (SPLAT_FACTOR / freq) as u16
}

fn play_note(pwm: &mut Slice<Pwm3, FreeRunning>, note: (f32, u32, u32), delay: &mut Delay) {
  // const CLOCK_FACTOR: f32 = 125_000_000 as f32 / 40 as f32;
  if note.0 > 0.0 {
    let top = calc_note(note.0);
    pwm.channel_a.set_duty(top / 2); // 50% duty cycle
    pwm.set_top(top);
    delay.delay_ms(note.1);
    // silence
  }
  if note.2 > 0 {
    pwm.channel_a.set_duty(0);
    delay.delay_ms(note.2);
  }
}

fn play_tune(tune: &[SimpleNote], pwm: &mut Slice<Pwm3, FreeRunning>, delay: &mut Delay) {
  println!("play tune: {}", tune.len());
  for note in tune {
    play_note(pwm, *note, delay);
  }
}

type NoteFrequencyHz = f32;
type SimpleNote = (NoteFrequencyHz, u32, u32);

const FREQ_G3: NoteFrequencyHz = 196.00;
const FREQ_B3: NoteFrequencyHz =  246.942;

const FREQ_C4: NoteFrequencyHz = 261.63;
const FREQ_D4: NoteFrequencyHz = 293.66;
const FREQ_E4: NoteFrequencyHz = 329.63;
const FREQ_F4: NoteFrequencyHz = 349.23;
const FREQ_F4_SHARP: NoteFrequencyHz = 369.994;
const FREQ_G4: NoteFrequencyHz = 392.00;
const FREQ_G4_SHARP: NoteFrequencyHz = 392.00;
const FREQ_A4: NoteFrequencyHz = 440.00;
const FREQ_B4: NoteFrequencyHz = 493.88;
const FREQ_C5: NoteFrequencyHz = 523.25;
const FREQ_D5: NoteFrequencyHz = 587.33;

//G♯4, F♯4,

const G3: SimpleNote = (FREQ_G3, BEAT, STANDARD_PAUSE);
const B3: SimpleNote = (FREQ_B3, BEAT, STANDARD_PAUSE);
const C4: SimpleNote = (FREQ_C4, BEAT, STANDARD_PAUSE);
const D4: SimpleNote = (FREQ_D4, BEAT, STANDARD_PAUSE);
const E4: SimpleNote = (FREQ_E4, BEAT, STANDARD_PAUSE);
const F4: SimpleNote = (FREQ_F4, BEAT, STANDARD_PAUSE);
const FS4: SimpleNote = (FREQ_F4_SHARP, BEAT, STANDARD_PAUSE);
const G4: SimpleNote = (FREQ_G4, BEAT, STANDARD_PAUSE);
const GS4: SimpleNote = (FREQ_G4_SHARP, BEAT, STANDARD_PAUSE);
const A4: SimpleNote = (FREQ_A4, BEAT, STANDARD_PAUSE);
const B4: SimpleNote = (FREQ_B4, BEAT, STANDARD_PAUSE);
const C5: SimpleNote = (FREQ_C5, BEAT, STANDARD_PAUSE);
const D5: SimpleNote = (FREQ_D5, BEAT, STANDARD_PAUSE);


const TUNE_BPM: u8 = 83; //166 alternate
const TUNE_BPS: f32 = TUNE_BPM as f32 /60 as f32;
const MS_PER_BEAT: u32 = (1000f32/TUNE_BPS) as u32;

const BEAT:u32 = MS_PER_BEAT;
const HALF_BEAT:u32 = BEAT/2;
const QTR_BEAT:u32 = BEAT/4;
const EIGHTH_BEAT:u32 = BEAT/8;
const FOUR_BEAT:u32 = BEAT*4;
const TWO_BEAT:u32 = BEAT*2;
const STANDARD_PAUSE:u32 = 10;
const HALF_PAUSE:u32 = STANDARD_PAUSE/2;
const SILENCIO: SimpleNote = (0.0, 0, BEAT);

const DO_RE_MI_TUNE: [SimpleNote; 8] = [C4, D4, E4, F4, G4, A4, B4, C5, ];

const HOURLY_CHIME: [SimpleNote; 9] = [
  // TODO big-ben-ify
  E4, C4, D4, (FREQ_G3, TWO_BEAT, STANDARD_PAUSE),
  SILENCIO,
  C4, D4, E4, (FREQ_C4, TWO_BEAT, STANDARD_PAUSE)
];

const HALF_HOUR_CHIME: [SimpleNote; 8] = [
  // Big Ben Style
  // E4, G♯4, F♯4, B3
  // E4, F♯4, G♯4, E4
  E4, GS4, FS4, (FREQ_B3, TWO_BEAT, STANDARD_PAUSE),
  E4, FS4, GS4, (FREQ_E4, TWO_BEAT, STANDARD_PAUSE),
];


/*
C4 F4 E4 F4 A4 G4 F4 G4
A4 G4 F4 F4 A4 C5 D5
D5 C5 A4 A4 F4 G4 F4 G4 A4
F4 D4 D4 C4 F4
-
D5 C5 A4 A4 F4 G4 F4 G4 D5 C5
A4 A4 C5 D5 D5 C5 A4 A4
F4 G4 F4 G4 A4 G4 F4 D4 D4 C4 F4
-
D5 C5 A4 A4 F4 G4 F4 G4 D5 C5
A4 A4 C5 D5 D5 C5 A4 A4
F4 G4 F4 G4 A4 G4 F4 D4 D4 C4 F4
 */
const AULD_LANG_SYNE_VERSE1: [SimpleNote; 32] = [
  C4, //(FREQ_C4, TWO_BEAT, STANDARD_PAUSE),
  F4,
  (FREQ_E4, HALF_BEAT, STANDARD_PAUSE),
  F4, A4, G4,
  (FREQ_F4, HALF_BEAT, STANDARD_PAUSE),
  G4,
  A4, G4,
  (FREQ_F4, HALF_BEAT, STANDARD_PAUSE),
  F4, A4, C5, (FREQ_D5, TWO_BEAT, STANDARD_PAUSE),
  SILENCIO,
  D5, C5, A4, A4, F4, G4, F4, G4, A4,
  F4, D4, D4, C4, (FREQ_F4, TWO_BEAT, STANDARD_PAUSE),
  SILENCIO, SILENCIO,
];

const AULD_LANG_SYNE_VERSE2: [SimpleNote; 32] = [
  D5, C5,
  (FREQ_A4, HALF_BEAT, STANDARD_PAUSE),
  A4, F4, G4,
  (FREQ_F4, HALF_BEAT, STANDARD_PAUSE),
  G4,
  D5, C5,
  (FREQ_A4, HALF_BEAT, STANDARD_PAUSE),
  A4, C5,  (FREQ_D5, TWO_BEAT, STANDARD_PAUSE),
  SILENCIO,
  D5, C5, A4, A4,
  F4, G4, F4, G4, A4, G4,
  F4, D4, D4, C4, (FREQ_F4, TWO_BEAT, STANDARD_PAUSE),
  SILENCIO, SILENCIO,
];



