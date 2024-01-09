
#![no_std]
#![no_main]

use core::ops::{Add, Sub};
use rp_pico::{entry};
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use fugit::{ RateExtU32};

use rp_pico as bsp;
use embedded_hal::{
  digital::v2::{OutputPin},
  PwmPin,
  // blocking::i2c::{Write, Read, WriteRead}
};

use bsp::hal as p_hal;
use numtoa::NumToA;
use arraystring::{ArrayString, typenum::{U40} };

use rv3028c7_rtc::{RV3028, DateTimeAccess, Duration, NaiveDateTime,  Datelike, Timelike, Weekday};

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


use embedded_graphics::{prelude::*, mono_font::MonoTextStyleBuilder, text::{Alignment, Baseline, Text, TextStyleBuilder}, mono_font, geometry};
use embedded_graphics::image::ImageRaw;
use embedded_graphics::mono_font::mapping::StrGlyphMapping;
// use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::text::TextStyle;

use embedded_graphics_core::{
  geometry::OriginDimensions,
  pixelcolor::BinaryColor,
};

use epd_waveshare::{prelude::*,
                    graphics::{DisplayRotation},
                    epd2in13_v2::{Epd2in13, Display2in13},
};

use rp_pico::hal::gpio::Pin;

type DisplayType = Display2in13;



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

fn naivedatetime_to_hms(dt: &NaiveDateTime) -> rtc::DateTime {
  rtc::DateTime {
    year: 2024,
    month: 1,
    day: 1,
    day_of_week: rtc::DayOfWeek::Monday,
    hour: dt.hour() as u8,
    minute: dt.minute() as u8,
    second: dt.second() as u8,
  }
}

const EXTERNAL_XTAL_FREQ_HZ:u32 = 12_000_000u32;


pub fn draw_text(display: &mut DisplayType, text: &str, x: i32, y: i32) {
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
  let mut sio = Sio::new(pac.SIO);

  // External high-speed crystal on the pico board is 12Mhz
  let clocks = init_clocks_and_plls(
    EXTERNAL_XTAL_FREQ_HZ,
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

  //Pin<FunctionPwm,PullNone,Gp22Pwm3>
  // let pwm_pin: Pin<_, _, _> =
  pins.gpio22.into_function::<gpio::FunctionPwm>();


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
  let mut sys_rtc = p_hal::rtc::RealTimeClock::new(
    pac.RTC,
    clocks.rtc_clock,
    &mut pac.RESETS,
    sys_dt,
  ).unwrap();

  let mut led_pin = pins.led.into_push_pull_output();
  led_pin.set_high().unwrap();



  // queue_tune(&DO_RE_MI_TUNE_SHORT, &mut sio.fifo);

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

  // for rendering strings
  let mut text_buf =  ArrayString::<U40>::new();

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

  epd.wake_up(&mut spi, &mut delay).expect("EPD wake_up fail");
  epd.clear_frame(&mut spi, &mut delay).expect("EPD clear_frame fail");

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

  display.set_rotation(DisplayRotation::Rotate270);
  draw_text(&mut display, "270: Todd Stellanova !", min_dim, min_dim);

  epd.update_and_display_frame(&mut spi, display.buffer(), &mut delay).expect("update fail");

  led_pin.set_low().unwrap();

  let fg_char_style = MonoTextStyleBuilder::new()
    .font(&embedded_graphics::mono_font::ascii::FONT_10X20)
    .text_color(BinaryColor::On)
    .background_color(BinaryColor::Off )
    .build();

  let clock_char_style = MonoTextStyleBuilder::new()
    .font(&CLOCK_FONT)
    .text_color(BinaryColor::On)
    .background_color(BinaryColor::Off )
    .build();

  const ALIGN_STYLE: TextStyle = TextStyleBuilder::new()
    .baseline(Baseline::Middle)
    .alignment(Alignment::Center)
    .build();

  const TEXT_FONT_HEIGHT: i32 = 20;
  let ctr_point =  Point::new(disp_width/2 , disp_height/2);
  println!("ctr_point.y: {}", ctr_point.y);
  let time_point = Point::new(ctr_point.x, 20);
  let wd_point = ctr_point.add(Point::new(0,25));
  let date_point = wd_point.add(Point::new(0,TEXT_FONT_HEIGHT));

  // use Landscape mode, with the RPico pin 1 "up"
  display.set_rotation(DisplayRotation::Rotate90);

  // unmask RTC_IRQ interrupts so we can process them
  unsafe {
    pac::NVIC::unmask(pac::Interrupt::RTC_IRQ);
  }

  let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
  let cores = mc.cores();
  let core1 = &mut cores[1];
  core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
    core1_task(clocks.system_clock.freq().to_Hz())
  }).unwrap();

  queue_note(&C4, &mut sio.fifo);

  println!("enter loop...");
  let mut last_minute = 99;
  loop {
    if let Ok(rtc_dt) = rtc.datetime() {
      if rtc_dt.minute() == last_minute {
        sys_rtc.set_datetime( naivedatetime_to_hms(&rtc_dt)).unwrap();
        // don't refresh until the next minute
        println!("wait..{:02}:{:02}", rtc_dt.minute(), rtc_dt.second());
        delay.delay_ms(100);
        continue;
      }
      let _ = led_pin.set_high();
      last_minute = rtc_dt.minute();

      // get time from external RTC
      // let rtc_dt = rtc.datetime().unwrap();
      let local_dt = rtc_dt.sub(Duration::hours(8));

      println!("check: {:02}:{:02}:{:02}",
               local_dt.time().hour() , local_dt.time().minute(), local_dt.time().second());

      display.clear_buffer(Color::White);

      format_time(&local_dt,&mut text_buf);
      let time_text =
        Text::with_text_style(
          &text_buf,
          time_point,
          clock_char_style, ALIGN_STYLE);
      time_text.draw(&mut display).unwrap();

      format_weekday(&local_dt, &mut text_buf);
      let wd_text =
        Text::with_text_style(
          &text_buf,
          wd_point,
          fg_char_style, ALIGN_STYLE);
      wd_text.draw(&mut display).unwrap();

      format_date(&local_dt,&mut text_buf);
      let date_text =
        Text::with_text_style(
          &text_buf,
          date_point,
          fg_char_style, ALIGN_STYLE);
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
      // let final_dt = sys_rtc.now().unwrap();
      // println!("tdelta {}", final_dt.second - draw_dt.second);

      // put display into low power mode
      epd.sleep(&mut spi, &mut delay).unwrap();

      if local_dt.day() == 1 && local_dt.month() == 1 &&
        local_dt.hour() == 0 && local_dt.minute() == 0 {
        queue_tune(&AULD_LANG_SYNE_VERSE1, &mut sio.fifo);
        queue_tune(&AULD_LANG_SYNE_VERSE2, &mut sio.fifo);
        queue_tune(&AULD_LANG_SYNE_VERSE2, &mut sio.fifo);
      }
      else if local_dt.minute() == 0 {
        queue_tune(&HOURLY_CHIME, &mut sio.fifo);
      }
      else if local_dt.minute() % 30 == 0 {
        queue_tune(&HALF_HOUR_CHIME, &mut sio.fifo);
      }
      else {
        queue_note(&FAST_PIP,  &mut sio.fifo);
      }

      sys_rtc.schedule_alarm(rtc::DateTimeFilter::default().second(0));
      enable_rtc_interrupt();

      println!("sleep...");
      let _ = led_pin.set_low();
      cortex_m::asm::wfi();
      println!("awake...");

    }
    else {
      println!("datetime failed");
    }


  }

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

//==== Font used for time display
pub const CLOCK_FONT_HEIGHT: u32 = 60; //54; //48; //40;
pub const CLOCK_FONT_WIDTH: u32 = 32; //29; // 22

pub const CLOCK_FONT: crate::mono_font::MonoFont = crate::mono_font::MonoFont {
  image: ImageRaw::new_binary(
    include_bytes!("../../res/mplus_60h.raw"),
    379,
  ),
  glyph_mapping: &StrGlyphMapping::new("0123456789:!", 0),
  character_size: geometry::Size::new(CLOCK_FONT_WIDTH, CLOCK_FONT_HEIGHT),
  character_spacing: 0,
  baseline: 60,
  underline: mono_font::DecorationDimensions::default_underline(CLOCK_FONT_HEIGHT),
  strikethrough: mono_font::DecorationDimensions::default_strikethrough(CLOCK_FONT_HEIGHT),
};


//=== Interrupt manipulation ===

use crate::pac::interrupt;

//TODO rp2040-hal has enable_interrupt coming
fn enable_rtc_interrupt() {
  unsafe {
    (*pac::RTC::PTR).inte.modify(|_, w| w.rtc().set_bit());
  }
}

//TODO rp2040-hal has disable_interrupt coming
fn disable_rtc_interrupt() {
  unsafe {
    (*pac::RTC::PTR).inte.modify(|_, w| w.rtc().clear_bit());
  }
}


#[allow(non_snake_case)]
#[interrupt]
fn RTC_IRQ() {
  disable_rtc_interrupt();
}



//===== Definitions for PWM audio calculations ===

const AUDIO_PWM_DIVISOR: u8 = 64;

/// Calculate the pwm counter top value for a given audio frequency
fn calc_note_count(freq: f32) -> u16 {
  const SPLAT_FACTOR: f32 = (EXTERNAL_XTAL_FREQ_HZ as f32) / AUDIO_PWM_DIVISOR as f32;
  (SPLAT_FACTOR / freq) as u16
}

fn play_note(pwm: &mut Slice<Pwm3, FreeRunning>, delay: &mut Delay, note: SimpleNote) {
  // println!("play_note: {}", note);

  // const CLOCK_FACTOR: f32 = 125_000_000 as f32 / 40 as f32;
  if note.0 > 20.0 {
    let top = calc_note_count(note.0);
    pwm.channel_a.set_duty(top / 2); // 50% duty cycle
    pwm.set_top(top);
    // sustain
    // if note.1 > 0 {
      delay.delay_ms(note.1);
    // }
  }
  if note.2 > 0 {
    pwm.channel_a.set_duty(0);
    delay.delay_ms(note.2);
  }
}

fn play_tune(tune: &[SimpleNote], pwm: &mut Slice<Pwm3, FreeRunning>, delay: &mut Delay) {
  println!("play tune: {}", tune.len());
  for note in tune {
    play_note(pwm, delay, *note);
  }
}

fn queue_tune(tune: &[SimpleNote], fifo: &mut SioFifo)
{
  println!("queue tune: {}", tune.len());
  for note in tune {
    queue_note(note, fifo);
  }
}


fn queue_note(note: &SimpleNote, fifo: &mut SioFifo)
{
  // println!("queue_note: {}", note);
  let note_number: u8 =
    if let Some((index, _)) =
      FREQ_KEY_MAP.iter().enumerate().find(|&(_, &val)| val == note.0) {
      index as u8
    }
    else {
      0
    };

  let sustain_idx: u8 =
    if let Some((index, _)) =
      BEAT_MAP.iter().enumerate().find(|&(_, &val)| val == note.1) {
      index as u8
    }
    else {
      3
    };

  let send_buf = [
    0, // MAGIC
    note_number,
    sustain_idx,
    (note.2 % 256) as u8
  ];
  let send_word:u32 = u32::from_ne_bytes(send_buf);
  fifo.write_blocking(send_word);

}

type NoteFrequencyHz = f32;
type SimpleNote = (NoteFrequencyHz, u32, u32);

const FREQ_B2: NoteFrequencyHz = 123.471;

const FREQ_E3: NoteFrequencyHz = 164.814;
const FREQ_F3: NoteFrequencyHz = 174.614;
const FREQ_F3_SHARP: NoteFrequencyHz = 184.997;
const FREQ_G3: NoteFrequencyHz = 195.998;
const FREQ_G3_SHARP: NoteFrequencyHz = 207.652;

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
const FREQ_BBC_TIME_PIP: NoteFrequencyHz = 1000.0;
const FREQ_FAST_PIP: NoteFrequencyHz  = 400.0;

const FREQ_KEY_MAP: [NoteFrequencyHz; 20] = [
  FREQ_B2,

  FREQ_E3 ,
  FREQ_F3 ,
  FREQ_F3_SHARP ,
  FREQ_G3 ,
  FREQ_G3_SHARP ,

  FREQ_B3 ,

  FREQ_C4 ,
  FREQ_D4 ,
  FREQ_E4 ,
  FREQ_F4 ,
  FREQ_F4_SHARP ,
  FREQ_G4 ,
  FREQ_G4_SHARP ,
  FREQ_A4,
  FREQ_B4 ,
  FREQ_C5 ,
  FREQ_D5 ,
  FREQ_BBC_TIME_PIP,
  FREQ_FAST_PIP,
];

pub const B2: SimpleNote = (FREQ_B2, BEAT, STANDARD_PAUSE);
pub const E3: SimpleNote = (FREQ_E3, BEAT, STANDARD_PAUSE);
pub const F3: SimpleNote = (FREQ_F3, BEAT, STANDARD_PAUSE);
pub const FS3: SimpleNote = (FREQ_F3_SHARP, BEAT, STANDARD_PAUSE);
pub const G3: SimpleNote = (FREQ_G3, BEAT, STANDARD_PAUSE);
pub const GS3: SimpleNote = (FREQ_G3_SHARP, BEAT, STANDARD_PAUSE);

pub const B3: SimpleNote = (FREQ_B3, BEAT, STANDARD_PAUSE);

pub const C4: SimpleNote = (FREQ_C4, BEAT, STANDARD_PAUSE);
pub const D4: SimpleNote = (FREQ_D4, BEAT, STANDARD_PAUSE);
pub const E4: SimpleNote = (FREQ_E4, BEAT, STANDARD_PAUSE);
pub const F4: SimpleNote = (FREQ_F4, BEAT, STANDARD_PAUSE);
pub const FS4: SimpleNote = (FREQ_F4_SHARP, BEAT, STANDARD_PAUSE);
pub const G4: SimpleNote = (FREQ_G4, BEAT, STANDARD_PAUSE);
pub const GS4: SimpleNote = (FREQ_G4_SHARP, BEAT, STANDARD_PAUSE);
pub const A4: SimpleNote = (FREQ_A4, BEAT, STANDARD_PAUSE);
pub const B4: SimpleNote = (FREQ_B4, BEAT, STANDARD_PAUSE);
pub const C5: SimpleNote = (FREQ_C5, BEAT, STANDARD_PAUSE);
pub const D5: SimpleNote = (FREQ_D5, BEAT, STANDARD_PAUSE);

pub const FAST_PIP: SimpleNote = (FREQ_FAST_PIP, QTR_BEAT, STANDARD_PAUSE);
pub const BBC_TIME_PIP: SimpleNote = (FREQ_BBC_TIME_PIP, 100, 900);
pub const BBC_TIME_LONG_PIP: SimpleNote = (FREQ_BBC_TIME_PIP, 500, STANDARD_PAUSE);


const TUNE_BPM: u8 = 83; //166 alternate
const TUNE_BPS: f32 = TUNE_BPM as f32 /60 as f32;
const MS_PER_BEAT: u32 = (1000f32/TUNE_BPS) as u32;

pub const BEAT:u32 = MS_PER_BEAT;
pub const HALF_BEAT:u32 = BEAT/2;
pub const QTR_BEAT:u32 = BEAT/4;
pub const EIGHTH_BEAT:u32 = BEAT/8;
pub const TWO_BEAT:u32 = BEAT*2;
pub const FOUR_BEAT:u32 = BEAT*4;
pub const EIGHT_BEAT:u32 = BEAT*8;
pub const BEAT_MAP: [u32; 7] = [
  EIGHTH_BEAT,
  QTR_BEAT,
  HALF_BEAT,
  BEAT,
  TWO_BEAT,
  FOUR_BEAT,
  EIGHT_BEAT
];

pub const STANDARD_PAUSE:u32 = 10;
pub const HALF_PAUSE:u32 = STANDARD_PAUSE/2;
pub const SILENCIO: SimpleNote = (0.0, 0, BEAT);


pub const DO_RE_MI_TUNE: [SimpleNote; 8] = [C4, D4, E4, F4, G4, A4, B4, C5, ];
pub const DO_RE_MI_TUNE_SHORT: [SimpleNote; 3] = [C4, D4, E4 ];

pub const BBC_HOUR_PIPS: [SimpleNote; 6] =
  [BBC_TIME_PIP, BBC_TIME_PIP, BBC_TIME_PIP, BBC_TIME_PIP, BBC_TIME_PIP, BBC_TIME_LONG_PIP];

pub const BIG_BEN_NOTE_SHORT: SimpleNote = (FREQ_E3, BEAT, STANDARD_PAUSE);

pub const HOURLY_CHIME: [SimpleNote; 8] = [
  // TODO big-ben-ify?
  E4, C4, D4, (FREQ_G3, TWO_BEAT, STANDARD_PAUSE),
  C4, D4, E4, (FREQ_C4, TWO_BEAT, STANDARD_PAUSE)
];

// E4, G♯4, F♯4, B3
// E4, F♯4, G♯4, E4

pub const HALF_HOUR_CHIME: [SimpleNote; 8] = [
  // Big Ben Style, quoted as:
  // E4, G♯4, F♯4, B3
  // E4, F♯4, G♯4, E4
  E3, GS3, FS3, (FREQ_B2, TWO_BEAT, STANDARD_PAUSE),
  E3, FS3, GS3, (FREQ_E3, TWO_BEAT, STANDARD_PAUSE)
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


// ==== multicore support ===
use p_hal::multicore::{Multicore, Stack};
use rp_pico::hal::gpio;
use rp_pico::hal::sio::SioFifo;

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static values is
/// reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything seperately and
/// modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte alignment, which allows
/// the stack guard to take up the least amount of usable RAM.
static mut CORE1_STACK: Stack<4096> = Stack::new();

/// The main task of the second core, "Core 1"
fn core1_task(sys_freq: u32) -> ! {
  println!("core1_task start... ");
  // let core = pac::CorePeripherals::take().unwrap();
  let core = unsafe { pac::CorePeripherals::steal() };

  // let mut le_pac = pac::Peripherals::take().unwrap();
  let mut le_pac = unsafe { pac::Peripherals::steal() };

  let mut sio = Sio::new(le_pac.SIO);

  let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);



  // println!("setup PWM player: {} ms per beat", MS_PER_BEAT);
  let mut pwm_slices = p_hal::pwm::Slices::new(le_pac.PWM, &mut le_pac.RESETS);

  // Configure PWM3
  let mut pwm = &mut pwm_slices.pwm3;
  pwm.set_ph_correct();
  pwm.enable();

  let channel = &mut pwm.channel_a;
  channel.set_duty(0);//initially off

  // Create PWM driver on GPIO22 / "GP22" pin

  // let pins = bsp::Pins::new(
  //   le_pac.IO_BANK0,
  //   le_pac.PADS_BANK0,
  //   sio.gpio_bank0,
  //   &mut le_pac.RESETS,
  // );
  //  let silly_pin = pins.gpio22.into_function::<gpio::FunctionPwm>();

  // Configure GPIO 22 as an output
  let silly_pin:  gpio::Pin<gpio::bank0::Gpio22, gpio::FunctionPwm, gpio::PullNone> =  unsafe { core::mem::zeroed() };
  channel.output_to(silly_pin);

  pwm.set_div_int(AUDIO_PWM_DIVISOR); // To set integer part of clock divider
  pwm.set_div_frac(0u8); // To set fractional part of clock divider

  // println!("test notes");
  // play_note( &mut pwm,  G3, &mut delay);
  // play_tune(&DO_RE_MI_TUNE_SHORT, &mut pwm, &mut delay);
  // play_tune(&HOURLY_CHIME,&mut pwm, &mut delay);


  loop {
    let word = sio.fifo.read_blocking();
    let msg_bytes:[u8; 4] = word.to_ne_bytes();
    if msg_bytes[0] == 0 { // MAGIC byte
      let le_note: SimpleNote = (
        FREQ_KEY_MAP[msg_bytes[1] as usize],
        BEAT_MAP[msg_bytes[2] as usize] as u32,
        // msg_bytes[2] as u32,
        msg_bytes[3] as u32
      );
      println!("le_note: {}", le_note);
      play_note(&mut pwm, &mut delay, le_note);
    }

  }

}

