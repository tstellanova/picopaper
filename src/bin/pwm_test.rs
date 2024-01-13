
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


use rp_pico::hal::gpio::Pin;



const EXTERNAL_XTAL_FREQ_HZ:u32 = 12_000_000u32;


type PwmPinType = Pin<gpio::bank0::Gpio22, gpio::FunctionPwm, gpio::PullDown>;



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


  let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

  let pins = bsp::Pins::new(
    pac.IO_BANK0,
    pac.PADS_BANK0,
    sio.gpio_bank0,
    &mut pac.RESETS,
  );

  let silly_pin:PwmPinType = pins.gpio22.into_function::<gpio::FunctionPwm>();

  let mut led_pin = pins.led.into_push_pull_output();
  led_pin.set_high().unwrap();



  led_pin.set_low().unwrap();


  let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
  let cores = mc.cores();
  let core1 = &mut cores[1];
  core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
    core1_task(clocks.system_clock.freq().to_Hz(), silly_pin)
  }).unwrap();

  queue_tune(&DO_RE_MI_TUNE_SHORT, &mut sio.fifo);


  println!("enter loop...");
  let mut target_second = 0;
  loop {
    delay.delay_ms(3000);
    let _ = led_pin.set_high();
    queue_tune(&HALF_HOUR_CHIME, &mut sio.fifo);
    let _ = led_pin.set_low();
    delay.delay_ms(3000);
    let _ = led_pin.set_high();
    queue_tune(&HOURLY_CHIME, &mut sio.fifo);
    let _ = led_pin.set_low();
  }

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
  pwm.channel_a.set_duty(0);
  if note.2 > 0 {
    delay.delay_ms(note.2);
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

pub const FAST_PIP: SimpleNote = (FREQ_FAST_PIP, EIGHTH_BEAT, HALF_PAUSE);
pub const BBC_TIME_PIP: SimpleNote = (FREQ_BBC_TIME_PIP, 100, 0);
pub const BBC_TIME_LONG_PIP: SimpleNote = (FREQ_BBC_TIME_PIP, 500, 0);


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
pub const BEAT_MAP: [u32; 10] = [
  EIGHTH_BEAT,
  QTR_BEAT,
  HALF_BEAT,
  BEAT,
  TWO_BEAT,
  FOUR_BEAT,
  EIGHT_BEAT,
  100,
  500,
  900
];

pub const STANDARD_PAUSE:u32 = 10;
pub const HALF_PAUSE:u32 = STANDARD_PAUSE/2;
pub const SILENCIO: SimpleNote = (0.0, 0, BEAT);


pub const DO_RE_MI_TUNE: [SimpleNote; 8] = [C4, D4, E4, F4, G4, A4, B4, C5, ];
pub const DO_RE_MI_TUNE_SHORT: [SimpleNote; 3] = [C4, D4, E4 ];


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
fn core1_task(sys_freq: u32, silly_pin: PwmPinType) -> ! {
  println!("core1_task start... ");
  // This isn't really stealing -- core1 has its own copies of
  // the same peripherals as core0. Someday svd2rust may support this safely.
  let core = unsafe { pac::CorePeripherals::steal() };
  let mut pac = unsafe { pac::Peripherals::steal() };

  let mut sio = Sio::new(pac.SIO);
  let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);

  // println!("setup PWM player: {} ms per beat", MS_PER_BEAT);
  let mut pwm_slices = p_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

  // Configure PWM3
  let mut pwm = &mut pwm_slices.pwm3;
  pwm.set_ph_correct();
  pwm.enable();

  let channel = &mut pwm.channel_a;
  channel.set_duty(0);//initially off

  // Create PWM driver on GPIO22 / "GP22" pin

  // TODO creating a Pins struct here appears to cause multicore problems
  // let pins = bsp::Pins::new(
  //   le_pac.IO_BANK0,
  //   le_pac.PADS_BANK0,
  //   sio.gpio_bank0,
  //   &mut le_pac.RESETS,
  // );
  // let silly_pin = pins.gpio22.into_function::<gpio::FunctionPwm>();

  // Hack to grab uninit  GPIO 22 as an output
  // let silly_pin:  gpio::Pin<gpio::bank0::Gpio22, gpio::FunctionPwm, gpio::PullNone> =  unsafe { core::mem::zeroed() };
  channel.output_to(silly_pin);

  pwm.set_div_int(AUDIO_PWM_DIVISOR); // To set integer part of clock divider
  pwm.set_div_frac(0u8); // To set fractional part of clock divider

  loop {
    let word = sio.fifo.read_blocking();
    let msg_bytes:[u8; 4] = word.to_ne_bytes();
    if msg_bytes[0] == 0 { // MAGIC byte
      let le_note: SimpleNote = (
        FREQ_KEY_MAP[msg_bytes[1] as usize],
        BEAT_MAP[msg_bytes[2] as usize] as u32,
        msg_bytes[3] as u32
      );
      println!("le_note: {}", le_note);
      play_note(&mut pwm, &mut delay, le_note);
    }

  }

}

