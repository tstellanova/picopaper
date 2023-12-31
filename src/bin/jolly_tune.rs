
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
use cortex_m::delay::Delay;


use p_hal::{
  Clock,
  clocks::{init_clocks_and_plls},
  pac::{self},
  sio::Sio,
  watchdog::Watchdog,
  rtc::{self, DayOfWeek},
  gpio::{FunctionI2C, Pin, PullUp},
  pwm::{FreeRunning, Pwm3, Slice},

};










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
  let _i2c = p_hal::I2C::i2c0(
    pac.I2C0,
    sda_pin,
    scl_pin,
    400.kHz(),
    &mut pac.RESETS,
    &clocks.system_clock,
  );

  let mut led_pin = pins.led.into_push_pull_output();

  let spi0_sck_pin = pins.gpio6.into_function::<bsp::hal::gpio::FunctionSpi>();
  let spi0_do_pin = pins.gpio7.into_function::<bsp::hal::gpio::FunctionSpi>();

  // rst_pin is used to reset the display during power-on sequence
  let mut rst_pin = pins.gpio10.into_push_pull_output();
  rst_pin.set_high().unwrap();
  delay.delay_us(200);
  rst_pin.set_low().unwrap();

  let busy_in = pins.gpio11.into_floating_input();

  let dc_pin = pins.gpio8.into_push_pull_output();
  let cs_pin = pins.gpio9.into_push_pull_output();

  // (Tx, Sck) (MOSI, SCK)
  let spi0 = p_hal::Spi::<_, _, _, 8>::new(pac.SPI0, (spi0_do_pin, spi0_sck_pin));

  println!("create SPI...");
  // Exchange the uninitialized SPI driver for an initialized one
  let mut _spi = spi0.init(
    &mut pac.RESETS,
    clocks.peripheral_clock.freq(),
    16u32.MHz(),
    //&embedded_hal::spi::MODE_0,
    &embedded_hal::spi::MODE_3,
  );

  let _ = led_pin.set_high();

  println!("setup PWM");
  // Init PWMs
  let mut pwm_slices = p_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

  // Configure PWM3
  let mut pwm = &mut pwm_slices.pwm3;
  pwm.set_ph_correct();
  pwm.enable();

  // Create PWM driver on GPIO22 / "GP22" pin
  let channel = &mut pwm.channel_a;
  channel.output_to(pins.gpio22);
  channel.set_duty(0);

  pwm.set_div_int(AUDIO_PWM_DIVISOR); // To set integer part of clock divider
  pwm.set_div_frac(0u8); // To set fractional part of clock divider

  println!("play notes");
  play_tune(&DO_RE_MI_TUNE, &mut pwm, &mut delay);

  delay.delay_ms(1000);
  println!("bpm: {} bps: {} beat ms: {} ",
           TUNE_BPM, TUNE_BPS, BEAT
  );
  play_tune(&AULD_LANG_SYNE_VERSE1, &mut pwm, &mut delay);
  play_tune(&AULD_LANG_SYNE_VERSE2, &mut pwm, &mut delay);
  play_tune(&AULD_LANG_SYNE_VERSE2, &mut pwm, &mut delay);

  println!("enter loop...");
  loop {}
}

//===== Definitions for PWM audio nonsense ===

const AUDIO_PWM_DIVISOR: u8 = 128;

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
  for note in tune {
    play_note(pwm, *note, delay);
  }
}

type NoteFrequencyHz = f32;
type SimpleNote = (NoteFrequencyHz, u32, u32);

const FREQ_C4: NoteFrequencyHz = 261.63;
const FREQ_D4: NoteFrequencyHz = 293.66;
const FREQ_E4: NoteFrequencyHz = 329.63;
const FREQ_F4: NoteFrequencyHz = 349.23;
const FREQ_G4: NoteFrequencyHz = 392.00;
const FREQ_A4: NoteFrequencyHz = 440.00;
const FREQ_B4: NoteFrequencyHz = 493.88;
const FREQ_C5: NoteFrequencyHz = 523.25;
const FREQ_D5: NoteFrequencyHz = 587.33;

const C4: SimpleNote = (FREQ_C4, BEAT, STANDARD_PAUSE);
const D4: SimpleNote = (FREQ_D4, BEAT, STANDARD_PAUSE);
const E4: SimpleNote = (FREQ_E4, BEAT, STANDARD_PAUSE);
const F4: SimpleNote = (FREQ_F4, BEAT, STANDARD_PAUSE);
const G4: SimpleNote = (FREQ_G4, BEAT, STANDARD_PAUSE);
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




