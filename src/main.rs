
#![no_std]
#![no_main]
use rp_pico::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use fugit::{ RateExtU32};

use rp_pico as bsp;
use embedded_hal::digital::v2::{OutputPin};
// use rp2040_hal as p_hal;
use bsp::hal as p_hal;


use bsp::hal::{
  clocks::{init_clocks_and_plls, Clock},
  pac,
  sio::Sio,
  // Timer,
  watchdog::Watchdog,
};

use embedded_graphics::{
  pixelcolor::BinaryColor::On as Black,
  // pixelcolor::BinaryColor::Off as White,
  prelude::*,
  // draw_target::DrawTarget,
  primitives::{Line, PrimitiveStyle},
};
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};

use embedded_graphics_core::geometry::OriginDimensions;
use embedded_graphics_core::pixelcolor::BinaryColor;


use epd_waveshare::{
  prelude::*,
  epd2in9bc::*,
  epd2in9_v2::*,
  // graphics::*,
  graphics::{DisplayRotation, TriDisplay},

};


type DisplayType = Display2in9bc;
// type EpaperType = Epd2in9bc;

#[entry]
fn mainish() -> ! {

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

  // let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
  //let mut count_down = timer.count_down();

  //let mut clocko =  Rp2040Monotonic ::new(pac.TIMER);
  let mut led_pin = pins.led.into_push_pull_output();

  // let spi0_sck_pin = pins.gpio6.into_mode::<bsp::hal::gpio::FunctionSpi>();
  // let spi0_do_pin = pins.gpio7.into_mode::<bsp::hal::gpio::FunctionSpi>();
  let spi0_sck_pin = pins.gpio6.into_function::<bsp::hal::gpio::FunctionSpi>();
  let spi0_do_pin = pins.gpio7.into_function::<bsp::hal::gpio::FunctionSpi>();

  // rst_pin is used to reset the OLED display during power-on sequence
  let mut rst_pin = pins.gpio10.into_push_pull_output();
  rst_pin.set_high().unwrap();
  delay.delay_us(200);
  rst_pin.set_low().unwrap();

  let busy_in = pins.gpio11.into_floating_input();

  let dc_pin  = pins.gpio8.into_push_pull_output();
  let cs_pin = pins.gpio9.into_push_pull_output();
  // let spi0 = p_hal::Spi::<_, _, 8>::new(pac.SPI0);

  // (Tx, Rx, Sck) (MOSI, MISO, SCK)
  // (Tx, Sck) (MOSI, SCK)
  let mut spi0 = p_hal::Spi::<_, _, _, 8>::new(pac.SPI0, (spi0_do_pin, spi0_sck_pin) );

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

  epd.set_border_color(&mut spi, TriColor::Chromatic).unwrap();
  println!("create display...");
  // Use display graphics from embedded-graphics
  let mut display = DisplayType::default();
  let disp_size = display.size();
  println!("size : {} x {}", display.size().width,display.size().height);
  display.clear_buffer(Color::White);



  println!("create lines...");
  // Use embedded graphics for drawing a line
  let high_line =
    Line::new(Point::new(5, 5),
              Point::new((disp_size.width -5 ) as i32 ,
                         (disp_size.height -5 ) as i32 )
    ).into_styled(PrimitiveStyle::with_stroke(Black, 2));

  let low_line =
    Line::new(Point::new(5, (disp_size.height-5)as i32),
              Point::new((disp_size.width -5 ) as i32 ,
                         5 )
    ).into_styled(PrimitiveStyle::with_stroke(Black, 2));

  let fg_char_style = MonoTextStyleBuilder::new()
    .font(&embedded_graphics::mono_font::ascii::FONT_6X13_BOLD)
    .text_color(Black)
    // .background_color(BinaryColor::Off )
    .build();

  let bg_char_style = MonoTextStyleBuilder::new()
    .font(&embedded_graphics::mono_font::ascii::FONT_6X13_BOLD)
    .text_color(BinaryColor::Off)
    .build();

  let align_style = TextStyleBuilder::new()
    .baseline(Baseline::Bottom)
    .alignment(Alignment::Left)
    .build();

  let label_text =
    Text::with_text_style(
      "ALL ONE!", Point::new(5, 20), fg_char_style, align_style);

  let sublabel_text_fg =
    Text::with_text_style(
      "BE ONE!!!", Point::new(5, 150), fg_char_style, align_style);
  let sublabel_text_bg =
    Text::with_text_style(
      "BE ONE!!!", Point::new(5, 150), bg_char_style, align_style);


  println!("enter loop...");

  let mut loop_count = 0;
  loop {
    let _ = led_pin.set_high();
    display.clear_buffer(Color::White);
    println!("{}",loop_count);
    // high_line.draw(&mut display).unwrap();
    // low_line.draw(&mut display).unwrap();
    label_text.draw(&mut display).unwrap();

    if (loop_count / 2) == 0 {
      sublabel_text_fg.draw(&mut display).unwrap();
    }
    else {
      sublabel_text_bg.draw(&mut display).unwrap();
    }

    // push changes to display
    epd.update_frame(&mut spi, &display.buffer(), &mut delay).unwrap();
    epd.display_frame(&mut spi, &mut delay).unwrap();
    epd.sleep(&mut spi, &mut delay).unwrap();
    let _ = led_pin.set_low();
    delay.delay_ms(2000);
    loop_count += 1;
  }
}


