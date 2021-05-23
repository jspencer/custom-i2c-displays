#![no_main]
#![no_std]

use core::panic::PanicInfo;
use cortex_m_rt::entry;

use embedded_hal::digital::v2::OutputPin;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{Output, PushPull, Pxx},
    prelude::*,
};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let core = cortex_m::Peripherals::take().unwrap();
    let device = stm32f1xx_hal::stm32::Peripherals::take().unwrap();
    let mut rcc = device.RCC.constrain();
    let mut flash = device.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let gpioa = device.GPIOA;
    let mut g_parts = gpioa.split(&mut rcc.apb2);
    let mut leds = [
        g_parts
            .pa0
            .into_push_pull_output(&mut g_parts.crl)
            .downgrade(),
        g_parts
            .pa1
            .into_push_pull_output(&mut g_parts.crl)
            .downgrade(),
        g_parts
            .pa2
            .into_push_pull_output(&mut g_parts.crl)
            .downgrade(),
        g_parts
            .pa3
            .into_push_pull_output(&mut g_parts.crl)
            .downgrade(),
        g_parts
            .pa4
            .into_push_pull_output(&mut g_parts.crl)
            .downgrade(),
        g_parts
            .pa5
            .into_push_pull_output(&mut g_parts.crl)
            .downgrade(),
        g_parts
            .pa6
            .into_push_pull_output(&mut g_parts.crl)
            .downgrade(),
        g_parts
            .pa7
            .into_push_pull_output(&mut g_parts.crl)
            .downgrade(),
    ];

    let mut delay = Delay::new(core.SYST, clocks);

    for i in (0..8).cycle().take(32) {
        delay.delay_ms(80_u8);
        output_leds(&mut leds, 1 << i);
    }

    let mut i = (0..=255).cycle();
    loop {
        delay.delay_ms(40_u8);
        output_leds(&mut leds, i.next().unwrap());
    }
}

fn output_leds(leds: &mut [Pxx<Output<PushPull>>; 8], value: u8) {
    for (i, led) in leds.iter_mut().enumerate() {
        if (value >> i) & 1 > 0 {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {}
}
