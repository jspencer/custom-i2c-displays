#![no_main]
#![no_std]

use core::panic::PanicInfo;

use embedded_hal::digital::v2::OutputPin;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1::stm32f103::Interrupt;
use stm32f1xx_hal::{
    gpio::{Output, PushPull, Pxx},
    pac::I2C1,
    prelude::*,
    rcc::{Enable, Reset},
};

const OUR_I2C_ADDRESS: u8 = 0x33;

#[rtic::app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        i2c: I2C1,
        leds: [Pxx<Output<PushPull>>; 8],
    }
    #[init]
    fn init(c: init::Context) -> init::LateResources {
        rtt_init_print!();
        let device: stm32f1xx_hal::stm32::Peripherals = c.device;
        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();

        let _clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .hclk(24.mhz())
            .sysclk(24.mhz())
            .pclk1(12.mhz())
            .pclk2(12.mhz())
            .freeze(&mut flash.acr);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let leds = [
            gpioa.pa0.into_push_pull_output(&mut gpioa.crl).downgrade(),
            gpioa.pa1.into_push_pull_output(&mut gpioa.crl).downgrade(),
            gpioa.pa2.into_push_pull_output(&mut gpioa.crl).downgrade(),
            gpioa.pa3.into_push_pull_output(&mut gpioa.crl).downgrade(),
            gpioa.pa4.into_push_pull_output(&mut gpioa.crl).downgrade(),
            gpioa.pa5.into_push_pull_output(&mut gpioa.crl).downgrade(),
            gpioa.pa6.into_push_pull_output(&mut gpioa.crl).downgrade(),
            gpioa.pa7.into_push_pull_output(&mut gpioa.crl).downgrade(),
        ];

        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let _scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let _sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        I2C1::enable(&mut rcc.apb1);
        I2C1::reset(&mut rcc.apb1);

        let i2c = device.I2C1;

        // Disable the i2c peripheral while we set it up
        i2c.cr1.write(|w| w.pe().clear_bit());

        // Interrupt enable for I2C
        i2c.cr2.write(|w| w.itevten().set_bit());

        // Since we are using a 7-bit address we need to shift it into bits 1:7 of OAR1:ADD
        i2c.oar1
            .write(|w| w.add().bits((OUR_I2C_ADDRESS as u16) << 1));

        // Enable i2c peripheral
        i2c.cr1.modify(|_, w| w.pe().set_bit());

        // ACK must be set after the peripheral is enabled or it will be ignored
        i2c.cr1.modify(|_, w| w.ack().set_bit());

        rtic::pend(Interrupt::I2C1_EV);
        init::LateResources { i2c, leds }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = I2C1_EV, resources = [i2c, leds], priority = 2)]
    fn i2c1_ev(c: i2c1_ev::Context) {
        let i2c = c.resources.i2c;
        let leds = c.resources.leds;

        let sr1 = i2c.sr1.read();

        if sr1.addr().bit_is_set() {
            // ADDR: Our address was matched on the bus
            // clear by reading SR1 and SR2
            i2c.sr1.read();
            i2c.sr2.read();
        }
        if sr1.rx_ne().bit_is_set() {
            // RX_NE: receive buffer not empty
            output_leds(leds, i2c.dr.read().bits() as u8);
        }
        if sr1.stopf().bit_is_set() {
            // STOPF: End of transmission
            // Datasheet: The STOPF bit is cleared by a read of the SR1 register
            // followed by a write to CR1
            i2c.sr1.read();
            i2c.cr1.modify(|_, w| {
                w.ack().set_bit();
                w.stop().clear_bit()
            });
        }
        if sr1.af().bit_is_set() {
            // AF: Failed to acknowledge
            i2c.sr1.modify(|_, w| w.af().clear_bit());
        }
        if sr1.tx_e().bit_is_set() && sr1.btf().bit_is_clear() {
            i2c.dr.write(|w| w.dr().bits(4));
        }
    }
};

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
