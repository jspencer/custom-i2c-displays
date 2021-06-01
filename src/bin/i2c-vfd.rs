#![no_main]
#![no_std]

use core::panic::PanicInfo;

use rtt_target::{rprintln, rtt_init_print};
use stm32f1::stm32f103::Interrupt;
use stm32f1xx_hal::{
    gpio::{Output, PushPull, Pxx, State},
    pac::{GPIOA, GPIOB, I2C1, TIM1},
    prelude::*,
    rcc::{Enable, Reset},
    timer::{CountDownTimer, Event, Timer},
};
use switch_hal::{ActiveLow, IntoSwitch, OutputSwitch, Switch};

const OUR_I2C_ADDRESS: u8 = 0x35;
const MESSAGE_LENGTH: usize = 4;

pub struct Display {
    digits: [Switch<Pxx<Output<PushPull>>, ActiveLow>; 4],
    segments: [Switch<Pxx<Output<PushPull>>, ActiveLow>; 7],
}

#[rtic::app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        i2c: I2C1,
        display: Display,
        refresh_timer: CountDownTimer<TIM1>,
        #[init([0,0,0,0])]
        display_buffer: [u8; MESSAGE_LENGTH],
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        rtt_init_print!();
        rprintln!("Running setup");
        let device: stm32f1xx_hal::stm32::Peripherals = c.device;
        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .hclk(24.mhz())
            .sysclk(24.mhz())
            .pclk1(12.mhz())
            .pclk2(12.mhz())
            .freeze(&mut flash.acr);

        let gpioa: GPIOA = device.GPIOA;
        let mut pins_a = gpioa.split(&mut rcc.apb2);
        let gpiob: GPIOB = device.GPIOB;
        let mut pins_b = gpiob.split(&mut rcc.apb2);

        let minute_tens = pins_b
            .pb14
            .into_push_pull_output_with_state(&mut pins_b.crh, State::High)
            .downgrade()
            .into_active_low_switch();
        let hour_tens = pins_a
            .pa3
            .into_push_pull_output_with_state(&mut pins_a.crl, State::High)
            .downgrade()
            .into_active_low_switch();
        let hour_ones = pins_a
            .pa2
            .into_push_pull_output_with_state(&mut pins_a.crl, State::High)
            .downgrade()
            .into_active_low_switch();
        let minute_ones = pins_b
            .pb10
            .into_push_pull_output_with_state(&mut pins_b.crh, State::High)
            .downgrade()
            .into_active_low_switch();

        let segment_a = pins_b
            .pb13
            .into_push_pull_output_with_state(&mut pins_b.crh, State::High)
            .downgrade()
            .into_active_low_switch();
        let segment_b = pins_b
            .pb15
            .into_push_pull_output_with_state(&mut pins_b.crh, State::High)
            .downgrade()
            .into_active_low_switch();
        let segment_c = pins_a
            .pa9
            .into_push_pull_output_with_state(&mut pins_a.crh, State::High)
            .downgrade()
            .into_active_low_switch();
        let segment_d = pins_a
            .pa11
            .into_push_pull_output_with_state(&mut pins_a.crh, State::High)
            .downgrade()
            .into_active_low_switch();
        let segment_e = pins_a
            .pa5
            .into_push_pull_output_with_state(&mut pins_a.crl, State::High)
            .downgrade()
            .into_active_low_switch();
        let segment_f = pins_b
            .pb8
            .into_push_pull_output_with_state(&mut pins_b.crh, State::High)
            .downgrade()
            .into_active_low_switch();
        let segment_g = pins_b
            .pb9
            .into_push_pull_output_with_state(&mut pins_b.crh, State::High)
            .downgrade()
            .into_active_low_switch();

        let mut _colon_grid = pins_a
            .pa8
            .into_push_pull_output_with_state(&mut pins_a.crh, State::High)
            .into_active_low_switch();
        let mut _bottom_dot_segment = pins_b
            .pb12
            .into_push_pull_output_with_state(&mut pins_b.crh, State::High)
            .into_active_low_switch();
        let mut _top_dot_segment = pins_a
            .pa10
            .into_push_pull_output_with_state(&mut pins_a.crh, State::High)
            .into_active_low_switch();

        let digits = [hour_tens, hour_ones, minute_tens, minute_ones];
        let segments = [
            segment_a, segment_b, segment_c, segment_d, segment_e, segment_f, segment_g,
        ];

        let mut refresh_timer =
            Timer::tim1(device.TIM1, &clocks, &mut rcc.apb2).start_count_down(280.hz());
        refresh_timer.listen(Event::Update);

        let _scl = pins_b.pb6.into_alternate_open_drain(&mut pins_b.crl);
        let _sda = pins_b.pb7.into_alternate_open_drain(&mut pins_b.crl);

        rtic::pend(Interrupt::I2C1_EV);

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

        init::LateResources {
            i2c,
            refresh_timer,
            display: Display { digits, segments },
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = I2C1_EV, resources = [i2c, display_buffer], priority = 2)]
    fn i2c1_ev(c: i2c1_ev::Context) {
        static mut READ_COUNT: usize = 0;
        static mut PAYLOAD: [u8; MESSAGE_LENGTH] = [0; MESSAGE_LENGTH];
        let i2c = c.resources.i2c;
        let display_buffer = c.resources.display_buffer;

        let sr1 = i2c.sr1.read();
        if sr1.addr().bit_is_set() {
            // ADDR: Our address was matched on the bus
            *READ_COUNT = 0;
            *PAYLOAD = [0; MESSAGE_LENGTH];
            i2c.sr1.read();
            i2c.sr2.read();
        }
        if sr1.rx_ne().bit_is_set() {
            // RX_NE: receive buffer not empty
            // We will ignore bytes if we are sent too many, but we need to read the register regardless to clear the condition
            let rx = i2c.dr.read();
            if *READ_COUNT < MESSAGE_LENGTH {
                PAYLOAD[*READ_COUNT] = rx.bits() as u8;
            }
            *READ_COUNT += 1;
        }
        if sr1.stopf().bit_is_set() {
            // STOPF: End of transmission
            if *READ_COUNT > 0 {
                *display_buffer = *PAYLOAD;
            }

            // The STOPF bit is cleared by a read of the SR1 register followed by a write to CR1
            i2c.sr1.read();
            i2c.cr1.modify(|_, w| {
                w.ack().set_bit();
                w.stop().clear_bit()
            });
        }

        if sr1.af().bit_is_set() {
            rprintln!("Failed to acknowledge");
            i2c.sr1.modify(|_, w| w.af().clear_bit());
        }
        if sr1.tx_e().bit_is_set() && sr1.btf().bit_is_clear() {
            i2c.dr.write(|w| w.dr().bits(4));
        }

        // Plain errors
        if sr1.berr().bit_is_set() {
            rprintln!("I2C Bus Error");
            i2c.sr1.write(|w| w.berr().clear_bit());
        }
        if sr1.arlo().bit_is_set() {
            rprintln!("Arbitration Loss");
            i2c.sr1.write(|w| w.arlo().clear_bit());
        }
        if sr1.ovr().bit_is_set() {
            rprintln!("OVER THE LINE");
            i2c.sr1.write(|w| w.ovr().clear_bit());
        }
        if sr1.pecerr().bit_is_set() {
            rprintln!("I2C PEC Error");
            i2c.sr1.write(|w| w.pecerr().clear_bit());
        }
        if sr1.timeout().bit_is_set() {
            rprintln!("I2C Timeout");
            i2c.sr1.write(|w| w.timeout().clear_bit());
        }
        if sr1.smbalert().bit_is_set() {
            rprintln!("SMBus Alert");
            i2c.sr1.write(|w| w.smbalert().clear_bit());
        }
    }

    #[task(binds = TIM1_UP, resources = [refresh_timer, display, display_buffer], priority = 1)]
    fn tim1_up(c: tim1_up::Context) {
        static mut DISPLAY_DIGIT: usize = 0;
        let display = c.resources.display;
        let mut display_buffer = c.resources.display_buffer;

        let mut frame: u8 = 0;
        display_buffer.lock(|buf| {
            frame = buf[*DISPLAY_DIGIT];
        });
        select_digit(display, *DISPLAY_DIGIT);
        output_segments(&mut display.segments, frame);

        *DISPLAY_DIGIT += 1;
        if *DISPLAY_DIGIT >= MESSAGE_LENGTH {
            *DISPLAY_DIGIT = 0;
        }
        c.resources.refresh_timer.clear_update_interrupt_flag();
    }
};

fn select_digit(display: &mut Display, digit_position: usize) {
    let digit_select = 1 << digit_position;
    for (i, digit) in display.digits.iter_mut().enumerate() {
        if (digit_select >> i) == 1 {
            digit.on().unwrap();
        } else {
            digit.off().unwrap();
        }
    }
}

fn output_segments(segments: &mut [Switch<Pxx<Output<PushPull>>, ActiveLow>; 7], value: u8) {
    for (i, segment) in segments.iter_mut().enumerate() {
        if (value >> i) & 1 > 0 {
            segment.on().unwrap();
        } else {
            segment.off().unwrap();
        }
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {}
}
