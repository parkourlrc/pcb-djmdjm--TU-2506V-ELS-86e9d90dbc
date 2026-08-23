//! TIM1-backed servo pulse generator.

use stm32f4xx_hal as hal;

use hal::gpio::{self, Speed};
use hal::pac;
use hal::rcc::{Clocks, Enable, Reset};

const TIMER_TICK_HZ: u32 = 1_000_000;
const MAX_PULSES_PER_BATCH: u32 = 256;

pub struct Pulser {
    tim: pac::TIM1,
    _pin: gpio::PB0<gpio::Alternate<1>>,
}

impl Pulser {
    pub fn new(tim: pac::TIM1, pin: gpio::PB0<gpio::Output>, clocks: &Clocks) -> Self {
        let timer_clock = clocks.timclk2().raw();
        assert_eq!(timer_clock % TIMER_TICK_HZ, 0);
        let prescaler = timer_clock / TIMER_TICK_HZ;
        assert!((1..=u16::MAX as u32 + 1).contains(&prescaler));

        unsafe {
            pac::TIM1::enable_unchecked();
            pac::TIM1::reset_unchecked();
        }

        // A 1 MHz timer tick, ARR=1 and CCR2=1 give a 500 kHz waveform
        // with one microsecond in each state.
        tim.cr1.write(|w| w.arpe().set_bit().opm().set_bit());
        tim.cr2.write(|w| w.ois2n().clear_bit());
        tim.psc.write(|w| w.psc().bits((prescaler - 1) as u16));
        tim.arr.write(|w| w.arr().bits(1));
        tim.ccr2().write(|w| w.ccr().bits(1));

        // PWM mode 1 is inverted by the complementary output: PB0 is low
        // during the first tick and high during the second tick. At an update
        // event the counter returns to zero, so the stopped output is low.
        tim.ccmr1_output()
            .write(|w| w.oc2pe().set_bit().oc2m().pwm_mode1());
        tim.ccer.write(|w| {
            w.cc2e()
                .clear_bit()
                .cc2p()
                .clear_bit()
                .cc2ne()
                .set_bit()
                .cc2np()
                .clear_bit()
        });

        // Load the preloaded registers before connecting the timer to PB0.
        tim.egr.write(|w| w.ug().set_bit());
        tim.sr.write(|w| w.uif().clear_bit());
        let pin = pin.into_alternate::<1>().speed(Speed::Medium);
        tim.bdtr.write(|w| w.moe().set_bit());

        Self { tim, _pin: pin }
    }

    pub fn pulse(&mut self, mut count: u32) {
        while count != 0 {
            let batch = count.min(MAX_PULSES_PER_BATCH);

            self.tim
                .rcr
                .write(|w| unsafe { w.rep().bits((batch - 1) as u8) });
            self.tim.egr.write(|w| w.ug().set_bit());
            self.tim.sr.write(|w| w.uif().clear_bit());
            self.tim.cr1.modify(|_, w| w.cen().set_bit());

            while self.tim.cr1.read().cen().bit_is_set() {
                core::hint::spin_loop();
            }

            count -= batch;
        }
    }

    #[allow(dead_code)]
    pub fn abort(&mut self) {
        self.tim.cr1.modify(|_, w| w.cen().clear_bit());
        self.tim.egr.write(|w| w.ug().set_bit());
        self.tim.sr.write(|w| w.uif().clear_bit());
    }
}
