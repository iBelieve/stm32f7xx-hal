//! Timers

use crate::hal::timer::{Cancel, CountDown, Periodic};
use crate::pac::{
    TIM1, TIM10, TIM11, TIM12, TIM13, TIM14, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9,
};
use crate::rcc::{Clocks, APB1, APB2};
use crate::time::Hertz;
use cast::{u16, u32};
use nb;
use void::Void;

/// Hardware timers
pub struct Timer<TIM> {
    clock: Hertz,
    tim: TIM,
    timeout: Hertz,
}

/// Interrupt events
#[derive(Debug, PartialEq)]
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

/// Timer errors
#[derive(Debug, PartialEq)]
pub enum Error {
    /// Timer is disabled.
    Disabled,
}

macro_rules! hal {
    ($($TIM:ident: ($tim:ident, $timXen:ident, $timXrst:ident, $apb:ident, $timclk:ident),)+) => {
        $(
            impl Periodic for Timer<$TIM> {}

            impl CountDown for Timer<$TIM> {
                type Time = Hertz;

                #[allow(unused_unsafe)]
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.disable();

                    self.timeout = timeout.into();
                    let frequency = self.timeout.0;
                    let ticks = self.clock.0 / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    self.tim.egr.write(|w| w.ug().set_bit());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());

                    self.enable();
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.tim.sr.modify(|_, w| w.uif().clear_bit());
                        Ok(())
                    }
                }
            }

            impl Cancel for Timer<$TIM> {
                type Error = Error;

                fn cancel(&mut self) -> Result<(), Self::Error> {
                    if !self.tim.cr1.read().cen().is_enabled() {
                        return Err(Error::Disabled);
                    }

                    self.disable();

                    Ok(())
                }
            }

            impl Timer<$TIM> {
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIM, timeout: T, clocks: Clocks, apb: &mut $apb) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let clock = clocks.$timclk();

                    let mut timer = Timer {
                        clock,
                        tim,
                        timeout: Hertz(0),
                    };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Clears interrupt associated with `event`.
                ///
                /// If the interrupt is not cleared, it will immediately retrigger after
                /// the ISR has finished.
                pub fn clear_interrupt(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Clear interrupt flag
                            self.tim.sr.write(|w| w.uif().clear_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Releases the TIM peripheral
                pub fn free(mut self) -> $TIM {
                    self.disable();

                    self.tim
                }

                /// Enables the counter.
                fn enable(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                /// Disables the counter.
                fn disable(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                }
            }
        )+
    }
}

hal! {
    TIM2: (tim2, tim2en, tim2rst, APB1, timclk1),
    TIM3: (tim3, tim3en, tim3rst, APB1, timclk1),
    TIM4: (tim4, tim4en, tim4rst, APB1, timclk1),
    TIM5: (tim5, tim5en, tim5rst, APB1, timclk1),
    TIM6: (tim6, tim6en, tim6rst, APB1, timclk1),
    TIM7: (tim7, tim7en, tim7rst, APB1, timclk1),
    TIM12: (tim12, tim12en, tim12rst, APB1, timclk1),
    TIM13: (tim13, tim13en, tim13rst, APB1, timclk1),
    TIM14: (tim14, tim14en, tim14rst, APB1, timclk1),

    TIM1: (tim1, tim1en, tim1rst, APB2, timclk2),
    TIM8: (tim8, tim8en, tim8rst, APB2, timclk2),
    TIM9: (tim9, tim9en, tim9rst, APB2, timclk2),
    TIM10: (tim10, tim10en, tim10rst, APB2, timclk2),
    TIM11: (tim11, tim11en, tim11rst, APB2, timclk2),
}

#[cfg(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f730",
    feature = "stm32f732",
    feature = "stm32f733",
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
))]
use crate::gpio::gpioa::*;

#[cfg(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f730",
    feature = "stm32f732",
    feature = "stm32f733",
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
))]
use crate::gpio::gpiob::*;

#[cfg(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f730",
    feature = "stm32f732",
    feature = "stm32f733",
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
))]
use crate::gpio::gpioc::*;

#[cfg(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f730",
    feature = "stm32f732",
    feature = "stm32f733",
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
))]
use crate::gpio::gpiod::*;

#[cfg(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f730",
    feature = "stm32f732",
    feature = "stm32f733",
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
))]
use crate::gpio::gpioe::*;

#[cfg(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f730",
    feature = "stm32f732",
    feature = "stm32f733",
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
))]
use crate::gpio::gpiof::*;

#[cfg(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f730",
    feature = "stm32f732",
    feature = "stm32f733",
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
))]
use crate::gpio::gpioh::*;

#[cfg(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f730",
    feature = "stm32f732",
    feature = "stm32f733",
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
))]
use crate::gpio::gpioi::*;

use crate::gpio::{Alternate, AF1, AF2, AF3};

// Output channels marker traits
pub trait PinC1<TIM> {}
pub trait PinC2<TIM> {}
pub trait PinC3<TIM> {}
pub trait PinC4<TIM> {}

macro_rules! channel_impl {
    ( $( $TIM:ident, $PINC:ident, $PINX:ident, $MODE:ident<$AF:ident>; )+ ) => {
        $(
            impl $PINC<$TIM> for $PINX<$MODE<$AF>> {}
        )+
    };
}

channel_impl!(
    TIM1, PinC1, PA8, Alternate<AF1>;
    TIM1, PinC2, PA9, Alternate<AF1>;
    TIM1, PinC3, PA10, Alternate<AF1>;
    TIM1, PinC4, PA11, Alternate<AF1>;

    TIM1, PinC1, PE9, Alternate<AF1>;
    TIM1, PinC2, PE11, Alternate<AF1>;
    TIM1, PinC3, PE13, Alternate<AF1>;
    TIM1, PinC4, PE14, Alternate<AF1>;

    TIM2, PinC1, PA0, Alternate<AF1>;
    TIM2, PinC2, PA1, Alternate<AF1>;
    TIM2, PinC3, PA2, Alternate<AF1>;
    TIM2, PinC4, PA3, Alternate<AF1>;

    TIM2, PinC1, PA5, Alternate<AF1>;
    TIM2, PinC1, PA15, Alternate<AF1>;

    TIM2, PinC2, PB3, Alternate<AF1>;
    TIM2, PinC3, PB10, Alternate<AF1>;
    TIM2, PinC4, PB11, Alternate<AF1>;

    TIM3, PinC1, PA6, Alternate<AF2>;
    TIM3, PinC2, PA7, Alternate<AF2>;
    TIM3, PinC3, PB0, Alternate<AF2>;
    TIM3, PinC4, PB1, Alternate<AF2>;

    TIM3, PinC1, PB4, Alternate<AF2>;
    TIM3, PinC2, PB5, Alternate<AF2>;

    TIM3, PinC1, PC6, Alternate<AF2>;
    TIM3, PinC2, PC7, Alternate<AF2>;
    TIM3, PinC3, PC8, Alternate<AF2>;
    TIM3, PinC4, PC9, Alternate<AF2>;

    TIM4, PinC1, PB6, Alternate<AF2>;
    TIM4, PinC2, PB7, Alternate<AF2>;
    TIM4, PinC3, PB8, Alternate<AF2>;
    TIM4, PinC4, PB9, Alternate<AF2>;

    TIM4, PinC1, PD12, Alternate<AF2>;
    TIM4, PinC2, PD13, Alternate<AF2>;
    TIM4, PinC3, PD14, Alternate<AF2>;
    TIM4, PinC4, PD15, Alternate<AF2>;

    TIM5, PinC1, PA0, Alternate<AF2>;
    TIM5, PinC2, PA1, Alternate<AF2>;
    TIM5, PinC3, PA2, Alternate<AF2>;
    TIM5, PinC4, PA3, Alternate<AF2>;

    TIM5, PinC1, PH10, Alternate<AF2>;
    TIM5, PinC2, PH11, Alternate<AF2>;
    TIM5, PinC3, PH12, Alternate<AF2>;
    TIM5, PinC4, PI0, Alternate<AF2>;

    TIM8, PinC1, PC6, Alternate<AF3>;
    TIM8, PinC2, PC7, Alternate<AF3>;
    TIM8, PinC3, PC8, Alternate<AF3>;
    TIM8, PinC4, PC9, Alternate<AF3>;

    TIM8, PinC1, PI5, Alternate<AF3>;
    TIM8, PinC2, PI6, Alternate<AF3>;
    TIM8, PinC3, PI7, Alternate<AF3>;
    TIM8, PinC4, PI2, Alternate<AF3>;

    TIM9, PinC1, PE5, Alternate<AF3>;
    TIM9, PinC2, PE6, Alternate<AF3>;

    TIM10, PinC1, PB8, Alternate<AF3>;
    TIM10, PinC1, PF6, Alternate<AF3>;

    TIM11, PinC1, PB9, Alternate<AF3>;
    TIM11, PinC1, PF7, Alternate<AF3>;
);
