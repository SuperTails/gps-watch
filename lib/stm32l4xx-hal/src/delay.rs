//! Delays

use cortex_m::asm;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use crate::hal::delay::DelayNs;
use crate::rcc::Clocks;
use crate::time::Hertz;

/// System timer (SysTick) as a delay provider
pub struct Delay {
    clocks: Clocks,
    syst: SYST,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new(mut syst: SYST, clocks: Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);

        Delay { syst, clocks }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

/// System timer (SysTick) as a delay provider.
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // TODO:
        let us = ns / 1000;

        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        const MAX_RVR: u32 = 0x00FF_FFFF;

        let mut total_rvr = us * (self.clocks.hclk().raw() / 1_000_000);

        while total_rvr != 0 {
            let current_rvr = if total_rvr <= MAX_RVR {
                total_rvr
            } else {
                MAX_RVR
            };

            self.syst.set_reload(current_rvr);
            self.syst.clear_current();
            self.syst.enable_counter();

            // Update the tracking variable while we are waiting...
            total_rvr -= current_rvr;

            while !self.syst.has_wrapped() {}

            self.syst.disable_counter();
        }
    }
}

/// Cortex-M `asm::delay` as provider
#[derive(Clone, Copy)]
pub struct DelayCM {
    sysclk: Hertz,
}

impl DelayCM {
    /// Create a new delay
    pub fn new(clocks: Clocks) -> Self {
        DelayCM {
            sysclk: clocks.sysclk(),
        }
    }

    /// Create a new delay that is unchecked. The user needs to know the `sysclk`.
    ///
    /// # Safety
    /// Sysclk must be the same as the actual clock frequency of the chip
    pub unsafe fn new_unchecked(sysclk: Hertz) -> Self {
        DelayCM { sysclk }
    }
}

impl DelayNs for DelayCM {
    fn delay_ns(&mut self, ns: u32) {
        // TODO:
        let us = ns / 1000;

        // Max delay is 53_687_091 us at 80 MHz
        let ticks = self.sysclk.raw() / 1_000_000;

        asm::delay(ticks * us);
    }
}