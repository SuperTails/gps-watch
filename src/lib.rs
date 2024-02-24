#![no_main]
#![no_std]

use core::{fmt::{self, Write}, sync::atomic::{AtomicUsize, Ordering}};
use defmt_brtt as _; // global logger

use panic_probe as _;

// TODO(6) Import your HAL
use stm32l4xx_hal as _;
use tinyvec::ArrayVec; // memory layout

pub mod display;
pub mod gps;
pub mod nmea;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub struct FmtBuf(ArrayVec<[u8; 256]>);

impl Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            self.0.try_push(b);
        }
        Ok(())
    }
}

impl FmtBuf {
    pub fn as_str(&self) -> Option<&str> {
        core::str::from_utf8(self.0.as_slice()).ok()
    }
}