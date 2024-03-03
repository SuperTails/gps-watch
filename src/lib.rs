#![no_main]
#![no_std]

use core::{
    fmt::{self, Write},
    sync::atomic::{AtomicUsize, Ordering},
};
use defmt_brtt as _; // global logger

use panic_probe as _;

// TODO(6) Import your HAL
use stm32l4xx_hal as _;
use tinyvec::ArrayVec; // memory layout

pub mod display;
pub mod gps;
pub mod ubx;

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

pub struct FmtBuf<const N: usize = 256>(pub ArrayVec<[u8; N]>);

impl<const N: usize> Write for FmtBuf<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            self.0.try_push(b);
        }
        Ok(())
    }
}

impl<const N: usize> FmtBuf<N> {
    pub fn as_str(&self) -> Option<&str> {
        core::str::from_utf8(self.0.as_slice()).ok()
    }

    pub fn new() -> Self {
        Self(Default::default())
    }
}

// This isn't in core for some reason, so do this to avoid pulling in a dependency
pub trait Abs {
    fn abs(self) -> Self;
}

impl Abs for f32 {
    fn abs(self) -> Self {
        f32::from_bits(self.to_bits() & 0x7fff_ffff)
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Position {
    pub lat: f32,
    pub lon: f32,
}
