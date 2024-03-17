#![no_main]
#![no_std]
#![feature(impl_trait_in_assoc_type)]

use defmt_brtt as _; // import + register global logger
use panic_probe as _; // import + register panic handler

use core::{
    fmt::{self, Write},
    sync::atomic::{AtomicUsize, Ordering},
};
use tinyvec::ArrayVec;

pub mod display;
pub mod rb;
pub mod ubx;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

// Incrementing timestamp for Defmt printouts
static TIMESTAMP: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = TIMESTAMP.load(Ordering::Relaxed);
    TIMESTAMP.store(n + 1, Ordering::Relaxed);
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

// This isn't in core for some reason, so do this to avoid pulling in libm as a dependency
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

// Async shims for StaticProd and StaticCons

// pub trait AsyncProd<T> {
//     fn push_iter_async(&mut self, iter: impl Iterator<Item = T>) -> impl Future<Output = usize>;
// }

// impl<'a, T, const N: usize> AsyncProd<T> for StaticProd<'a, T, N> {
//     fn push_iter_async(&mut self, iter: impl Iterator<Item = T>) -> impl Future<Output = usize> {
//         let mut iter = iter.peekable();
//         let mut count = 0;
//         poll_fn(move |_| {
//             defmt::info!("pended");
//             // Push as many bytes as we can
//             count += self.push_iter(&mut iter);
//             // Pend the uart interrupt to make sure stuff gets sent
//             NVIC::pend(Interrupt::LPUART1);
//             // FIXME: need to wake!
//             match iter.peek() {
//                 Some(_) => Poll::Pending,
//                 None => Poll::Ready(count),
//             }
//         })
//     }
// }

// pub trait AsyncCons<T> {
//     fn pop_async(&mut self) -> impl Future<Output = T>;
// }

// impl<'a, T, const N: usize> AsyncCons<T> for StaticCons<'a, T, N> {
//     fn pop_async(&mut self) -> impl Future<Output = T> {
//         poll_fn(move |_| {
//             match self.try_pop() {
//                 Some(b) => Poll::Ready(b),
//                 None => Poll::Pending
//             }
//         })
//     }
// }
