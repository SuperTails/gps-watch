use core::{cell::UnsafeCell, future::{poll_fn, Future}, mem::MaybeUninit, ops::DerefMut, sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed}, task::{Poll::{Pending, Ready}, Waker}};
use core::mem::take;

use spin::Mutex;
use stm32l4xx_hal::pac::{Interrupt, NVIC};

// Push at HEAD, pop at TAIL
#[derive(Debug)]
pub struct Ringbuf<T, const N: usize> {
    is_split: AtomicBool,
    interrupt: Option<Interrupt>,
    head: AtomicUsize,
    tail: AtomicUsize,
    consumer_waker: Mutex<Option<Waker>>,
    producer_waker: Mutex<Option<Waker>>,
    buf: [UnsafeCell<MaybeUninit<T>>; N],
}

unsafe impl<T, const N: usize> Send for Ringbuf<T, N> {}
unsafe impl<T, const N: usize> Sync for Ringbuf<T, N> {}

impl<T, const N: usize> Default for Ringbuf<T, N> {
    fn default() -> Self {
        Self::new(None)
    }
}

impl<T, const N: usize> Ringbuf<T, N> {
    pub const fn new(interrupt: Option<Interrupt>) -> Self {
        Self {
            is_split: AtomicBool::new(false),
            interrupt,
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            consumer_waker: Mutex::new(None),
            producer_waker: Mutex::new(None),
            // SAFETY: This array only contains MaybeUninits, which are sound to
            // have hold an uninit value
            buf: unsafe { MaybeUninit::uninit().assume_init() }
        }
    }

    /// SAFETY: Must only be called once
    pub unsafe fn split(&'static self) -> (Producer<T, N>, Consumer<T, N>) {
        self.is_split.store(true, Relaxed);
        (Producer(self), Consumer(self))
    }

    pub fn try_split(&'static self) -> Option<(Producer<T, N>, Consumer<T, N>)> {
        if self.is_split.fetch_or(true, Relaxed) {
            None
        } else {
            // SAFETY: We have just checked to ensure that this Ringbuf has not
            // been split.
            Some(unsafe { self.split() })
        }
    }

    fn is_empty(&'static self) -> bool {
        self.head.load(Relaxed) == self.tail.load(Relaxed)
    }

    fn is_full(&'static self) -> bool {
        let head = self.head.load(Relaxed);
        let tail = self.tail.load(Relaxed);
        head - tail == N
    }
}

pub struct Consumer<T: 'static, const N: usize>(&'static Ringbuf<T, N>);

impl<T: 'static, const N: usize> Consumer<T, N> {
    pub fn try_read(&self) -> Option<T> {
        if self.is_empty() {
            None
        } else {
            // SAFETY: The buffer is not empty, and could not have become empty since
            // we checked it because only one Consumer may exist.
            let val = unsafe {
                self.0.buf[self.0.tail.fetch_add(1, Relaxed) % N].get().read().assume_init()
            };
            // Wake the producer task
            if let Some(waker) = take(self.0.producer_waker.lock().deref_mut()) {
                waker.wake();
                defmt::debug!("woke up Producer from Consumer");
            }

            Some(val)
        }
    }

    pub fn async_read<'a>(&'a self) -> impl Future<Output = T> + 'a {
        poll_fn(|ctx| match self.try_read() {
            None => {
                *self.0.consumer_waker.lock() = Some(ctx.waker().clone());
                Pending
            },
            Some(val) => {
                Ready(val)
            }
        })
    }

    pub fn is_full(&self) -> bool {
        self.0.is_full()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

pub struct Producer<T: 'static, const N: usize>(&'static Ringbuf<T, N>);

impl<T: 'static, const N: usize> Producer<T, N> {
    pub fn try_write(&self, val: T) -> Result<(), T> {
        defmt::trace!("try_write");
        if self.is_full() {
            Err(val)
        } else {
            // SAFETY: The buffer is not full, and could not have become full since
            // we checked it because only one Producer may exist.
            unsafe {
                (*self.0.buf[self.0.head.fetch_add(1, Relaxed) % N].get()).write(val);
            }
            // Wake the consumer task
            if let Some(waker) = take(self.0.consumer_waker.lock().deref_mut()) {
                waker.wake();
                defmt::debug!("woke up Consumer from Producer");
            }
            if let Some(interrupt) = self.0.interrupt {
                NVIC::pend(interrupt);
            }
            Ok(())
        }
    }

    pub fn try_write_iter(&self, iter: &mut impl Iterator<Item = T>, leftover: Option<T>) -> (Result<(), T>, usize) {
        if let Some(val) = leftover {
            if let Err(val) = self.try_write(val) {
                return (Err(val), 0)
            }
        }
        let mut count = 0_usize; // can't enumerate cause we don't own the iterator
        while let Some(val) = iter.next() {
            count += 1;
            if let Err(val) = self.try_write(val) {
                return (Err(val), count)
            }
        }
        (Ok(()), count)
    }

    pub fn async_write<'a>(&'a self, val: T) -> impl Future<Output = ()> + 'a {
        let mut val_buf = Some(val);
        poll_fn(move |ctx| match self.try_write(take(&mut val_buf).unwrap()) {
            Err(val) => {
                *self.0.producer_waker.lock() = Some(ctx.waker().clone());
                val_buf = Some(val);
                Pending
            }
            Ok(()) => Ready(())
        })
    }

    pub fn async_write_iter<'a>(&'a self, mut iter: impl Iterator<Item = T> + 'a) -> impl Future<Output = usize> + 'a {
        let mut leftover = Some(None);
        let mut count = 0;
        poll_fn(move |ctx| match self.try_write_iter(&mut iter, leftover.take().flatten()) {
            (Err(val), newcount) => {
                *self.0.producer_waker.lock() = Some(ctx.waker().clone());
                leftover = Some(Some(val));
                count += newcount;
                Pending
            }
            (Ok(()), newcount) => Ready(count + newcount)
        })
    }

    pub fn is_full(&self) -> bool {
        self.0.is_full()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}