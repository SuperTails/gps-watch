#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use gps_watch as _;
use stm32l4xx_hal as hal;

use hal::{pac, prelude::*};
use rtic_monotonics::create_systick_token;
use rtic_monotonics::systick::Systick;
use stm32l4xx_hal::gpio::{Alternate, Output, PA11, PB3, PB4, PB5, PinExt, PushPull};
use stm32l4xx_hal::hal::spi::{Mode, Phase, Polarity};
use stm32l4xx_hal::pac::SPI1;
use stm32l4xx_hal::spi::Spi;
use defmt::{trace, info};
use core::num::Wrapping;
use embedded_graphics::primitives::PrimitiveStyle;
use embedded_graphics::prelude::*;
use rtic::Mutex;

type SharpMemDisplay = gps_watch::display::SharpMemDisplay<
    Spi<SPI1, (
        PB3<Alternate<PushPull, 5>>,
        PB4<Alternate<PushPull, 5>>,
        PB5<Alternate<PushPull, 5>>
    )>, PA11<Output<PushPull>>>;

#[rtic::app(
    device = stm32l4xx_hal::pac,
    // peripherals = true,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0],

)]
mod app {
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::primitives::Rectangle;
    use super::*;

    // Shared resources go here
    #[shared]
    struct Shared {
        display: SharpMemDisplay
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        trace!("init enter");

        // Configure clocks
        // unsafe {
        //     // MSI is already on
        //     // Set MSI to 200 kHz
        //     cx.device.RCC.cr.write(|w| w.msirange().bits(0b0001).msirgsel().set_bit());
        //     // Set MSI as system clock source
        //     cx.device.RCC.cfgr.write(|w| w.sw().bits(0b00));
        // }
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);
        let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

        // Create SysTick monotonic for task scheduling
        Systick::start(
            cx.core.SYST,
            4_000_000,
            // 200_000,
            create_systick_token!()
        );

        // Initialize SPI
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);

        let mut cs = gpioa.pa11.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        cs.set_low();

        // <cs as hal::hal::digital::v2::OutputPin>;
        let mut sck = gpiob.pb3.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let mut mosi = gpiob.pb5.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let mut miso = gpiob.pb4.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);


        let SPI1_COPY = unsafe { core::ptr::read(&cx.device.SPI1) };
        let spi1 = hal::spi::Spi::spi1(
            cx.device.SPI1,
            (sck, miso, mosi),
            Mode {
                phase: Phase::CaptureOnFirstTransition,
                polarity: Polarity::IdleLow
            },
            1.MHz(),
            clocks,
            &mut rcc.apb2
        );
        // SPI1_COPY.cr1.write(|w| w.lsbfirst().lsbfirst());
        // let spi1_cr1 = 0x4001_3000 as *mut u32;
        // unsafe { core::ptr::write_volatile(spi1_cr1, core::ptr::read_volatile(spi1_cr1) | (1u32 << 7)); }

        let mut display: SharpMemDisplay = SharpMemDisplay::new(spi1, cs);

        display_task::spawn().unwrap();

        trace!("init exit");
        (
            Shared {
                display
            },
            Local {
                // Initialization of local resources go here
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        trace!("idle enter");

        // let device_stolen = unsafe { pac::Peripherals::steal() };

        loop {
            core::hint::spin_loop();
            // Sleep
            // rtic::export::wfi();
            // defmt::info!("hewo 1");
            // unsafe {
            //     device_stolen.RCC.cr.write(|w| w.msirange().bits(0b0001).msirgsel().set_bit());
            // }
            // defmt::info!("hewo 2");
            // // Return to our previously-selected MSI frequency
            // device_stolen.RCC.cr.write(|w| w.msirgsel().set_bit());
            // defmt::info!("hewo 3");
            // unsafe {
            //     device_stolen.RCC.cfgr.write(|w| w.sw().bits(0b00));
            // }
            // defmt::info!("hewo 4");
        }
    }

    // TODO: Add tasks
    #[task(
        priority = 1,
        shared = [display]
    )]
    async fn display_task(mut cx: display_task::Context) {
        trace!("display_task enter");
        cx.shared.display.lock(|display| display.clear());

        let mut i = Wrapping(0u8);
        loop {
            Systick::delay(500.millis()).await;
            let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 3);
            let rect_styled = Rectangle { top_left: Point {x: i.0 as i32, y: i.0 as i32}, size: Size { width: 20, height: 20 } }
                .into_styled(stroke);
            cx.shared.display.lock(|display| {
                rect_styled.draw(display).unwrap();
                display.flush();
            });
            info!("hello world");
            i += 1;
        }
    }
}
