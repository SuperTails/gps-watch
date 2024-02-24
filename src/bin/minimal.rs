#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use gps_watch as _;

use stm32l4xx_hal::{self as hal, pac, prelude::*};
use rtic_monotonics::create_systick_token;
use rtic_monotonics::systick::Systick;
use stm32l4xx_hal::gpio::{Alternate, Output, PA10, PA11, PA9, PB3, PB4, PB5, PushPull};
use stm32l4xx_hal::hal::spi::{Mode, Phase, Polarity};
use stm32l4xx_hal::pac::{SPI1, USART1};
use stm32l4xx_hal::spi::Spi;
use defmt::{trace, info};
use core::num::Wrapping;
use embedded_graphics::primitives::PrimitiveStyle;
use embedded_graphics::prelude::*;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Text;
use stm32l4xx_hal::serial::Serial;
use core::fmt::Write;
use rtic::Mutex;
use stm32l4xx_hal::pac::Interrupt;
use stm32l4xx_hal::rcc::{ClockSecuritySystem, CrystalBypass};
use stm32l4xx_hal::rtc::{Event, RtcClockSource, RtcConfig};
use stm32l4xx_hal::serial;
use stm32l4xx_hal::serial::Config;
use gps_watch::gps::Gps;

// Rename type to squash generics
type SharpMemDisplay = gps_watch::display::SharpMemDisplay<
    Spi<SPI1, (
        PB3<Alternate<PushPull, 5>>,
        PB4<Alternate<PushPull, 5>>,
        PB5<Alternate<PushPull, 5>>
    )>, PA11<Output<PushPull>>>;

type GpsUart = Serial<USART1, (PA9<Alternate<PushPull, 7>>, PA10<Alternate<PushPull, 7>>)>;

#[rtic::app(
    device = stm32l4xx_hal::pac,
    // peripherals = true,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0],
)]
mod app {
    use super::*;

    // Shared resources go here
    #[shared]
    struct Shared {
        display: SharpMemDisplay,
        gps: Gps<GpsUart>
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        trace!("init enter");

        // #[cfg(debug_assertions)]
        // {
        //     // Enable debugging while the processor is sleeping
        //     cx.device.DBGMCU.cr.write(|w| w
        //         .dbg_sleep().set_bit()
        //         .dbg_standby().set_bit()
        //         .dbg_stop().set_bit()
        //     );
        //     cx.device.RCC.ahb1enr.write(|w| w.dma1en().set_bit());
        // }

        // const _: () = [][core::mem::size_of::<Shared>()];

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
        // let clocks = rcc.cfgr.lse(CrystalBypass::Disable, ClockSecuritySystem::Disable).freeze(&mut flash.acr, &mut pwr);
        let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

        // Create SysTick monotonic for task scheduling
        Systick::start(
            cx.core.SYST,
            clocks.sysclk().raw(),
            create_systick_token!()
        );

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);

        // Initialize SPI and display
        let mut cs = gpioa.pa11.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        cs.set_low();
        let sck = gpiob.pb3.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let mosi = gpiob.pb5.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let miso = gpiob.pb4.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

        let spi1 = hal::spi::Spi::spi1(
            cx.device.SPI1,
            (sck, miso, mosi),
            Mode {
                phase: Phase::CaptureOnFirstTransition,
                polarity: Polarity::IdleLow
            },
            true,
            1.MHz(),
            clocks,
            &mut rcc.apb2
        );

        // let display = SharpMemDisplay::new(spi1, cs);

        // Initialize RTC and interrupts
        // let mut rtc = hal::rtc::Rtc::rtc(
        //     cx.device.RTC,
        //     &mut rcc.apb1r1,
        //     &mut rcc.bdcr,
        //     &mut pwr.cr1,
        //     RtcConfig::default().clock_config(RtcClockSource::LSE)
        // );
        // rtc.wakeup_timer().start(10000u16);
        // rtc.listen(&mut cx.device.EXTI, Event::WakeupTimer);
        // rtic::pend(Interrupt::RTC_WKUP);

        // Initialize UART for GPS
        let tx = gpioa.pa9.into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        let rx = gpioa.pa10.into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

        let mut gps_uart = hal::serial::Serial::usart1(
            cx.device.USART1,
            (tx, rx),
            Config::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb2
        );
        gps_uart.listen(serial::Event::Rxne);

        // Spawn tasks
        display_task::spawn().unwrap();

        info!("done initializing!");
        trace!("init exit");
        (
            Shared {
                display: SharpMemDisplay::new(spi1, cs),
                gps: Gps::new(gps_uart)
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

        loop {
            trace!("sleep");
            // Only sleep in release mode, since the debugger doesn't interact with sleep very nice
            #[cfg(debug_assertions)]
            core::hint::spin_loop();
            #[cfg(not(debug_assertions))]
            rtic::export::wfi();
        }
    }

    #[task(binds = USART1, shared = [gps])]
    fn on_uart(mut cx: on_uart::Context) {
        cx.shared.gps.lock(|gps| {
            gps.handle();
        });

    }

    #[task(binds = RTC_WKUP)]
    fn on_rtc(_cx: on_rtc::Context) {
        info!("rtc wakeup!");
    }

    // TODO: Add tasks
    #[task(
        priority = 1,
        shared = [display, gps]
    )]
    async fn display_task(mut cx: display_task::Context) {
        trace!("display_task enter");
        cx.shared.display.lock(|display| display.clear());

        let mut i = Wrapping(0u8);
        loop {
            let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 3);
            let rect_styled = Rectangle { top_left: Point {x: i.0 as i32, y: i.0 as i32}, size: Size { width: 20, height: 20 } }
                .into_styled(stroke);
            cx.shared.display.lock(|display| {
                rect_styled.draw(display).unwrap();
                display.flush();
            });
            Systick::delay(500.millis()).await;
            i += 1;
        }
    }
}
