#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use gps_watch as _;
use gps_watch::{gps::Gps, Abs as _, FmtBuf};

use core::{fmt::Write, num::Wrapping};
use defmt::{debug, error, info, trace};
use embedded_graphics::{
    mono_font::{iso_8859_3::FONT_6X12, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use embedded_text::TextBox;
use rtic_monotonics::{create_systick_token, systick::Systick};
use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
};
use stm32_usbd::UsbBus;
use stm32l4xx_hal::{
    self as hal,
    gpio::{Alternate, Output, PushPull, PA1, PA2, PA3, PB3, PB4, PB5},
    hal::spi::{Mode, Phase, Polarity},
    pac,
    pac::{SPI1, LPUART1},
    prelude::*,
    rcc::{ClockSecuritySystem, CrystalBypass},
    rtc::{Event, Rtc, RtcClockSource, RtcConfig},
    serial,
    serial::{Config, Serial},
    spi::Spi,
};
use tinyvec::ArrayVec;
use u8g2_fonts::{fonts, FontRenderer};
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

// Rename type to squash generics
type SharpMemDisplay = gps_watch::display::SharpMemDisplay<
    Spi<
        SPI1,
        (
            PB3<Alternate<PushPull, 5>>,
            PB4<Alternate<PushPull, 5>>,
            PB5<Alternate<PushPull, 5>>,
        ),
    >,
    PA1<Output<PushPull>>,
>;

type GpsUart = Serial<LPUART1, (PA2<Alternate<PushPull, 8>>, PA3<Alternate<PushPull, 8>>)>;

#[rtic::app(
    device = stm32l4xx_hal::pac,
    dispatchers = [EXTI0, EXTI1],
)]
mod app {

    use super::*;

    // Shared resources go here
    #[shared]
    struct Shared {
        display: SharpMemDisplay,
        gps: Gps<GpsUart>,
    }

    // Local resources go here
    #[local]
    struct Local {}

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
        let clocks = rcc
            .cfgr
            .lse(CrystalBypass::Disable, ClockSecuritySystem::Disable)
            .freeze(&mut flash.acr, &mut pwr);

        // Create SysTick monotonic for task scheduling
        Systick::start(cx.core.SYST, clocks.sysclk().raw(), create_systick_token!());

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);

        // Initialize SPI and display
        let mut cs = gpioa
            .pa1
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        cs.set_low();
        let sck = gpiob
            .pb3
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let mosi = gpiob
            .pb5
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let miso = gpiob
            .pb4
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

        let spi1 = Spi::spi1(
            cx.device.SPI1,
            (sck, miso, mosi),
            Mode {
                phase: Phase::CaptureOnFirstTransition,
                polarity: Polarity::IdleLow,
            },
            true,
            2.MHz(),
            clocks,
            &mut rcc.apb2,
        );

        // let display = SharpMemDisplay::new(spi1, cs);

        // Initialize RTC and interrupts
        let mut rtc = Rtc::rtc(
            cx.device.RTC,
            &mut rcc.apb1r1,
            &mut rcc.bdcr,
            &mut pwr.cr1,
            RtcConfig::default().clock_config(RtcClockSource::LSE),
        );
        rtc.wakeup_timer().start(10000u16);
        rtc.listen(&mut cx.device.EXTI, Event::WakeupTimer);
        // rtic::pend(Interrupt::RTC_WKUP);

        // Initialize UART for GPS
        let tx = gpioa
            .pa2
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let rx = gpioa
            .pa3
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

        let mut gps_uart = Serial::lpuart1(
            cx.device.LPUART1,
            (tx, rx),
            Config::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb1r2,
        );
        gps_uart.listen(serial::Event::Rxne);

        // Enable the USB interrupt
        // let usb = unsafe {hal::pac::Peripherals::steal()}.USB;
        // usb.cntr.write(|w| w.wkupm().enabled());

        // Initialize USB Serial
        let dm = gpioa
            .pa11
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        let dp = gpioa
            .pa12
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

        // Turn on USB power
        unsafe {
            pac::Peripherals::steal()
                .PWR
                .cr2
                .modify(|_, w| w.usv().set_bit())
        };

        // Create USB peripheral object
        let usb = hal::usb::Peripheral {
            usb: cx.device.USB,
            pin_dm: dm,
            pin_dp: dp,
        };

        let (usb_tx, usb_rx) = make_channel!(u8, 16);
        // Pass to task for remaining initialization
        let _ = usb_poll::spawn(usb, usb_rx);

        // Spawn tasks
        display_task::spawn(usb_tx.clone()).unwrap();
        // gps_status::spawn().unwrap();

        info!("done initializing!");
        trace!("init exit");
        (
            Shared {
                display: SharpMemDisplay::new(spi1, cs),
                gps: Gps::new(gps_uart),
            },
            Local {},
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        trace!("idle enter");

        loop {
            // Only sleep in release mode, since the debugger doesn't interact with sleep very nicely
            #[cfg(debug_assertions)]
            core::hint::spin_loop();
            #[cfg(not(debug_assertions))]
            rtic::export::wfi();
        }
    }

    #[task(priority = 1)]
    async fn usb_poll(
        _cx: usb_poll::Context,
        usb: hal::usb::Peripheral,
        mut rx: Receiver<'static, u8, 16>,
    ) {
        trace!("usb_poll enter");

        let usb_bus = UsbBus::new(usb);

        let mut serial = SerialPort::new(&usb_bus);

        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("ECE500")
            .product("Landhopper")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        let mut tx_buf = ArrayVec::<[u8; 16]>::new();

        loop {
            Systick::delay(10.millis()).await;

            debug!("usb_poll loop");

            while tx_buf.len() < tx_buf.capacity() {
                if let Ok(b) = rx.try_recv() {
                    tx_buf.push(b);
                } else {
                    break;
                }
            }
            trace!("usb state: {}", usb_dev.state());

            if !usb_dev.poll(&mut [&mut serial]) {
                continue;
            }

            match serial.write(&tx_buf) {
                Ok(count) => {
                    trace!("sent {} bytes to usb", count);
                    tx_buf.drain(0..count).for_each(|_| ());
                }
                Err(_) => error!("usb error"),
            }
        }
    }

    #[task(binds = LPUART1, priority = 10, shared = [gps])]
    fn on_uart(mut cx: on_uart::Context) {
        cx.shared.gps.lock(|gps| {
            gps.handle();
        });
    }

    // #[task(priority = 1, shared = [recv_buf])]
    // async fn gps_status(mut cx: gps_status::Context) {
    //     loop {
    //         Systick::delay(1000.millis()).await;
    //         cx.shared.recv_buf.lock(|x| {
    //             info!("received: {:x}", x.as_slice());
    //             x.clear();
    //         });
    //     }
    // }

    #[task(binds = RTC_WKUP)]
    fn on_rtc(_cx: on_rtc::Context) {
        info!("rtc wakeup!");
    }

    #[task(
        priority = 1,
        shared = [display, gps]
    )]
    async fn display_task(mut cx: display_task::Context, _tx: Sender<'static, u8, 16>) {
        trace!("display_task enter");
        cx.shared.display.lock(|display| display.clear_flush());

        let clock_font = FontRenderer::new::<fonts::u8g2_font_logisoso42_tr>();

        let mut i = Wrapping(0u8);
        loop {
            debug!("display_task loop");
            let mut dbg_txt = FmtBuf::<512>::new();
            let mut time = FmtBuf::<8>::new();
            let mut coords = FmtBuf::<64>::new();
            let (last_navpvt, last_packet, count) = cx
                .shared
                .gps
                .lock(|gps| (gps.last_navpvt, gps.last_packet, gps.count));
            let _ = write!(
                dbg_txt,
                "Debug Info:\nReceived {} bytes\nLast Packet: {:x?}",
                count, last_packet
            );
            if let Some(n) = last_navpvt {
                write!(time, "{}:{:02}", n.hour - 5, n.min).unwrap();

                let lat = n.lat_degrees();
                let lon = n.lon_degrees();
                write!(
                    coords,
                    "{:.4}°{}\n{:.4}°{}",
                    lat.abs(),
                    if lat >= 0.0 { 'N' } else { 'S' },
                    lon.abs(),
                    if lon >= 0.0 { 'E' } else { 'W' }
                )
                .unwrap();
            } else {
                write!(time, "10:10").unwrap();
                write!(coords, "No Fix").unwrap();
            }

            cx.shared.display.lock(|display| {
                display.clear();
                // Debug infos
                TextBox::new(
                    dbg_txt.as_str().unwrap(),
                    Rectangle {
                        top_left: Point { x: 5, y: 135 },
                        size: Size {
                            width: 350,
                            height: 100,
                        },
                    },
                    MonoTextStyle::new(&FONT_6X12, BinaryColor::On),
                )
                .draw(display)
                .unwrap();

                // Watch UI
                Rectangle {
                    top_left: Point { x: 0, y: 0 },
                    size: Size {
                        width: 128,
                        height: 128,
                    },
                }
                .into_styled(
                    PrimitiveStyleBuilder::new()
                        .stroke_alignment(embedded_graphics::primitives::StrokeAlignment::Inside)
                        .stroke_color(BinaryColor::On)
                        .stroke_width(4)
                        .build(),
                )
                .draw(display)
                .unwrap();

                clock_font
                    .render_aligned(
                        time.as_str().unwrap(),
                        Point { x: 64, y: 20 },
                        u8g2_fonts::types::VerticalPosition::Top,
                        u8g2_fonts::types::HorizontalAlignment::Center,
                        u8g2_fonts::types::FontColor::Transparent(BinaryColor::On),
                        display,
                    )
                    .unwrap();

                Text::new(
                    coords.as_str().unwrap(),
                    Point { x: 20, y: 80 },
                    MonoTextStyle::new(&FONT_6X12, BinaryColor::On),
                )
                .draw(display)
                .unwrap();

                display.flush();
            });

            // let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 3);
            // let rect_styled = Rectangle { top_left: Point {x: i.0 as i32, y: i.0 as i32}, size: Size { width: 20, height: 20 } }
            //     .into_styled(stroke);
            // cx.shared.display.lock(|display| {
            //     rect_styled.draw(display).unwrap();
            //     display.flush();
            // });
            Systick::delay(200.millis()).await;
            i += 1;
        }
    }
}
