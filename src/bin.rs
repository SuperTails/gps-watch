#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use gps_watch as _;
use gps_watch::{
    ubx::{cfg, packets::CfgValSet, ParsedPacket, SendablePacket, UbxParser},
    Abs as _, FmtBuf, Position,
};

use chrono::{prelude::*, DateTime, Utc};
use core::{fmt::Write, num::Wrapping};
use defmt::{debug, error, info, trace};
use embedded_graphics::{
    mono_font::{iso_8859_3::FONT_6X12, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use hal::pac::Interrupt;
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
    pac::{LPUART1, SPI1},
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

type LpUart1 = Serial<LPUART1, (PA2<Alternate<PushPull, 8>>, PA3<Alternate<PushPull, 8>>)>;

const UART_TX_BUF: usize = 16;
const UART_RX_BUF: usize = 256;
const USB_BUF: usize = 16;

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
        position: Position,
        time: DateTime<Utc>,
    }

    // Local resources go here
    #[local]
    struct Local {
        uart: LpUart1,
        uart_rx_send: Sender<'static, u8, UART_RX_BUF>,
        uart_tx_recv: Receiver<'static, u8, UART_TX_BUF>,
    }

    ////////////////////////////////////////////////////////////////////////////
    // Main thread tasks ///////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

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

        let mut uart = Serial::lpuart1(
            cx.device.LPUART1,
            (tx, rx),
            Config::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb1r2,
        );
        uart.listen(serial::Event::Rxne);
        uart.listen(serial::Event::Txe);

        // Create channels for communicating between tasks and the UART interrupt
        let (uart_rx_send, uart_rx_recv) = make_channel!(u8, UART_RX_BUF);
        let (uart_tx_send, uart_tx_recv) = make_channel!(u8, UART_TX_BUF);

        // Create GPS handler task
        gps_task::spawn(uart_tx_send, uart_rx_recv).unwrap();

        // Enable the USB interrupt
        // let usb = unsafe {hal::pac::Peripherals::steal()}.USB;
        // usb.cntr.modify(|_, w| w.wkupm().enabled());

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

        let (usb_tx, usb_rx) = make_channel!(u8, USB_BUF);
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
                position: Position::default(),
                time: DateTime::default(),
            },
            Local {
                uart,
                uart_rx_send,
                uart_tx_recv,
            },
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

    ////////////////////////////////////////////////////////////////////////////
    // Hardware interrupt handlers /////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    // Transfer UART data to/from the buffers
    #[task(binds = LPUART1, priority = 10, local = [uart, uart_rx_send, uart_tx_recv])]
    fn on_uart(cx: on_uart::Context) {
        trace!("on_uart enter");
        // Rxne
        if let Ok(b) = cx.local.uart.read() {
            // If the recv buffer is full, then drop the received value
            let _ = cx.local.uart_rx_send.try_send(b);
        }
        // Txe
        if cx.local.uart.can_write() {
            if let Ok(b) = cx.local.uart_tx_recv.try_recv() {
                cx.local.uart.write(b).unwrap();
            }
        }
    }

    #[task(binds = RTC_WKUP)]
    fn on_rtc(_cx: on_rtc::Context) {
        info!("rtc wakeup!");
    }

    ////////////////////////////////////////////////////////////////////////////
    // Periodic tasks //////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    // Read GPS data
    #[task(priority = 2, shared = [position, time])]
    async fn gps_task(
        mut cx: gps_task::Context,
        mut uart_tx_send: Sender<'static, u8, UART_TX_BUF>,
        mut uart_rx_recv: Receiver<'static, u8, UART_RX_BUF>,
    ) {
        trace!("gps_task enter");

        let packet = CfgValSet {
            ram: true,
            bbr: true,
            flash: false,
            items: [
                // Leave UART settings at default (if we're connected they already work)
                // Use UBX only
                (cfg::CFG_UART1OUTPROT_UBX, true).into(),
                (cfg::CFG_UART1OUTPROT_NMEA, false).into(),
                // Send NAV-PVT packets every second (minimum rate)
                (cfg::CFG_MSGOUT_UBX_NAV_PVT_UART1, 1_u8).into(),
                // PSMOO power save mode (turn off after getting signal)
                (cfg::CFG_PM_OPERATEMODE, cfg::OPERATEMODE_PSMOO).into(),
                // Only stay on for 10 seconds after acquiring signal
                (cfg::CFG_PM_ONTIME, 10_u8).into(),
                // Hold EXTINT low to force off, when GPS not needed
                (cfg::CFG_PM_EXTINTBACKUP, true).into(),
                // Wait for a Time fix before starting ONTIME timeout
                (cfg::CFG_PM_WAITTIMEFIX, true).into(),
                // Make sure the ephemeris stays up to date
                (cfg::CFG_PM_UPDATEEPH, true).into(),
            ],
        };
        for b in packet.to_bytes() {
            uart_tx_send.send(b).await.unwrap();
            rtic::pend(Interrupt::LPUART1);
        }

        let mut parser = UbxParser::new();

        loop {
            match parser.process_byte(uart_rx_recv.recv().await.unwrap()) {
                Some(Ok(pkt)) => match pkt {
                    ParsedPacket::NavPvt(n) => {
                        cx.shared.position.lock(|p| *p = n.position());
                        cx.shared.time.lock(|t| *t = n.datetime());
                    }
                    _ => (),
                },
                Some(Err(e)) => error!("UBX error: {}", e),
                None => (),
            }
        }
    }

    #[task(priority = 1, shared = [display, position, time])]
    async fn display_task(mut cx: display_task::Context, _tx: Sender<'static, u8, USB_BUF>) {
        trace!("display_task enter");
        cx.shared.display.lock(|display| display.clear_flush());

        let clock_font = FontRenderer::new::<fonts::u8g2_font_logisoso42_tr>();

        let mut i = Wrapping(0u8);
        loop {
            debug!("display_task loop");
            // let mut dbg_txt = FmtBuf::<512>::new();
            let mut time_buf = FmtBuf::<8>::new();
            let mut coords_buf = FmtBuf::<64>::new();
            let pos = cx.shared.position.lock(|p| *p);
            let time = cx.shared.time.lock(|t| t.clone());
            // let _ = write!(
            //     dbg_txt,
            //     "Debug Info:\nReceived {} bytes\nLast Packet: {:x?}",
            //     count, last_packet
            // );
            write!(time_buf, "{}:{:02}", time.hour(), time.minute()).unwrap();

            write!(
                coords_buf,
                "{:.4}°{}\n{:.4}°{}",
                pos.lat.abs(),
                if pos.lat >= 0.0 { 'N' } else { 'S' },
                pos.lon.abs(),
                if pos.lon >= 0.0 { 'E' } else { 'W' }
            )
            .unwrap();

            cx.shared.display.lock(|display| {
                display.clear();
                // Debug infos
                // TextBox::new(
                //     dbg_txt.as_str().unwrap(),
                //     Rectangle {
                //         top_left: Point { x: 5, y: 135 },
                //         size: Size {
                //             width: 350,
                //             height: 100,
                //         },
                //     },
                //     MonoTextStyle::new(&FONT_6X12, BinaryColor::On),
                // )
                // .draw(display)
                // .unwrap();

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
                        time_buf.as_str().unwrap(),
                        Point { x: 64, y: 20 },
                        u8g2_fonts::types::VerticalPosition::Top,
                        u8g2_fonts::types::HorizontalAlignment::Center,
                        u8g2_fonts::types::FontColor::Transparent(BinaryColor::On),
                        display,
                    )
                    .unwrap();

                Text::new(
                    coords_buf.as_str().unwrap(),
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

    // Poll USB
    #[task(priority = 1)]
    async fn usb_poll(
        _cx: usb_poll::Context,
        usb: hal::usb::Peripheral,
        mut rx: Receiver<'static, u8, USB_BUF>,
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
}
