#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use gps_watch as _;
use gps_watch::{
    tactile::Button,
    ubx::{cfg, packets::CfgValSet, ParsedPacket, SendablePacket, UbxParser},
    Abs as _, FmtBuf, Position,
};

use chrono::{prelude::*, DateTime, Utc};
use core::{num::Wrapping, sync::atomic::AtomicUsize};
use defmt::{debug, error, info, trace};
use embedded_graphics::{
    mono_font::{iso_8859_3::FONT_6X12, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use gps_watch::{
    rb::{Consumer, Producer, Ringbuf},
    ubx::packets::CfgCfg,
};
use hal::{
    adc::ADC,
    delay::Delay,
    gpio::{Analog, Edge, Input, PullUp, PA1, PA4, PA5, PA6, PA7, PA8, PB0, PB1},
    pac::Interrupt,
};
use rtic_monotonics::{create_systick_token, systick::Systick};
use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
    portable_atomic::Ordering,
};
use stm32_usbd::UsbBus;
use stm32l4xx_hal::{
    self as hal,
    gpio::{Alternate, Output, PushPull, PA2, PA3, PB4, PB5},
    hal::spi::{Mode, Phase, Polarity},
    pac,
    pac::{LPUART1, SPI1},
    prelude::*,
    rcc::{ClockSecuritySystem, CrystalBypass},
    rtc::{Rtc, RtcClockSource, RtcConfig},
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
            PA5<Alternate<PushPull, 5>>,
            PA6<Alternate<PushPull, 5>>,
            PA7<Alternate<PushPull, 5>>,
        ),
    >,
    PA4<Output<PushPull>>,
>;

type LpUart1 = Serial<LPUART1, (PA2<Alternate<PushPull, 8>>, PA3<Alternate<PushPull, 8>>)>;

type Joystick = gps_watch::tactile::Joystick<
    PB5<Input<PullUp>>,
    PB4<Input<PullUp>>,
    PB1<Input<PullUp>>,
    PB0<Input<PullUp>>,
    PA8<Input<PullUp>>,
>;

struct UartStuff {
    uart: LpUart1,
    rx_send: Producer<u8, UART_RX_BUFSIZE>,
    tx_recv: Consumer<u8, UART_TX_BUFSIZE>,
}

const UART_TX_BUFSIZE: usize = 16;
const UART_RX_BUFSIZE: usize = 256;
const USB_BUFSIZE: usize = 16;

static UART_TX_COUNTER: AtomicUsize = AtomicUsize::new(0);
static UART_RX_COUNTER: AtomicUsize = AtomicUsize::new(0);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RedrawEvent {
    Button(Button),
    Time,
    Location,
    Battery,
}

#[rtic::app(
    device = stm32l4xx_hal::pac,
    dispatchers = [EXTI2, EXTI3],
)]
mod app {

    use embedded_io::{Read, Write};
    use gps_watch::ubx::packets::CfgRst;
    use core::fmt::Write as _;

    use super::*;

    // Shared resources go here
    #[shared]
    struct Shared {
        display: SharpMemDisplay,
        position: Position,
        time: DateTime<Utc>,
        joystick: Joystick,
        battery: u16,
    }

    // Local resources go here
    #[local]
    struct Local {
        uart: UartStuff,
        button_redraw_0: Sender<'static, RedrawEvent, 8>,
        button_redraw_1: Sender<'static, RedrawEvent, 8>,
        button_redraw_4: Sender<'static, RedrawEvent, 8>,
        button_redraw_5_9: Sender<'static, RedrawEvent, 8>,
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

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);

        let mut delay = Delay::new(cx.core.SYST, clocks);
        let pa1 = gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
        let adc = hal::adc::ADC::new(
            cx.device.ADC1,
            cx.device.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        );
        cx.core.SYST = delay.free();

        // Create SysTick monotonic for task scheduling
        Systick::start(cx.core.SYST, clocks.sysclk().raw(), create_systick_token!());

        // Initialize SPI and display
        let mut cs = gpioa
            .pa4
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        cs.set_low();
        let sck = gpioa
            .pa5
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let miso = gpioa
            .pa6
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let mosi = gpioa
            .pa7
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let spi1 = Spi::spi1(
            cx.device.SPI1,
            (sck, miso, mosi),
            Mode {
                phase: Phase::CaptureOnFirstTransition,
                polarity: Polarity::IdleLow,
            },
            false,
            2.MHz(),
            clocks,
            &mut rcc.apb2,
        );

        // Initialize RTC and interrupts
        let _rtc = Rtc::rtc(
            cx.device.RTC,
            &mut rcc.apb1r1,
            &mut rcc.bdcr,
            &mut pwr.cr1,
            RtcConfig::default().clock_config(RtcClockSource::LSE),
        );
        // enable RTCAPBSMEN?
        //rtc.wakeup_timer().start(10000u16);
        //rtc.listen(&mut cx.device.EXTI, Event::WakeupTimer);
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

        // Create channels for communicating between tasks and the UART interrupt
        static UART_RX: Ringbuf<u8, UART_RX_BUFSIZE> = Ringbuf::new(None);
        let (uart_rx_send, uart_rx_recv) = UART_RX.try_split().unwrap();
        static UART_TX: Ringbuf<u8, UART_TX_BUFSIZE> = Ringbuf::new(Some(Interrupt::LPUART1));
        let (uart_tx_send, uart_tx_recv) = UART_TX.try_split().unwrap();

        let uart = UartStuff {
            uart,
            rx_send: uart_rx_send,
            tx_recv: uart_tx_recv,
        };

        let mut pin_left = gpiob
            .pb5
            .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);
        let mut pin_right = gpiob
            .pb4
            .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);
        let mut pin_up = gpiob
            .pb1
            .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);
        let mut pin_down = gpiob
            .pb0
            .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);
        let mut pin_center = gpioa
            .pa8
            .into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);

        pin_left.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
        pin_right.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
        pin_up.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
        pin_down.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
        pin_center.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);

        pin_left.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);
        pin_right.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);
        pin_up.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);
        pin_down.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);
        pin_center.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

        pin_left.enable_interrupt(&mut cx.device.EXTI);
        pin_right.enable_interrupt(&mut cx.device.EXTI);
        pin_up.enable_interrupt(&mut cx.device.EXTI);
        pin_down.enable_interrupt(&mut cx.device.EXTI);
        pin_center.enable_interrupt(&mut cx.device.EXTI);

        let joystick = Joystick {
            pin_left,
            pin_right,
            pin_up,
            pin_down,
            pin_center,
        };

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

        static USB_RX: Ringbuf<u8, USB_BUFSIZE> = Ringbuf::new(None);
        let (usb_rx_send, _usb_rx_recv) = USB_RX.try_split().unwrap();
        static USB_TX: Ringbuf<u8, USB_BUFSIZE> = Ringbuf::new(Some(Interrupt::LPUART1));
        let (_usb_tx_send, usb_tx_recv) = USB_TX.try_split().unwrap();
        // Pass to task for remaining initialization
        let _ = usb_poll::spawn(usb, usb_tx_recv, usb_rx_send);

        let (display_redraw_tx, display_redraw_rx) = make_channel!(RedrawEvent, 8);

        // Spawn tasks
        display_task::spawn(display_redraw_rx).unwrap();
        gps_task::spawn(uart_tx_send, uart_rx_recv, display_redraw_tx.clone())
            .map_err(|_| ())
            .unwrap();
        battery_task::spawn(pa1, adc, display_redraw_tx.clone())
            .map_err(|_| ())
            .unwrap();

        info!("done initializing!");
        trace!("init exit");
        (
            Shared {
                display: SharpMemDisplay::new(spi1, cs),
                position: Position::default(),
                time: DateTime::default(),
                joystick,
                battery: 0,
            },
            Local {
                uart,
                button_redraw_0: display_redraw_tx.clone(),
                button_redraw_1: display_redraw_tx.clone(),
                button_redraw_4: display_redraw_tx.clone(),
                button_redraw_5_9: display_redraw_tx,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        trace!("idle enter");

        loop {
            trace!("idle loop");
            // Only sleep in release mode, since the debugger doesn't interact with sleep very nicely
            #[cfg(debug_assertions)]
            cortex_m::asm::nop();
            #[cfg(not(debug_assertions))]
            cortex_m::asm::wfi();
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Hardware interrupt handlers /////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    // Transfer UART data to/from the buffers
    #[task(binds = LPUART1, priority = 10, local = [uart])]
    fn on_uart(cx: on_uart::Context) {
        trace!("on_uart enter");
        // Rxne
        if cx.local.uart.uart.can_read() {
            // If the recv buffer is full, then drop the received value
            let mut b = 0;
            let _ = cx.local.uart.uart.read_exact(core::slice::from_mut(&mut b)); // TODO: Should we ignore UART errors?
            let _ = cx.local.uart.rx_send.try_write(b);
            UART_RX_COUNTER.fetch_add(1, Ordering::Relaxed);
        }
        // Txe
        if cx.local.uart.uart.can_write() {
            if let Some(b) = cx.local.uart.tx_recv.try_read() {
                cx.local.uart.uart.write_all(&[b]).unwrap();
                UART_TX_COUNTER.fetch_add(1, Ordering::Relaxed);
            }
            if !cx.local.uart.tx_recv.is_empty() {
                cx.local.uart.uart.listen(serial::Event::Txe);
            } else {
                cx.local.uart.uart.unlisten(serial::Event::Txe);
            }
        }
    }

    #[task(binds = RTC_WKUP)]
    fn on_rtc(_cx: on_rtc::Context) {
        info!("rtc wakeup!");
    }

    // Button interrupts: multiple identical handlers due to different EXTI lines
    #[task(binds = EXTI0, shared = [joystick], local = [button_redraw_0])]
    fn on_button_0(mut cx: on_button_0::Context) {
        trace!("on_button_0 enter");
        cx.shared.joystick.lock(|joystick| {
            while let Some(button) = joystick.next_edge() {
                let _ = cx
                    .local
                    .button_redraw_0
                    .try_send(RedrawEvent::Button(button));
            }
        });
    }

    #[task(binds = EXTI1, shared = [joystick], local = [button_redraw_1])]
    fn on_button_1(mut cx: on_button_1::Context) {
        trace!("on_button_1 enter");
        cx.shared.joystick.lock(|joystick| {
            while let Some(button) = joystick.next_edge() {
                let _ = cx
                    .local
                    .button_redraw_1
                    .try_send(RedrawEvent::Button(button));
            }
        });
    }

    #[task(binds = EXTI4, shared = [joystick], local = [button_redraw_4])]
    fn on_button_4(mut cx: on_button_4::Context) {
        trace!("on_button_4 enter");
        cx.shared.joystick.lock(|joystick| {
            while let Some(button) = joystick.next_edge() {
                let _ = cx
                    .local
                    .button_redraw_4
                    .try_send(RedrawEvent::Button(button));
            }
        });
    }

    #[task(binds = EXTI9_5, shared = [joystick], local = [button_redraw_5_9])]
    fn on_button_5_9(mut cx: on_button_5_9::Context) {
        trace!("on_button_5_9 enter");
        cx.shared.joystick.lock(|joystick| {
            while let Some(button) = joystick.next_edge() {
                let _ = cx
                    .local
                    .button_redraw_5_9
                    .try_send(RedrawEvent::Button(button));
            }
        });
    }

    ////////////////////////////////////////////////////////////////////////////
    // Periodic tasks //////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    // Monitor battery level
    #[task(priority = 1, shared = [battery])]
    async fn battery_task(
        mut cx: battery_task::Context,
        mut pin: PA1<Analog>,
        mut adc: ADC,
        mut redraw: Sender<'static, RedrawEvent, 8>,
    ) {
        // Read the battery level every 10 seconds
        loop {
            if let Ok(level) = adc.read(&mut pin) {
                cx.shared.battery.lock(|b| *b = level);
                let _ = redraw.send(RedrawEvent::Battery).await;
            }
            Systick::delay(10.secs()).await;
        }
    }

    // Read GPS data
    #[task(priority = 2, shared = [position, time])]
    async fn gps_task(
        mut cx: gps_task::Context,
        uart_tx_send: Producer<u8, UART_TX_BUFSIZE>,
        uart_rx_recv: Consumer<u8, UART_RX_BUFSIZE>,
        mut redraw: Sender<'static, RedrawEvent, 8>,
    ) {
        trace!("gps_task enter");

        info!("clearing GPS configuration");
        CfgCfg {
            clear_mask: 0xFFFFFFFF,
            save_mask: 0,
            load_mask: 0xFFFFFFFF,
            dev_bbr: true,
            dev_flash: true,
            dev_eeprom: false,
            dev_spi_flash: false,
        }
        .send(&uart_tx_send)
        .await;

        info!("restarting GPS");
        CfgRst {
            nav_bbr_mask: 0x0000, // Hot start (don't clear any GPS data)
            reset_mode: 1,        // Controlled software reset
        }
        .send(&uart_tx_send)
        .await;

        // Give the GPS a second to reboot before sending the new configuration
        uart_tx_send.flush().await;
        Systick::delay(1.secs()).await;

        info!("sending new GPS configuration");
        CfgValSet {
            ram: true,
            bbr: true,
            flash: false,
            items: [
                // Leave UART settings at default
                // (if we're connected they already work, and 9600 is plenty fast enough)
                // Use UBX only
                (cfg::CFG_UART1OUTPROT_UBX, true).into(),
                (cfg::CFG_UART1OUTPROT_NMEA, false).into(),
                // Send NAV-PVT packets every second
                (cfg::CFG_MSGOUT_UBX_NAV_PVT_UART1, 1_u8).into(),
                // Send MON-RF packets every 10 seconds
                (cfg::CFG_MSGOUT_UBX_MON_RF_UART1, 10_u8).into(),
                // PSMOO power save mode (turn off after getting signal)
                // (cfg::CFG_PM_OPERATEMODE, cfg::OPERATEMODE_PSMOO).into(),
                // // Only stay on for 10 seconds after acquiring signal
                // (cfg::CFG_PM_ONTIME, 10_u16).into(),
                // // Hold EXTINT low to force off, when GPS not needed
                // (cfg::CFG_PM_EXTINTBACKUP, true).into(),
                // // Wait for a Time fix before starting ONTIME timeout
                // (cfg::CFG_PM_WAITTIMEFIX, true).into(),
                // // Make sure the ephemeris stays up to date
                // (cfg::CFG_PM_UPDATEEPH, true).into(),
                // // Use the "wrist-worn" dynamic model
                // (cfg::CFG_NAVSPG_DYNMODEL, cfg::DYNMODEL_WRIST).into(),
                // // Measure GPS once every 2 seconds and get a nav solution
                // (cfg::CFG_RATE_MEAS, 2000_u16).into(),
                // (cfg::CFG_RATE_NAV, 1_u16).into(),
            ],
        }
        .send(&uart_tx_send)
        .await;

        let mut parser = UbxParser::new();
        loop {
            match parser.process_byte(uart_rx_recv.async_read().await) {
                Some(Ok(pkt)) => {
                    info!("Got packet: {}", pkt);
                    #[allow(clippy::single_match)]
                    match pkt {
                        ParsedPacket::NavPvt(n) => {
                            let mut changed = false;
                            cx.shared.position.lock(|p| {
                                if *p != n.position() {
                                    changed = true;
                                }
                                *p = n.position();
                            });
                            cx.shared.time.lock(|t| *t = n.datetime());

                            if changed {
                                let _ = redraw.try_send(RedrawEvent::Location);
                            }
                        }
                        _ => (),
                    }
                }
                Some(Err(e)) => error!("UBX error: {}", e),
                None => (),
            }
        }
    }

    #[task(priority = 1, shared = [display, position, time])]
    async fn display_task(
        mut cx: display_task::Context,
        mut redraw_events: Receiver<'static, RedrawEvent, 8>,
    ) {
        cx.shared.display.lock(|display| display.clear_flush());

        let clock_font = FontRenderer::new::<fonts::u8g2_font_logisoso42_tr>();

        /*
        static NODES: [[f32; 2]; 2806] = include!("../formatted_nodes.txt");
        static LENGTHS: [usize; 588] = include!("../formatted_lengths.txt");

        const MIN_X: f32 = 0.27792380555555557;
        const MIN_Y: f32 = 0.3769527127520212;

        const MAX_X: f32 = 0.27796433333333337;
        const MAX_Y: f32 = 0.3769777514380456;

        const SPAN: f32 = 2.5038686024436707e-05;
        */

        let mut i = Wrapping(0u8);
        let mut indicator = false;

        while let Ok(event) = redraw_events.recv().await {
            debug!("display_task loop, got event {}", event);

            let mut dbg_txt = FmtBuf::<64>::new();
            let mut time_buf = FmtBuf::<8>::new();
            let mut coords_buf = FmtBuf::<64>::new();
            let pos = cx.shared.position.lock(|p| *p);
            let time = cx
                .shared
                .time
                .lock(|t| *t)
                .with_timezone(&FixedOffset::west_opt(3600 * 4).unwrap());
            let _ = write!(
                dbg_txt,
                "TX: {}, RX: {}",
                UART_TX_COUNTER.load(Ordering::Relaxed),
                UART_RX_COUNTER.load(Ordering::Relaxed),
            );
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
            indicator = !indicator;

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
                Text::new(
                    dbg_txt.as_str().unwrap(),
                    Point { x: 5, y: 135 },
                    MonoTextStyle::new(&FONT_6X12, BinaryColor::On),
                )
                .draw(display)
                .unwrap();

                /*
                let mut i = 0;
                for length in LENGTHS {
                    for j in i..(i + length - 1) {
                        let [x0, y0] = NODES[j];
                        let [x1, y1] = NODES[j + 1];

                        let [x0, y0] = [(x0 - MIN_X) / SPAN, (y0 - MIN_Y) / SPAN];
                        let [x1, y1] = [(x1 - MIN_X) / SPAN, (y1 - MIN_Y) / SPAN];

                        embedded_graphics::primitives::Line {
                            start: Point { x: (x0 * 128.0) as i32, y: (y0 * 128.0) as i32 },
                            end:   Point { x: (x1 * 128.0) as i32, y: (y1 * 128.0) as i32 },
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
                    }
                    i += length;
                }
                */

                // Show indicator for when screen is redrawn
                #[cfg(debug_assertions)]
                if indicator {
                    Rectangle {
                        top_left: Point { x: 0, y: 0 },
                        size: Size {
                            width: 2,
                            height: 2,
                        },
                    }
                    .into_styled(
                        PrimitiveStyleBuilder::new()
                            .fill_color(BinaryColor::On)
                            .build(),
                    )
                    .draw(display)
                    .unwrap();
                }

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
            i += 1;
        }
    }

    // Poll USB
    #[task(priority = 1)]
    async fn usb_poll(
        _cx: usb_poll::Context,
        usb: hal::usb::Peripheral,
        tx_recv: Consumer<u8, USB_BUFSIZE>,
        _rx_send: Producer<u8, USB_BUFSIZE>,
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
                if let Some(b) = tx_recv.try_read() {
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
