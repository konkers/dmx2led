//! Uses RTIC with the RTC as time source to blink an LED.
//!
//! The idle task is sleeping the CPU, so in practice this gives similar power
//! figure as the "sleeping_timer_rtc" example.
#![no_std]
#![no_main]

extern crate feather_m0 as bsp;
#[cfg(not(feature = "semihosting"))]
extern crate panic_halt;
#[cfg(feature = "semihosting")]
extern crate panic_semihosting;

mod ws2812;

#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [EVSYS])]
mod app {
    #[cfg(feature = "semihosting")]
    use cortex_m_semihosting::hprintln;
    use hal::target_device::SYST;
    use rtic::Monotonic;

    use bsp::hal;
    use hal::clock::{ClockGenId, ClockSource, GenericClockController};
    use hal::dmac::{
        BufferPair, Busy, CallbackStatus, Ch0, Channel, DmaController, PriorityLevel, Ready,
        Transfer,
    };
    use hal::gpio::v2::{PA10, PA11, PB10, PB11};
    use hal::pac::Peripherals;
    use hal::prelude::*;
    use hal::rtc::{Count32Mode, Rtc};
    use hal::sercom::v2::spi::{self, Master};
    use hal::sercom::v2::uart::{self, BaudMode, Duplex, EightBit, Oversampling};
    use hal::sercom::v2::{Sercom2, Sercom4};
    use hal::typelevel::NoneT;
    use rtic_monotonic::Extensions;

    use crate::ws2812;

    const NUM_LEDS: usize = 5;
    const BUF_LEN: usize = ws2812::Ws2812::buffer_len(NUM_LEDS);
    const DMX_ADDR: u16 = 3;

    // SERCOM4 is the SPI for the feather board.
    type SpiPads = spi::PadsFromIds<Sercom4, NoneT, PB10, PB11>;
    type SpiConfig = spi::Config<SpiPads, Master>;
    type Spi = spi::Spi<SpiConfig>;

    // SERCOM2 is the UART for the feather board
    type Pads = uart::PadsFromIds<Sercom2, PA11, PA10>;
    type UartConfig = uart::Config<Pads, EightBit>;
    type Uart = uart::Uart<UartConfig, Duplex>;

    #[local]
    struct Local {
        opt_spi_chan_buf: Option<(Spi, Channel<Ch0, Ready>, &'static mut [u8])>,
        dmx_index: u16,
    }

    #[shared]
    struct Shared {
        // The LED could be a local resource, since it is only used in one task
        // But we want to showcase shared resources and locking
        red_led: bsp::RedLed,

        uart: Uart,

        #[lock_free]
        sys_tick: SYST,

        #[lock_free]
        opt_transfer: Option<
            Transfer<Channel<Ch0, Busy>, BufferPair<&'static mut [u8], Spi>, fn(CallbackStatus)>,
        >,

        // Need a lockless reader/writer buffer here
        #[lock_free]
        buf: [u8; 3],
    }

    #[monotonic(binds = RTC, default = true)]
    type RtcMonotonic = Rtc<Count32Mode>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        #[cfg(feature = "semihosting")]
        hprintln!("dmx2led");

        let mut peripherals: Peripherals = cx.device;
        let mut clocks = GenericClockController::with_external_32kosc(
            peripherals.GCLK,
            &mut peripherals.PM,
            &mut peripherals.SYSCTRL,
            &mut peripherals.NVMCTRL,
        );

        let gclk0 = clocks.gclk0();
        let rtc_clock_src = clocks
            .configure_gclk_divider_and_source(ClockGenId::GCLK2, 1, ClockSource::XOSC32K, false)
            .unwrap();
        clocks.configure_standby(ClockGenId::GCLK2, true);
        let rtc_clock = clocks.rtc(&rtc_clock_src).unwrap();
        let rtc = Rtc::count32_mode(peripherals.RTC, rtc_clock.freq(), &mut peripherals.PM);

        let mut dmac = DmaController::init(peripherals.DMAC, &mut peripherals.PM);
        let channels = dmac.split();
        let chan0 = channels.0.init(PriorityLevel::LVL0);

        let pins = bsp::Pins::new(peripherals.PORT);
        let mut red_led: bsp::RedLed = pins.d13.into();
        red_led.set_low().unwrap();

        let clock = clocks.sercom4_core(&gclk0).unwrap();
        let freq = clock.freq();
        let spi = SpiConfig::new(
            &mut peripherals.PM,
            peripherals.SERCOM4,
            spi::Pads::default().data_out(pins.mosi).sclk(pins.sclk),
            freq,
        )
        .baud(3.mhz())
        .spi_mode(spi::MODE_0)
        .enable();

        let clock = clocks.sercom2_core(&gclk0).unwrap();
        let freq = clock.freq();
        let mut uart = UartConfig::new(
            &mut peripherals.PM,
            peripherals.SERCOM2,
            uart::Pads::default().rx(pins.d0).tx(pins.d1),
            freq,
        )
        .baud(250.khz(), BaudMode::Fractional(Oversampling::Bits16))
        .stop_bits(uart::StopBits::TwoBits)
        .enable();

        uart.enable_interrupts(uart::Flags::RXC);

        let buf: &'static mut [u8; BUF_LEN] =
            cortex_m::singleton!(: [u8; BUF_LEN] = [0x00; BUF_LEN]).unwrap();

        // We can use the RTC in standby for maximum power savings
        //core.SCB.set_sleepdeep();

        // blink task is causing the LED strip to flash.  Need to
        // sort out the conflict.
        // blink::spawn().unwrap();

        // Start the LED scan out.
        send_leds::spawn().unwrap();

        let mut sys_tick = cx.core.SYST;

        sys_tick.disable_counter();
        sys_tick.set_reload(200 * SYST::get_ticks_per_10ms() / 10000 * 8);
        sys_tick.clear_current();
        sys_tick.enable_counter();

        (
            Shared {
                red_led,
                uart,
                sys_tick,
                opt_transfer: None,
                buf: [0u8; 3],
            },
            Local {
                opt_spi_chan_buf: Some((spi, chan0, buf)),
                dmx_index: 0,
            },
            init::Monotonics(rtc),
        )
    }

    #[task(shared = [red_led])]
    fn blink(mut ctx: blink::Context) {
        ctx.shared.red_led.lock(|led| led.toggle().unwrap());
        blink::spawn_after(1_u32.seconds()).ok();
    }

    #[task(binds = DMAC, shared = [opt_transfer])]
    fn tcmpl(ctx: tcmpl::Context) {
        ctx.shared.opt_transfer.as_mut().unwrap().callback();
    }

    #[task(binds = SERCOM2, shared = [ sys_tick, uart, buf], local = [ dmx_index])]
    fn sercom0_irq(mut ctx: sercom0_irq::Context) {
        let dmx_index = ctx.local.dmx_index;
        let sys_tick = ctx.shared.sys_tick;
        let buf = ctx.shared.buf;
        ctx.shared.uart.lock(|uart| {
            let flags = uart.read_flags();
            let status = uart.read_status();
            if flags.contains(uart::Flags::RXC) {
                let data = unsafe { uart.read_data() };
                if sys_tick.has_wrapped() {
                    *dmx_index = 0;
                }
                if *dmx_index > 1 {
                    let dmx_addr = *dmx_index - 1;
                    if dmx_addr == DMX_ADDR {
                        buf[0] = data as u8;
                    }
                    if dmx_addr == DMX_ADDR + 1 {
                        buf[1] = data as u8;
                    }
                    if dmx_addr == DMX_ADDR + 2 {
                        buf[2] = data as u8;
                    }
                }

                *dmx_index += 1;

                sys_tick.disable_counter();
                sys_tick.clear_current();
                sys_tick.enable_counter();
            }

            uart.clear_status(status);
            uart.clear_flags(flags);
        });
    }

    #[task(shared = [opt_transfer, buf], local = [opt_spi_chan_buf])]
    fn send_leds(ctx: send_leds::Context) {
        let opt_spi_chan_buf = ctx.local.opt_spi_chan_buf;
        let opt_transfer = ctx.shared.opt_transfer;
        let r = ctx.shared.buf[0];
        let g = ctx.shared.buf[1];
        let b = ctx.shared.buf[2];

        if let Some(transfer) = opt_transfer {
            if transfer.complete() {
                let (chan, buf, spi) = opt_transfer.take().unwrap().wait();
                *opt_spi_chan_buf = Some((spi, chan, buf));
            }
        }

        if let Some((spi, chan, buf)) = opt_spi_chan_buf.take() {
            set_led(buf, 0, r, g, b);
            set_led(buf, 1, r, g, b);
            set_led(buf, 2, r, g, b);
            set_led(buf, 3, r, g, b);
            set_led(buf, 4, r, g, b);

            let callback: fn(CallbackStatus) = spi_finished;
            let transfer = spi.send_with_dma(buf, chan, callback);
            *opt_transfer = Some(transfer);
        }
    }

    fn spi_finished(_status: CallbackStatus) {
        send_leds::spawn().ok();
    }

    fn set_byte(mut buf: &mut [u8], mut data: u8) -> &mut [u8] {
        const PATTERNS: [u8; 4] = [0b1000_1000, 0b1000_1110, 0b11101000, 0b11101110];

        for _ in 0..4 {
            buf[0] = PATTERNS[((data & 0b1100_0000) >> 6) as usize];
            data <<= 2;
            buf = &mut buf[1..];
        }
        buf
    }

    fn set_led(buf: &mut [u8], index: usize, r: u8, g: u8, b: u8) {
        let buf = &mut buf[140 + index * 12..];

        let buf = set_byte(buf, g);
        let buf = set_byte(buf, r);
        set_byte(buf, b);
    }
}
