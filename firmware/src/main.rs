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

mod triple_buffer;
mod ws2812;

#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [EVSYS, DAC])]
mod app {
    #[cfg(feature = "semihosting")]
    use cortex_m_semihosting::hprintln;
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

    use crate::triple_buffer::{TripleBuffer, TripleBufferReader};
    use crate::ws2812::{self, Ws2812};

    const NUM_LEDS: usize = 24;
    const BUF_LEN: usize = ws2812::buffer_len(NUM_LEDS);
    const DMX_ADDR: u16 = 1;

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
        opt_spi_chan_ws: Option<(Spi, Channel<Ch0, Ready>, Ws2812<'static, BUF_LEN>)>,
        dmx_index: u16,
    }

    #[shared]
    struct Shared {
        // The LED could be a local resource, since it is only used in one task
        // But we want to showcase shared resources and locking
        #[lock_free]
        red_led: bsp::RedLed,

        #[lock_free]
        uart: Uart,

        #[lock_free]
        opt_transfer: Option<
            Transfer<Channel<Ch0, Busy>, BufferPair<&'static mut [u8], Spi>, fn(CallbackStatus)>,
        >,

        // Need a lockless reader/writer buffer here
        #[lock_free]
        color_buf_writer: TripleBuffer<u8, { 3 * NUM_LEDS }>,

        #[lock_free]
        color_buf_reader: TripleBufferReader<u8, { 3 * NUM_LEDS }>,
    }

    #[monotonic(binds = RTC, default = true)]
    type RtcMonotonic = Rtc<Count32Mode>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        #[cfg(feature = "semihosting")]
        hprintln!("dmx2led").ok();

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
        .baud(2400.khz())
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

        uart.enable_interrupts(uart::Flags::ERROR | uart::Flags::RXBRK | uart::Flags::RXC);
        let buf: &'static mut [u8; BUF_LEN] =
            cortex_m::singleton!(: [u8; BUF_LEN] = [0x00; BUF_LEN]).unwrap();
        let ws = Ws2812::new(buf);

        let color_buf: &'static mut [u8; NUM_LEDS * 3] =
            cortex_m::singleton!(: [u8; NUM_LEDS * 3] = [0x00; NUM_LEDS * 3]).unwrap();
        let (color_buf_writer, color_buf_reader) = TripleBuffer::new(color_buf);

        // We can use the RTC in standby for maximum power savings
        //core.SCB.set_sleepdeep();

        // blink task is causing the LED strip to flash.  Need to
        // sort out the conflict.
        // blink::spawn().unwrap();

        // Start the LED scan out.
        send_leds::spawn().unwrap();

        (
            Shared {
                red_led,
                uart,
                opt_transfer: None,
                color_buf_writer,
                color_buf_reader,
            },
            Local {
                opt_spi_chan_ws: Some((spi, chan0, ws)),
                dmx_index: 0,
            },
            init::Monotonics(rtc),
        )
    }

    #[task(shared = [red_led])]
    fn blink(ctx: blink::Context) {
        ctx.shared.red_led.toggle().unwrap();
        blink::spawn_after(1_u32.seconds()).ok();
    }

    #[task(binds = DMAC, priority = 2, shared = [opt_transfer])]
    fn tcmpl(ctx: tcmpl::Context) {
        ctx.shared.opt_transfer.as_mut().unwrap().callback();
    }

    #[task(binds = SERCOM2, priority = 3, shared = [uart, color_buf_writer], local = [ dmx_index])]
    fn sercom0_irq(ctx: sercom0_irq::Context) {
        let dmx_index = ctx.local.dmx_index;
        let color_buf_writer = ctx.shared.color_buf_writer;
        let uart = ctx.shared.uart;

        let flags = uart.read_flags();
        let status = uart.read_status();
        if flags.contains(uart::Flags::RXC) {
            let data = unsafe { uart.read_data() };
            if flags.contains(uart::Flags::ERROR) {
                *dmx_index = 0;
            }
            if *dmx_index > 0 {
                let dmx_addr = *dmx_index;

                if dmx_addr >= DMX_ADDR && dmx_addr < (DMX_ADDR + 3 * NUM_LEDS as u16) {
                    color_buf_writer.set((dmx_addr - DMX_ADDR) as usize, data as u8);
                }
            }

            *dmx_index += 1;
        }

        uart.clear_status(status);
        uart.clear_flags(flags);
    }

    #[task(priority = 2, shared = [opt_transfer, color_buf_reader], local = [opt_spi_chan_ws])]
    fn send_leds(ctx: send_leds::Context) {
        let opt_spi_chan_ws = ctx.local.opt_spi_chan_ws;
        let opt_transfer = ctx.shared.opt_transfer;
        let color_buf = ctx.shared.color_buf_reader;

        if let Some(transfer) = opt_transfer {
            if transfer.complete() {
                let (chan, buf, spi) = opt_transfer.take().unwrap().wait();
                *opt_spi_chan_ws = Some((spi, chan, Ws2812::new(buf)));
            }
        }
        if let Some((spi, chan, mut ws)) = opt_spi_chan_ws.take() {
            for i in 0..NUM_LEDS {
                ws.set_led(
                    i,
                    color_buf.get(i * 3 + 0),
                    color_buf.get(i * 3 + 1),
                    color_buf.get(i * 3 + 2),
                );
            }

            let callback: fn(CallbackStatus) = spi_finished;
            let buf = ws.into_buf();
            let transfer = spi.send_with_dma(buf, chan, callback);
            *opt_transfer = Some(transfer);
        }
    }

    fn spi_finished(_status: CallbackStatus) {
        send_leds::spawn().ok();
    }
}
