#![no_std]
#![no_main]

#![feature(custom_test_frameworks)]
#![reexport_test_harness_main = "test_main"]

//#![feature(alloc)]
#![feature(alloc_error_handler)]
#![test_runner(crate::test_runner)]

// pick a panicking behavior
// extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
extern crate panic_itm; // logs messages over ITM; requires ITM support
//extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m::{iprintln};
use rtfm;
use rtfm::cyccnt::{U32Ext as _};
use alloc_cortex_m::CortexMHeap;
extern crate alloc;
use self::alloc::vec::Vec;
use core::alloc::Layout;
use cortex_m::{asm};
use stm32f4xx_hal as hal;
use stm32f4::stm32f411 as target_device;

use hal::prelude::*;
use embedded_hal::digital::v2::{OutputPin};
use stm32f4xx_hal::gpio::{gpioa, gpiob, gpioc, Output, PushPull, Analog, Alternate, AF5, AF7};
use stm32f4xx_hal::adc;
use stm32f4xx_hal::spi;
use heapless::{
    i,
    spsc::{Consumer, Producer, Queue},
};

use typenum::consts::U512;

use pd_driver_messages::{
    Parser,
    serialize,
    messages::*
};

// Create an allocator for heap usage
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const HEAP_SIZE: usize = 8192; // in bytes

const BLANKING_DELAY_NS: u64 = 14000;
const RESET_DELAY_NS: u64 = 1000;
const SAMPLE_DELAY_NS: u64 = 5000;
const FULL_READ_PERIOD: u32 = 500;
const N_PINS: usize = 128;

#[inline(always)]
fn delay_ns(nanos: u64) {
    const FCLK: u64 = 100_000_000;
    let cycles = (nanos * FCLK + 999_999_999) / 1_000_000_000;
    asm::delay(cycles as u32);
}

#[derive(Clone, Default)]
pub struct ElectrodeState {
    pins: [u8; N_PINS/8],
    version: u32,
}

#[rtfm::app(device = stm32f4::stm32f411, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // A resource
        //uart: hal::serial::Serial<hal::stm32::USART3, (gpiod::PD8<Alternate<AF7>>, gpiod::PD9<Alternate<AF7>>)>,
        //uart: hal::serial::Serial<hal::stm32::USART2, (gpioa::PA2<Alternate<AF7>>, gpioa::PA3<Alternate<AF7>>)>,
        uart: hal::serial::Serial<hal::stm32::USART1, (gpioa::PA9<Alternate<AF7>>, gpioa::PA10<Alternate<AF7>>)>,
        uart_rx_producer: Producer<'static, u8, U512>,
        uart_rx_consumer: Consumer<'static, u8, U512>,
        uart_tx_producer: Producer<'static, u8, U512>,
        uart_tx_consumer: Consumer<'static, u8, U512>,
        bl_pin: gpioc::PC3<Output<PushPull>>,
        pol_pin: gpioc::PC2<Output<PushPull>>,
        le_pin: gpiob::PB4<Output<PushPull>>,
        int_reset_pin: gpiob::PB10<Output<PushPull>>,
        spi: spi::Spi<hal::stm32::SPI1, (gpioa::PA5<Alternate<AF5>>, spi::NoMiso, gpiob::PB5<Alternate<AF5>>)>,
        adc: adc::Adc<hal::stm32::ADC1>,
        adc_channel: gpioa::PA0<Analog>,
        #[init(false)]
        transfer_active: bool,
        parser: Parser,
        debugio: gpioc::PC0<Output<PushPull>>,
        electrodes: ElectrodeState,
    }

    #[init(schedule = [drive])]
    fn init(cx: init::Context) -> init::LateResources {

        static mut UART_RX_BUFFER: Queue<u8, U512> = Queue(i::Queue::new());
        static mut UART_TX_BUFFER: Queue<u8, U512> = Queue(i::Queue::new());

        unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) }
        // Cortex-M peripherals
        let mut core: rtfm::Peripherals = cx.core;

        // Device specific peripherals
        let device: target_device::Peripherals = cx.device;

        let stim = &mut core.ITM.stim[0];
        iprintln!(stim, "Running");

        let rcc = device.RCC;

        // Configure PLL settings to run at 100MHz sysclk
        let clocks = rcc.constrain().cfgr.sysclk(100.mhz()).freeze();

        // Initialize (enable) the monotonic timer (CYCCNT)
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();
        let _gpiod = device.GPIOD.split();
        let tx_pin = gpioa.pa9.into_alternate_af7();
        let rx_pin = gpioa.pa10.into_alternate_af7();
        let mut uart = hal::serial::Serial::usart1(
            device.USART1,
            (tx_pin, rx_pin),
            hal::serial::config::Config::default().baudrate(230400.bps()),
            clocks,
        ).unwrap();

        let (uart_rx_producer, uart_rx_consumer) = UART_RX_BUFFER.split();
        let (uart_tx_producer, uart_tx_consumer) = UART_TX_BUFFER.split();

        let debugio = gpioc.pc0.into_push_pull_output();
        let pol_pin = gpioc.pc2.into_push_pull_output();
        let bl_pin = gpioc.pc3.into_push_pull_output();
        let le_pin = gpiob.pb4.into_push_pull_output();
        let int_reset_pin = gpiob.pb10.into_push_pull_output();
        let mosi = gpiob.pb5.into_alternate_af5();
        let sck = gpioa.pa5.into_alternate_af5();
        let miso = hal::spi::NoMiso{};
        let spi = hal::spi::Spi::spi1(
            device.SPI1,
            (sck, miso, mosi),
            spi::Mode{polarity: spi::Polarity::IdleLow, phase: spi::Phase::CaptureOnFirstTransition },
            8_000_000.hz(),
            clocks
        );
        let adc_config = adc::config::AdcConfig::default();
        adc_config.default_sample_time(adc::config::SampleTime::Cycles_3);

        let adc = adc::Adc::adc1(
            device.ADC1,
            true,
            adc_config,
        );
        let adc_channel = gpioa.pa0.into_analog();
        let now = cx.start; // the start time of the system
        cx.schedule.drive(now + 100_000_000u32.cycles()).unwrap();

        hal::block!(uart.write(52)).unwrap();
        hal::block!(uart.write(52)).unwrap();
        hal::block!(uart.write(52)).unwrap();

        uart.listen(hal::serial::Event::Rxne);

        let parser = Parser::new();

        let electrodes = ElectrodeState::default();

        init::LateResources {
            uart,
            uart_rx_producer,
            uart_rx_consumer,
            uart_tx_producer,
            uart_tx_consumer,
            bl_pin,
            pol_pin,
            le_pin,
            int_reset_pin,
            spi,
            adc,
            adc_channel,
            parser,
            debugio,
            electrodes,
        }
    }


    #[task(priority=3, resources = [debugio, spi, electrodes, transfer_active, bl_pin, pol_pin, le_pin, int_reset_pin, adc, adc_channel, uart, uart_tx_producer], schedule=[drive])]
    fn drive(mut cx: drive::Context) {

        static mut STATE: bool = false;
        static mut LAST_ELECTRODE_VERSION: u32 = 0xFFFF;
        static mut LAST_SENT_PAGE: usize = 0;
        static mut FULL_READ_COUNTER: u32 = 0;
        static mut FULL_READ_DATA: [u16; N_PINS] = [0u16; N_PINS];
        const PAGE_SIZE: usize = 4;
        let bl_pin = cx.resources.bl_pin;
        let pol_pin = cx.resources.pol_pin;
        let le_pin = cx.resources.le_pin;
        let int_reset_pin = cx.resources.int_reset_pin;
        let adc = cx.resources.adc;
        let adc_channel = cx.resources.adc_channel;


        // Every FULL_READ_PERIOD cycles, we do a scan of all electrodes
        // The results are sent as pages in short messages interleaved with the
        // active electrode measurements so that the large amount of data generated
        // by the full read doesn't cause a bubble in the active electrode messages.
        *FULL_READ_COUNTER += 1;
        if *FULL_READ_COUNTER == FULL_READ_PERIOD {
            // Clear all bits in the shift registers, except the first
            for _ in 0..N_PINS/8 - 1 {
                cx.resources.spi.write(&[0]).unwrap();
            }
            cx.resources.spi.write(&[1]).unwrap();
            // Temporarily take over the SCLK pin
            let peripherals = unsafe {hal::stm32::Peripherals::steal()};
            let gpioa = peripherals.GPIOA.split();
            let gpiob = peripherals.GPIOB.split();
            let mut sckpin = gpioa.pa5.into_push_pull_output();
            let mut mosipin = gpiob.pb5.into_push_pull_output();

            pol_pin.set_high().ok();
            bl_pin.set_low().ok();
            mosipin.set_low().ok();
            // Empirically, it is taking a long time for the current to settle here, and this
            // delay is necessary to prevent excessive noise while sampling the first electrodes
            // This needs more investigation.
            delay_ns(200000);

            for i in 0..N_PINS {
                if N_PINS - 1 - i == 15 {
                    // This pin is the top plate; skip it.
                    sckpin.set_high().ok();
                    delay_ns(80);
                    sckpin.set_low().ok();
                    continue;
                }
                int_reset_pin.set_low().ok();
                delay_ns(RESET_DELAY_NS);

                // Take the pre-pulse/baseline sample
                adc.start_conversion();
                adc.wait_for_conversion_sequence();
                let sample0 = adc.current_sample();
                bl_pin.set_high().ok();
                delay_ns(SAMPLE_DELAY_NS);
                // Measure integrator after current pulse
                cx.resources.debugio.set_high().ok();
                adc.start_conversion();
                adc.wait_for_conversion_sequence();
                let sample1 = adc.current_sample();
                cx.resources.debugio.set_low().ok();

                FULL_READ_DATA[N_PINS - 1 - i] = sample1 - sample0;
                bl_pin.set_low().ok();
                sckpin.set_high().ok();
                int_reset_pin.set_high().ok();
                delay_ns(80);
                sckpin.set_low().ok();
                le_pin.set_low().ok();
                delay_ns(80);
                le_pin.set_high().ok();
                delay_ns(4000);
            }

            // Put it back into alternate function for the SPI controller
            mosipin.into_alternate_af5();
            sckpin.into_alternate_af5();
            *FULL_READ_COUNTER = 0;
            *LAST_SENT_PAGE = 0;
            // Restore the active electrode settings
            cx.resources.spi.write(&cx.resources.electrodes.pins).unwrap();
            *LAST_ELECTRODE_VERSION = cx.resources.electrodes.version;
        } else if cx.resources.electrodes.version != *LAST_ELECTRODE_VERSION {
            // Write new shift register value if there is one (from serial messages)
            cx.resources.spi.write(&cx.resources.electrodes.pins).unwrap();
            *LAST_ELECTRODE_VERSION = cx.resources.electrodes.version;
        }

        // Drive the HV507 polarity switching sequence
        // On polarity rise, also measure the capacitance of the activated electrodes
        // int_reset resets the analog integrator to a near-zero when driven high.
        // When low, the integrator is free to integrate the current pulse.
        if *STATE {
            adc.configure_channel(adc_channel, adc::config::Sequence::One, adc::config::SampleTime::Cycles_3);
            bl_pin.set_low().ok();
            pol_pin.set_high().ok();
            if !*cx.resources.transfer_active {
                le_pin.set_low().ok();
                delay_ns(80); // datasheet says 80ns min LE pulse width
                le_pin.set_high().ok();
            }

            delay_ns(BLANKING_DELAY_NS);
            int_reset_pin.set_low().ok();
            delay_ns(RESET_DELAY_NS);
            adc.start_conversion();
            adc.wait_for_conversion_sequence();
            let sample0 = adc.current_sample();
            bl_pin.set_high().ok();
            delay_ns(SAMPLE_DELAY_NS);

            adc.start_conversion();
            adc.wait_for_conversion_sequence();
            let sample1 = adc.current_sample();
            int_reset_pin.set_high().ok();
            let msg = ActiveCapacitanceStruct{baseline: sample0, measurement: sample1};
            let buf: Vec<u8> = msg.into();
            for b in serialize(ACTIVE_CAPACITANCE_ID, &buf) {
                match cx.resources.uart_tx_producer.enqueue(b) {
                    Ok(_) => (),
                    Err(_) => (),
                }
            }
            // Enable the TXE interrupt
            cx.resources.uart.lock(|uart| {
                uart.listen(hal::serial::Event::Txe);
            });

            if *LAST_SENT_PAGE + PAGE_SIZE <= N_PINS {
                let start_index = *LAST_SENT_PAGE;
                let mut values: Vec<u16> = Vec::new();
                for i in start_index..start_index+PAGE_SIZE {
                    values.push(FULL_READ_DATA[i]);
                }
                // Send the next page
                let msg = BulkCapacitanceStruct{start_index: start_index as u8, values};
                let buf: Vec<u8> = msg.into();
                for b in serialize(BULK_CAPACITANCE_ID, &buf) {
                    cx.resources.uart_tx_producer.enqueue(b).ok();
                }
                *LAST_SENT_PAGE += PAGE_SIZE;
            }
            *STATE = false;
        } else {
            pol_pin.set_low().ok();
            *STATE = true;
        }

        // Run every 1ms
        cx.schedule.drive(cx.scheduled + 100_000u32.cycles()).unwrap();
    }

    /// The idle task handles parsing of incoming messages
    #[idle(resources=[uart_rx_consumer, parser], spawn = [update_active_electrodes])]
    fn idle(cx: idle::Context) -> ! {
        let parser = cx.resources.parser;
        loop {
            match cx.resources.uart_rx_consumer.dequeue() {
                Some(x) => {
                    if let Some(msg) = parser.parse(x) {
                        match msg {
                            Message::ElectrodeEnableMsg(msg) => {
                                cx.spawn.update_active_electrodes(msg).unwrap();
                            },
                            _ => (),
                        }
                    }
                },
                None => (),
            };
        }
    }

    #[task(priority=2, resources=[electrodes])]
    fn update_active_electrodes(mut cx: update_active_electrodes::Context, msg: ElectrodeEnableStruct) {
        // Store the new electrode settings in shared data structure, and increment the version
        // The drive task will handle the SPI transfer, so that it can be synchronized with the
        // rest of the HV507 control activity
        cx.resources.electrodes.lock(|electrodes| {
            for i in 0..N_PINS/8 {
                electrodes.pins[i] = msg.values[i];
            }
            electrodes.version = (electrodes.version + 1) % 32768;
        })
    }

    /// UART IRQ Handler just moves data from UART to/from tx/rx ring buffers
    #[task(binds = USART1, priority = 4, resources = [uart, uart_rx_producer, uart_tx_consumer])]
    fn uart(cx: uart::Context) {
        static mut OVFCOUNT: u32 = 0;

        // Try to read; it may return an error such as an ORE or WouldBlock if no data
        // is available to read, but we have to read it every time we get an IRQ in
        // order to ensure an ORE flag gets cleared
        let result = cx.resources.uart.read();
        match result {
            Ok(x) => cx.resources.uart_rx_producer.enqueue(x).expect("Rx overflow"),
            Err(_) => *OVFCOUNT += 1,
        }

        if cx.resources.uart.is_txe() {
            match cx.resources.uart_tx_consumer.dequeue() {
                Some(x) => cx.resources.uart.write(x).unwrap(),
                None => cx.resources.uart.unlisten(hal::serial::Event::Txe),
            }
        }
    }

    // Interrupt handlers used by RTFM to dispatch software tasks
    extern "C" {
        fn TIM2();
        fn TIM3();
    }
};

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();

    loop {}
}
