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
use core::fmt::Write;
use cortex_m::{asm};
use stm32f4xx_hal as hal;
use stm32f4::stm32f411 as target_device;
use target_device::Interrupt;

use hal::prelude::*;
use embedded_hal::digital::v2::{OutputPin};
use embedded_hal::adc::{OneShot, Channel};
use stm32f4xx_hal::gpio::{gpioa, gpiob, gpioc, gpiod, Output, PushPull, Analog, Alternate, AF5, AF7};
use stm32f4xx_hal::spi;
use stm32f4xx_hal::adc;
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


#[inline(always)]
fn delay_ns(nanos: u64) {
    const FCLK: u64 = 100_000_000;
    let cycles = (nanos * FCLK + 999_999_999) / 1_000_000_000;
    asm::delay(cycles as u32);
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
        let gpiod = device.GPIOD.split();
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
        }
    }


    #[task(priority=3, resources = [debugio, transfer_active, bl_pin, pol_pin, le_pin, int_reset_pin, adc, adc_channel, uart, uart_tx_producer], schedule=[drive])]
    fn drive(mut cx: drive::Context) {
        static mut STATE: bool = false;

        let bl_pin = cx.resources.bl_pin;
        let pol_pin = cx.resources.pol_pin;
        let le_pin = cx.resources.le_pin;
        let int_reset_pin = cx.resources.int_reset_pin;
        let adc = cx.resources.adc;
        let adc_channel = cx.resources.adc_channel;

        // Drive the HV507b polarity switching sequence
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
            cx.resources.debugio.set_high();
            adc.start_conversion();
            adc.wait_for_conversion_sequence();
            let sample0 = adc.current_sample();
            //let sample0 = 0; // adc.read(adc_channel).unwrap();
            cx.resources.debugio.set_low();
            bl_pin.set_high().ok();
            delay_ns(SAMPLE_DELAY_NS);

            cx.resources.debugio.set_high();
            adc.start_conversion();
            adc.wait_for_conversion_sequence();
            let sample1 = adc.current_sample();
            //let sample1 = 100;
            cx.resources.debugio.set_low();
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

    // Write new value to the HV507 shift register
    // It is not latched until the next polarity rise event
    #[task(priority=2, resources=[transfer_active, spi])]
    fn update_active_electrodes(mut cx: update_active_electrodes::Context, msg: ElectrodeEnableStruct) {
        cx.resources.transfer_active.lock(|flag| {
            *flag = true;
        });

        cx.resources.spi.write(&msg.values).unwrap();

        cx.resources.transfer_active.lock(|flag| {
            *flag = false;
        });
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
            Err(_) => (),
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
