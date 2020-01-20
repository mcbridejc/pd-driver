#![no_std]
#![no_main]

#![feature(custom_test_frameworks)]
#![reexport_test_harness_main = "test_main"]

#![feature(alloc)]
#![feature(alloc_error_handler)]

#![test_runner(crate::test_runner)]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

//use panic_semihosting as _;

//use cortex_m::asm;
//use cortex_m::peripheral::{syst, Peripherals};
use rtfm;
use rtfm::cyccnt::{Instant, U32Ext as _};
use alloc_cortex_m::CortexMHeap;
extern crate alloc;
use self::alloc::vec::Vec;
use core::cell::RefCell;
use core::alloc::Layout;
use cortex_m_semihosting::hprintln;
use cortex_m::asm;
use stm32f4xx_hal as hal;
use stm32f4::stm32f411 as target_device;
use target_device::Interrupt;

use hal::prelude::*;
use embedded_hal::digital::v2::{OutputPin};
use stm32f4xx_hal::gpio::{gpioa, gpiob, gpioc, gpiod, Output, PushPull, Analog, Alternate, AF5, AF7};
use stm32f4xx_hal::spi;
use stm32f4xx_hal::adc;
use heapless::{
    consts::*,
    i,
    spsc::{Consumer, Producer, Queue},
};

use typenum::consts::U512;

mod hv507;
use hv507::Hv507;

use pd_driver_messages::{
    Parser,
    serialize,
    messages::*
};

/// Error type combining SPI, I2C, and Pin errors
/// You can remove anything you don't need / add anything you do
/// (as well as additional driver-specific values) here
// #[derive(Debug, Clone, PartialEq)]
// pub enum Error<PinError> {
//     /// Underlying GPIO pin error
//     Pin(PinError),
// }

// pub struct PdDriver<PolPin, ResetPin, PinError> 
// where 
//     PolPin: OutputPin,
//     ResetPin: OutputPin
// {
//     pol: PolPin,
//     reset: ResetPin,
    
//     pub(crate) err: Option<Error<PinError>>,
// }

// impl<PolPin, ResetPin, PinError> PdDriver<PolPin, ResetPin, PinError>
// where 
//     PolPin: OutputPin<Error = PinError>,
//     ResetPin: OutputPin<Error = PinError>
// {
//     pub fn new(pol: PolPin, reset: ResetPin) -> PdDriver<PolPin, ResetPin, PinError> {
//         PdDriver{pol, reset, err: None}
//     }
// }

// this is the allocator the application will use
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const HEAP_SIZE: usize = 8192; // in bytes

//type PolPin = gpiob::PB11<StatefulOutputPin>;

#[rtfm::app(device = stm32f4::stm32f411, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // A resource
        //uart: hal::serial::Serial<hal::stm32::USART3, (gpiod::PD8<Alternate<AF7>>, gpiod::PD9<Alternate<AF7>>)>,
        uart: hal::serial::Serial<hal::stm32::USART2, (gpioa::PA2<Alternate<AF7>>, gpioa::PA3<Alternate<AF7>>)>,
        uart_rx_producer: Producer<'static, u8, U512>,
        uart_rx_consumer: Consumer<'static, u8, U512>,
        uart_tx_producer: Producer<'static, u8, U512>,
        uart_tx_consumer: Consumer<'static, u8, U512>,
        driver: Hv507<
            spi::Spi<hal::stm32::SPI1, (gpiob::PB3<Alternate<AF5>>, spi::NoMiso, gpiob::PB5<Alternate<AF5>>)>,
            hal::adc::Adc<hal::stm32::ADC1>,
            gpioa::PA0<Analog>,
            gpioc::PC3<Output<PushPull>>,
            gpioc::PC2<Output<PushPull>>,
            gpiob::PB4<Output<PushPull>>,
            gpiob::PB10<Output<PushPull>>,
            (),
        >,
        parser: Parser,
    }

    #[init(schedule = [drive])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut UART_RX_BUFFER: Queue<u8, U512> = Queue(i::Queue::new());
        static mut UART_TX_BUFFER: Queue<u8, U512> = Queue(i::Queue::new());

        unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) }
        // Cortex-M peripherals
        let mut core: rtfm::Peripherals = cx.core;

        // Device specific peripherals
        let mut device: target_device::Peripherals = cx.device;

        let mut rcc = device.RCC;
        let mut flash = device.FLASH;

        // Configure PLL settings to run at 100MHz sysclk
        let clocks = rcc.constrain().cfgr.sysclk(100.mhz()).freeze();

        // Initialize (enable) the monotonic timer (CYCCNT)
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        
        let mut gpioa = device.GPIOA.split();
        let mut gpiob = device.GPIOB.split();
        let mut gpioc = device.GPIOC.split();
        let mut gpiod = device.GPIOD.split();
        let tx_pin = gpioa.pa2.into_alternate_af7();
        let rx_pin = gpioa.pa3.into_alternate_af7();
        let mut uart = hal::serial::Serial::usart2(
            device.USART2,
            (tx_pin, rx_pin),
            hal::serial::config::Config::default().baudrate(230400.bps()),
            clocks,
        ).unwrap();
        
        let (uart_rx_producer, uart_rx_consumer) = UART_RX_BUFFER.split();
        let (uart_tx_producer, uart_tx_consumer) = UART_TX_BUFFER.split();

        let pol = gpioc.pc2.into_push_pull_output();
        let bl = gpioc.pc3.into_push_pull_output();
        let le = gpiob.pb4.into_push_pull_output();
        let int_reset = gpiob.pb10.into_push_pull_output();
        let mosi = gpiob.pb5.into_alternate_af5();
        let sck = gpiob.pb3.into_alternate_af5();
        let miso = hal::spi::NoMiso{};
        let spi = hal::spi::Spi::spi1(
            device.SPI1,
            (sck, miso, mosi),
            spi::Mode{polarity: spi::Polarity::IdleLow, phase: spi::Phase::CaptureOnFirstTransition },
            8_000_000.hz(),
            clocks
        );
        let adc_config = hal::adc::config::AdcConfig::default();

        let adc = hal::adc::Adc.adc1(
            device.ADC1,
            true,
            adc_config,
        );
        let adc_channel = gpioa::PA0::into_analog();
        let driver = Hv507::new(spi, adc, adc_channel, bl, pol, le, int_reset);
        //let driver = PdDriver::new( );
        let now = cx.start; // the start time of the system
        cx.schedule.drive(now + 100_000_000u32.cycles()).unwrap();
        
        //let mut systick = core.SYST;
        // systick.set_clock_source(systick::SystClkSource::Core);
        // systick.set_reload(1_000);
        // systick.enable_counter();

        hal::block!(uart.write(52)).unwrap();
        hal::block!(uart.write(52)).unwrap();
        hal::block!(uart.write(52)).unwrap();
        
        uart.listen(hal::serial::Event::Rxne);
        
        let parser = Parser::new();

        init::LateResources { uart, uart_rx_producer, uart_rx_consumer, uart_tx_producer, uart_tx_consumer, driver, parser}
    }

    #[task(resources = [driver, uart, uart_tx_producer], schedule=[drive])]
    fn drive(mut cx: drive::Context) {
        static mut state: bool = false;

        let driver = cx.resources.driver;
        if *state {
            let (sample0, sample1) = driver.set_polarity_with_blank(true);
            let msg = ActiveCapacitanceStruct{baseline: sample0, measurement: sample1};
            let buf: Vec<u8> = msg.into();
            for b in serialize(ACTIVE_CAPACITANCE_ID, &buf) {
                cx.resources.uart_tx_producer.enqueue(b).expect("rx Overflow");
            }
            // Enable the TXE interrupt
            cx.resources.uart.lock(|uart| {
                uart.listen(hal::serial::Event::Txe);
            });
            *state = false;
        } else {
            driver.set_polarity(false);
            *state = true;
        }



        // let mut pol = &mut cx.resources.driver.pol;
        // let mut reset = &mut cx.resources.driver.reset;
        // if *state {
        //     hprintln!("POL HIGH");
        //     reset.set_low();
        //     asm::delay(1000);
        //     pol.set_high();
        //     asm::delay(1000);
        //     reset.set_high();
        //     *state = false;
        // } else {
        //     hprintln!("POL LOW");
        //     pol.set_low();
        //     *state = true;
        // }

        // Run every 1ms
        cx.schedule.drive(cx.scheduled + 100_000u32.cycles()).unwrap();
    }

    #[idle(resources=[uart_rx_consumer, parser], spawn = [update_active_electrodes])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut parser = cx.resources.parser;
        loop {
            match cx.resources.uart_rx_consumer.dequeue() {
                Some(x) => {
                    if let Some(msg) = parser.parse(x) {
                        match msg {
                            Message::ElectrodeEnableMsg(msg) => {
                                cx.spawn.update_active_electrodes(msg);
                            },
                            _ => (),
                        }
                    }
                },
                None => (),
            };
        }
    }

    #[task(resources=[driver])]
    fn update_active_electrodes(mut cx: update_active_electrodes::Context, msg: ElectrodeEnableStruct) {
        let driver = cx.resources.driver;
        driver.update_sr(&msg.values);
    }

    /// UART IRQ Handler just moves data from UART to/from tx/rx ring buffers
    #[task(binds = USART2, priority = 4, resources = [uart, uart_rx_producer, uart_tx_consumer])]
    fn uart2(cx: uart2::Context) {

        if cx.resources.uart.is_rxne() {
            match cx.resources.uart.read() {
                Ok(x) => cx.resources.uart_rx_producer.enqueue(x).expect("rx Overflow"),
                Err(_) => (),
            };
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
    }
};

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();

    loop {}
}
