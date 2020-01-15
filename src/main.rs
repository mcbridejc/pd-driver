#![no_std]
#![no_main]

#![feature(custom_test_frameworks)]
#![reexport_test_harness_main = "test_main"]


#![test_runner(crate::test_runner)]

// pick a panicking behavior
//extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use panic_semihosting as _;

//use cortex_m::asm;
//use cortex_m::peripheral::{syst, Peripherals};
use rtfm;
use cortex_m_semihosting::hprintln;
use stm32f4xx_hal as hal;
use stm32f4::stm32f413 as target_device;
use target_device::Interrupt;

use hal::prelude::*;
use stm32f4xx_hal::gpio::{gpiod, Alternate, AF7};

use heapless::{
    consts::*,
    i,
    spsc::{Consumer, Producer, Queue},
};

use typenum::consts::U512;

#[rtfm::app(device = stm32f4::stm32f413, peripherals = true)]
const APP: () = {
    struct Resources {
        // A resource
        uart: hal::serial::Serial<hal::stm32::USART3, (gpiod::PD8<Alternate<AF7>>, gpiod::PD9<Alternate<AF7>>)>,
        //uart2_rx: hal::serial::Rx<hal::stm32::USART2>,
        uart_rx_producer: Producer<'static, u8, U512>,
        uart_rx_consumer: Consumer<'static, u8, U512>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut X: u32 = 0;
        static mut UART_BUFFER: Queue<u8, U512> = Queue(i::Queue::new());

        // Cortex-M peripherals
        let core: cortex_m::Peripherals = cx.core;

        // Device specific peripherals
        let device: target_device::Peripherals = cx.device;

        let mut rcc = device.RCC.constrain();
        // 
        // clock configuration using the default settings (all clocks run at 8 MHz)
        let clocks = rcc.cfgr.freeze();
        
        let mut gpiod = device.GPIOD.split();
        let tx_pin = gpiod.pd8.into_alternate_af7();
        let rx_pin = gpiod.pd9.into_alternate_af7();
        let mut uart = hal::serial::Serial::usart3(
            device.USART3,
            (tx_pin, rx_pin),
            hal::serial::config::Config::default().baudrate(115_200.bps()),
            clocks,
        ).unwrap();
        //let (uart2_tx, uart2_rx) = uart2.split();
        let (uart_rx_producer, uart_rx_consumer) = UART_BUFFER.split();
        
        let mut systick = core.SYST;
        // systick.set_clock_source(systick::SystClkSource::Core);
        // systick.set_reload(1_000);
        // systick.enable_counter();
        hprintln!("Here I am").unwrap();

        hal::block!(uart.write(52)).unwrap();
        hal::block!(uart.write(52)).unwrap();
        hal::block!(uart.write(52)).unwrap();
        
        uart.listen(hal::serial::Event::Rxne);

        rtfm::pend(Interrupt::USART3);

        init::LateResources { uart, uart_rx_producer, uart_rx_consumer }
    }

    #[idle(resources=[uart, uart_rx_consumer])]
    fn idle(mut cx: idle::Context) -> ! {
        static mut X: u32 = 0;

        // Safe access to local `static mut` variable
        let _x: &'static mut u32 = X;

        hprintln!("Still here").unwrap();

        //debug::exit(debug::EXIT_SUCCESS);

        loop {
            match cx.resources.uart_rx_consumer.dequeue() {
                Some(x) => cx.resources.uart.lock(|uart| {
                    hal::block!(uart.write(x)).unwrap();
                }),
                None => (),
            };
        }
    }

    // `shared` can be accessed from this context
    #[task(binds = USART3, resources = [uart, uart_rx_producer])]
    fn uart3(cx: uart3::Context) {
        // let shared: &mut u32 = cx.resources.shared;
        // *shared += 1;
        match cx.resources.uart.read() {
            Ok(x) => cx.resources.uart_rx_producer.enqueue(x).expect("rx Overflow"),
            Err(_) => (),
        };

        hprintln!("UART2").unwrap(); //: shared = {}", shared).unwrap();
    }
};


// #[entry]
// fn main() -> ! {
//     asm::nop(); // To not have main optimize to abort in release mode, remove when you add code

//     let mut peripherals = Peripherals::take().unwrap();
//     let mut systick = peripherals.SYST;
//     systick.set_clock_source(syst::SystClkSource::Core);
//     systick.set_reload(1_000);
//     systick.clear_current();
//     systick.enable_counter();

//     hprintln!("Here we go...").unwrap();
//     let pac_periphs = PacPeripherals::take().unwrap();
//     let mut pins = pac_periphs.PORT.split();
//     let mut red_led = pins.pa23.into_open_drain_output(&mut pins.port);

//     //let redLed = hal::gpio::Parts::Pa23.into_push_pull_output();
//     //let mut port = samperiphs.PORT;


//     // port.dir0.write(|w| unsafe {w.bits((1<<23))});
//     // port.out0.write(|w| unsafe {w.bits((1<<23))});
//     // while !systick.has_wrapped() {
//     //     // Loop
//     // }

//     loop {
//         // for _ in 0..10000000 {
//         //     asm::nop();
//         //     asm::nop();
//         //     asm::nop();
//         // }
//         //port.out0.write(|w| unsafe {w.bits((1<<23))});
//         // for _ in 0..10000000 {
//         //     asm::nop();
//         //     asm::nop();
//         //     asm::nop();
//         // }
//         //port.out0.write(|w| unsafe {w.bits(0)});

//         // your code goes here
//     }
// }


// #[cfg(test)]
// fn test_runner(tests: &[&dyn Fn()]) {
//     println!("Running {} tests", tests.len());
//     for test in tests {
//         test();
//     }
// }


// #[no_mangle]
// pub extern "C" fn _start() -> ! {
//     hprintln!("Hello World{}", "!");

//     #[cfg(test)]
//     test_main();

//     loop {}
// }

// #[cfg(test)]
// mod tests {
//     #[test_case]
//     fn test_cb_write() {
//         assert_eq!(1, 0);
//     }
// }