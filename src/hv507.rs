use core::cell::{Cell, RefCell};
use embedded_hal as emhal;
use emhal::adc::{Channel, OneShot};
use cortex_m::interrupt;
use cortex_m::interrupt::Mutex;
use cortex_m::asm;
use cortex_m_semihosting::hprintln;

const BLANKING_DELAY_NS: u64 = 3000;
const RESET_DELAY_NS: u64 = 2000;
const SAMPLE_DELAY_NS: u64 = 2000;

#[inline(always)]
fn delay_ns(nanos: u64) {
    const FCLK: u64 = 100_000_000;
    let cycles = ((nanos * FCLK + 999_999_999) / 1_000_000_000);
    asm::delay(cycles as u32);
}

pub struct Hv507<SPI, ADC, AdcPin, BlOutputPin, PolOutputPin, LeOutputPin, IntResetOutputPin, AdcError> 
// where AdcOneShot: OneShot<ADC, u16, AdcPin, Error=AdcError>.
{
    /// Output GPIO to control polarity on HV507
    /// Low = inverted output, high = non-inverted
    pol: RefCell<PolOutputPin>,
    /// Output GPIO to control blanking on HV507
    /// Low = blank (equivalent to setting all shift register bits 0)
    bl: RefCell<BlOutputPin>,
    /// Latch enable pin to latch shift register contents to output on HV507
    le: RefCell<LeOutputPin>,
    /// Resets the integrator voltage when set high
    int_reset: RefCell<IntResetOutputPin>,
    /// SPI bus for writing out to shift register
    spi: RefCell<SPI>,
    /// ADC for reading current sense
    adc: RefCell<ADC>,
    //adc: RefCell<OneShot<ADC, u16, Channel, Error=AdcError>>,
    adc_channel: RefCell<AdcPin>,

    /// Flag indicates when a shift register transfer is in progress
    transfer_active: Mutex<Cell<bool>>,
}

impl <SPI, ADC, AdcPin, BlOutputPin, PolOutputPin, LeOutputPin, IntResetOutputPin, SpiError, PinError, AdcError> 
Hv507<SPI, ADC, AdcPin, BlOutputPin, PolOutputPin, LeOutputPin, IntResetOutputPin, AdcError> 
where
    SpiError: core::fmt::Debug,
    PinError: core::fmt::Debug,
    AdcError: core::fmt::Debug,
    ADC: Adc<hal::stm32::ADC1, u16, AdcPin, Error = AdcError>,
    SPI: emhal::blocking::spi::Write<u8, Error = SpiError>,
    AdcPin: emhal::adc::Channel<hal::stm32::ADC1, ID = u8>,
    BlOutputPin: emhal::digital::v2::OutputPin<Error = PinError>,
    PolOutputPin: emhal::digital::v2::OutputPin<Error = PinError>,
    LeOutputPin: emhal::digital::v2::OutputPin<Error = PinError>,
    IntResetOutputPin: emhal::digital::v2::OutputPin<Error = PinError>,
{
    pub fn new(spi: SPI, adc: AdcOneShot, adc_channel: AdcPin, bl: BlOutputPin, pol: PolOutputPin, le: LeOutputPin, int_reset: IntResetOutputPin) -> 
        Hv507<SPI, AdcOneShot, AdcPin, BlOutputPin, PolOutputPin, LeOutputPin, IntResetOutputPin, AdcError> 
    {
        Hv507 { 
            spi: RefCell::new(spi),
            adc: RefCell::new(adc),
            adc_channel: RefCell::new(adc_channel),
            bl: RefCell::new(bl),
            pol: RefCell::new(pol),
            le: RefCell::new(le),
            int_reset: RefCell::new(int_reset), 
            transfer_active: Mutex::new(Cell::new(false)) 
        }
    }

    pub fn update_sr(&self, data: &[u8]) -> Result<(), SpiError> {
        if data.len() != 16 {
            panic!("Need 16 bytes to write to HV507, one bit per each of 128 electrodes");
        }
        interrupt::free(|cs| {
            self.transfer_active.borrow(cs).set(true);
        });

        self.spi.borrow_mut().write(data)?;
        
        interrupt::free(|cs| {
            self.transfer_active.borrow(cs).set(false);
        });
        Ok(())
    }

    pub fn set_polarity(&self, value: bool) {
        if value {
            self.pol.borrow_mut().set_high();
        } else {
            self.pol.borrow_mut().set_low();
        }
    }

    /// Change the polarity, while wrapping it in a blanking sequence,
    /// latching the latest shift register value, and sampling the 
    pub fn set_polarity_with_blank(&self, value: bool) -> (u16, u16) {
        // Enable blanking
        // Borrow these all up front because borrowing is crazy slow. 
        // These could be optimized by combining them all behind a single
        // RefCell, or maybe I should do something else entirely. Driver needs
        // to be internally mutable, so that it can be shared among tasks; it should
        // be safe as long as one task only calls update_sr, and the other only
        // calls set_polarity methods. 
        let mut bl = self.bl.borrow_mut();
        let mut pol = self.pol.borrow_mut();
        let mut le = self.le.borrow_mut();
        let mut int_reset = self.int_reset.borrow_mut();
        bl.set_low();
        pol.set_high();
        // self.bl.borrow_mut().set_low().unwrap();
        // // Set the POL bit
        // self.set_polarity(value);
        // Set latch enable, unless we are currently in the process of shifting new data
        interrupt::free(|cs| {
            if !self.transfer_active.borrow(cs).get() {
                le.set_low();
                asm::delay(10); // Datasheet says 80ns min LE pulse width
                le.set_high();
            }
        });

        delay_ns(BLANKING_DELAY_NS);
        //asm::delay(300);
        int_reset.set_low();
        delay_ns(RESET_DELAY_NS);
        //asm::delay(200);
        // TODO: SAMPLE ADC
        let sample0 = 0;
        //self.bl.borrow_mut().set_high().unwrap();
        bl.set_high();
        delay_ns(SAMPLE_DELAY_NS);
        // TODO: SAMPLE ADC
        let sample1 = 100;

        (sample0, sample1)
    }
}
