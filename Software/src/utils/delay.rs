use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use stm32f4xx_hal::rcc::Clocks;

pub struct Delay {
    clocks: Clocks
}

impl Delay {
    pub fn new(clocks: Clocks) -> Delay {
        Delay { clocks }
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(ms as u32);
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(ms as u32);
    }
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        cortex_m::asm::delay(self.clocks.sysclk().0 / 2_000 * ms);
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, ms: u8) {
        self.delay_us(ms as u32);
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, ms: u16) {
        self.delay_us(ms as u32);
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, ms: u32) {
        cortex_m::asm::delay(self.clocks.sysclk().0 / 2_000_000 * ms);
    }
}
