use core::num::Wrapping;

pub struct RNG {
    randa: Wrapping<u32>,
    randb: Wrapping<u32>,
    randc: Wrapping<u32>,
    randx: Wrapping<u32>,
}

impl RNG {
    pub fn new(seed: [u32; 4]) -> RNG {
        RNG {
            randa: Wrapping(seed[0]),
            randb: Wrapping(seed[1]),
            randc: Wrapping(seed[2]),
            randx: Wrapping(seed[3]),
        }
    }

    pub fn current(&self) -> u32 {
        self.randc.0
    }

    pub fn next_u32(&mut self) -> u32 {
        self.randx += Wrapping(1);
        self.randa = self.randa ^ self.randc ^ self.randx;
        self.randb = self.randb + self.randa;
        self.randc = self.randc + (self.randb >> 1) ^ self.randa;
        self.randc.0
    }

    pub fn next_u16(&mut self) -> u16 {
        self.next_u32() as u16
    }

    pub fn next_u8(&mut self) -> u8 {
        self.next_u32() as u8
    }
}
