use embedded_hal::digital::v2::OutputPin;

pub struct ShiftRegister<PinDat, PinClk, PinLe> {
    pin_dat: PinDat,
    pin_clk: PinClk,
    pin_le: PinLe,
}

impl<PinDat, PinClk, PinLe> ShiftRegister<PinDat, PinClk, PinLe>
    where PinDat: OutputPin,
          PinClk: OutputPin,
          PinLe: OutputPin
{
    pub fn new(pin_dat: PinDat,
               pin_clk: PinClk,
               pin_le: PinLe,
    ) -> ShiftRegister<PinDat, PinClk, PinLe> {
        ShiftRegister {
            pin_dat,
            pin_clk,
            pin_le,
        }
    }

    pub fn shift(&mut self, data: u32, bits: u32) {
        let mut x = data;
        for _ in 0..bits {
            if x & 0x01 == 1 {
                self.pin_dat.set_high().ok();
            } else {
                self.pin_dat.set_low().ok();
            }
            self.pin_clk.set_high().ok();
            self.pin_clk.set_low().ok();
            x = x.wrapping_shr(1);
        }
    }

    pub fn commit(&mut self) -> () {
        self.pin_le.set_high().ok();
        self.pin_le.set_low().ok();
    }
}
