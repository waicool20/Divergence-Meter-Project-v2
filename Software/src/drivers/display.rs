use core::ops::{AddAssign, SubAssign};

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use num::NumCast;
use num_traits::Unsigned;

use embedded_hal::prelude::*;

use crate::drivers::shift_register::ShiftRegister;
use crate::utils::delay::Delay;
use crate::utils::rng::RNG;

#[derive(Copy, Clone, PartialEq, FromPrimitive)]
#[allow(dead_code)]
pub enum Digit { Num0 = 0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9, Off }

impl Digit {
    fn data(&self) -> u32 {
        match *self {
            Digit::Off => 0,
            Digit::Num0 => 2,
            _ => 1 << (11 - *self as u32)
        }
    }
}

impl<T> From<T> for Digit
    where T: NumCast + Unsigned {
    fn from(t: T) -> Self {
        num::FromPrimitive::from_u32(num::cast(t).unwrap()).unwrap()
    }
}

impl AddAssign<u8> for Digit {
    fn add_assign(&mut self, rhs: u8) {
        if *self == Digit::Off { return; }
        let rhs = (rhs + *self as u8) % 10;
        *self = num::FromPrimitive::from_u8(rhs).unwrap()
    }
}

impl SubAssign<u8> for Digit {
    fn sub_assign(&mut self, rhs: u8) {
        if *self == Digit::Off { return; }
        let rhs = (*self as i8 - rhs as i8) % 10;
        *self = num::FromPrimitive::from_i8(rhs).unwrap()
    }
}

#[derive(Copy, Clone)]
pub struct Tube {
    pub digit: Digit,
    pub show_ldp: bool,
    pub show_rdp: bool,
}

impl Tube {
    fn new() -> Tube {
        Tube {
            digit: Digit::Off,
            show_ldp: false,
            show_rdp: false,
        }
    }

    fn data(self) -> u32 {
        self.digit.data() | ((self.show_ldp as u32) << 11) | (self.show_rdp as u32)
    }

    fn data_len() -> u32 { 12 }

    pub fn clear(&mut self) {
        self.digit = Digit::Off;
        self.show_ldp = false;
        self.show_rdp = false;
    }
}

pub struct Display<PinDat, PinClk, PinLe, PinEn, PinPwm> {
    pub tubes: [Tube; 8],
    enabled: bool,
    shift_register: ShiftRegister<PinDat, PinClk, PinLe>,
    pin_en: PinEn,
    pwm: PinPwm,
}

impl<PinDat, PinClk, PinLe, PinEn, PinPwm, T> Display<PinDat, PinClk, PinLe, PinEn, PinPwm>
    where PinDat: OutputPin,
          PinClk: OutputPin,
          PinLe: OutputPin,
          PinEn: OutputPin,
          PinPwm: PwmPin<Duty=T>,
          T: NumCast
{
    pub fn new(pin_dat: PinDat,
               pin_clk: PinClk,
               pin_le: PinLe,
               pin_en: PinEn,
               pwm: PinPwm,
    ) -> Display<PinDat, PinClk, PinLe, PinEn, PinPwm> {
        let mut display = Display {
            tubes: [Tube::new(); 8],
            enabled: false,
            shift_register: ShiftRegister::new(pin_dat, pin_clk, pin_le),
            pin_en,
            pwm,
        };
        display.set_brightness(1.0);
        display.clear();
        display.refresh();
        display.enable();
        display
    }

    pub fn refresh(&mut self) {
        for &tube in self.tubes.iter().rev() {
            self.shift_register.shift(tube.data(), Tube::data_len());
        }
        self.shift_register.commit();
    }

    pub fn clear(&mut self) {
        self.tubes.iter_mut().for_each(|x| x.clear());
    }

    pub fn set_brightness(&mut self, level: f32) {
        let x = num::cast(self.pwm.get_max_duty().to_f32().unwrap() * level).unwrap();
        self.pwm.set_duty(x);
    }

    pub fn get_brightness(&self) -> f32 {
        self.pwm.get_duty().to_f32().unwrap() / self.pwm.get_max_duty().to_f32().unwrap()
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn enable(&mut self) {
        self.enabled = true;
        self.pwm.enable();
        self.pin_en.set_high().ok();
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.pwm.disable();
        self.pin_en.set_low().ok();
    }

    pub fn roll(&mut self, delay: &mut Delay, result: [Digit; 8]) {
        self.roll_world_line(delay, true, result);
    }

    pub fn roll_world_line(&mut self, delay: &mut Delay, roll_tube_1: bool, result: [Digit; 8]) {
        let mut rng = RNG::new([result[0] as u32, result[2] as u32, result[4] as u32, result[6] as u32]);
        self.tubes.iter_mut()
            .filter(|x| x.digit == Digit::Off)
            .for_each(|x| {
                x.clear();
                x.digit = Digit::Num1
            });

        if !roll_tube_1 {
            self.tubes[1].clear();
            self.tubes[1].show_rdp = true;
        }
        let mut rand_char = [
            ((rng.next_u8() % 10 + 2) * 10) + (10 - self.tubes[0].digit as u8) + result[0] as u8 % 10,
            ((rng.next_u8() % 10 + 2) * 10) + (10 - self.tubes[1].digit as u8) + result[1] as u8 % 10,
            ((rng.next_u8() % 10 + 2) * 10) + (10 - self.tubes[2].digit as u8) + result[2] as u8 % 10,
            ((rng.next_u8() % 10 + 2) * 10) + (10 - self.tubes[3].digit as u8) + result[3] as u8 % 10,
            ((rng.next_u8() % 10 + 2) * 10) + (10 - self.tubes[4].digit as u8) + result[4] as u8 % 10,
            ((rng.next_u8() % 10 + 2) * 10) + (10 - self.tubes[5].digit as u8) + result[5] as u8 % 10,
            ((rng.next_u8() % 10 + 2) * 10) + (10 - self.tubes[6].digit as u8) + result[6] as u8 % 10,
            ((rng.next_u8() % 10 + 2) * 10) + (10 - self.tubes[7].digit as u8) + result[7] as u8 % 10
        ];

        let max_number = *rand_char.iter().max().unwrap();

        for _ in 0..max_number {
            for j in 0..8 {
                if !roll_tube_1 && j == 1 {
                    continue;
                }
                if rand_char[j] > 0 {
                    self.tubes[j].digit += 1;
                    rand_char[j] -= 1;
                } else {
                    if result[j] == Digit::Off {
                        self.tubes[j].digit = Digit::Off
                    }
                }
            }
            self.refresh();
            delay.delay_ms(50u16);
        }
    }
}
