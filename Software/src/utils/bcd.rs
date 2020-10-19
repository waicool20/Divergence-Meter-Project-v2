use num_traits::NumCast;

pub struct BCD {
    pub sign: i8,
    value: u32,
}

impl BCD {
    pub fn get_digit(&self, n: usize) -> u8 {
        let shifts = n * 4;
        (self.value >> shifts & 0x0F) as u8
    }

    pub fn to_u32(&self) -> u32 {
        let mut v = self.value;
        let mut result = 0u32;
        let mut places = 1u32;

        while v > 0 {
            result += (v & 0x0F) * places;
            places *= 10;
            v >>= 4;
        }
        result
    }
}

impl<T: NumCast> From<T> for BCD {
    fn from(x: T) -> Self {
        let mut x: i32 = num::cast(x).unwrap();
        let mut sign = 0;
        if x == 0 {
            return BCD { sign, value: 0 };
        } else if x > 0 {
            sign = 1;
        } else {
            sign = -1;
            x = x * -1;
        }

        let mut x = x as u32;
        let mut result = 0u32;
        let mut shifts = 0u32;

        while x > 0 {
            result |= (x % 10) << (shifts << 2);
            shifts += 1;
            x /= 10;
        }
        BCD { sign, value: result }
    }
}
