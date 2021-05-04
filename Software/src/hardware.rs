use shared_bus::{CortexMMutex, I2cProxy};
use stm32f4xx_hal::gpio::{AF4, AlternateOD, Output, PushPull};
use stm32f4xx_hal::gpio::gpioa::PA15;
use stm32f4xx_hal::gpio::gpiob::{PB4, PB5, PB6, PB8, PB9};
use stm32f4xx_hal::i2c::I2c;
use stm32f4xx_hal::pwm::{C2, PwmChannels};
use stm32f4xx_hal::stm32::{I2C1, TIM4};

use crate::drivers::battery_charger::BatteryCharger;
use crate::drivers::battery_gauge::{BatteryGauge, Unsealed};
use crate::drivers::display::Display;

pub type HwI2c = I2c<I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)>;
pub type HwDisplay = Display<PB4<Output<PushPull>>, PB5<Output<PushPull>>, PB6<Output<PushPull>>, PA15<Output<PushPull>>, PwmChannels<TIM4, C2>>;

type _I2CBus = I2cProxy<'static, CortexMMutex<HwI2c>>;
pub type HwBatteryCharger = BatteryCharger<_I2CBus>;
pub type HwBatteryGauge = BatteryGauge<Unsealed, _I2CBus>;
