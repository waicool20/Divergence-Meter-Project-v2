use stm32f4xx_hal::time::MilliSeconds;

const SHORT_PRESS_MIN: MilliSeconds = MilliSeconds(40);
const SHORT_PRESS_MAX: MilliSeconds = MilliSeconds(60);
const LONG_PRESS_MIN: MilliSeconds = MilliSeconds(1200);

#[derive(Clone, Copy)]
pub struct Button {
    pub duration: MilliSeconds,
}

impl Button {
    pub fn new() -> Button {
        Button { duration: MilliSeconds(0) }
    }

    pub fn is_pressed(&self) -> bool {
        self.duration > MilliSeconds(0)
    }

    pub fn is_short_pressed(&self) -> bool {
        self.duration > SHORT_PRESS_MIN && self.duration < SHORT_PRESS_MAX
    }

    pub fn is_long_pressed(&self) -> bool {
        self.duration > LONG_PRESS_MIN
    }
}
