use rtic::time::duration::Seconds;
use stm32f4xx_hal::gpio::{ExtiPin, Floating, Input};
use stm32f4xx_hal::gpio::gpioc::PC9;

use crate::app;

pub fn task_vbus_detect(cx: app::task_vbus_detect::Context) {
    let vbus_detect: &mut PC9<Input<Floating>> = cx.resources.vbus_detect;
    vbus_detect.clear_interrupt_pending_bit();
    app::task_charger::spawn_after(Seconds(5u32)).unwrap();
}
