use embedded_hal::digital::v2::InputPin;
use rtic::time::duration::Milliseconds;
use stm32f4xx_hal::gpio::{Floating, Input};
use stm32f4xx_hal::gpio::gpioa::{PA5, PA6, PA7};
use stm32f4xx_hal::gpio::gpioc::{PC4, PC5};
use stm32f4xx_hal::time::MilliSeconds;

use crate::app;
use crate::drivers::button::Button;

pub fn task_buttons(cx: app::task_buttons::Context) {
    let buttons: &mut [Button; 5] = cx.resources.buttons;
    let pin_b1: &mut PA5<Input<Floating>> = cx.resources.pin_b1;
    let pin_b2: &mut PA6<Input<Floating>> = cx.resources.pin_b2;
    let pin_b3: &mut PA7<Input<Floating>> = cx.resources.pin_b3;
    let pin_b4: &mut PC4<Input<Floating>> = cx.resources.pin_b4;
    let pin_b5: &mut PC5<Input<Floating>> = cx.resources.pin_b5;

    if pin_b1.is_high().unwrap() {
        buttons[0].duration = MilliSeconds(buttons[0].duration.0 + 10)
    } else {
        buttons[0].duration = MilliSeconds(0)
    }

    if pin_b2.is_high().unwrap() {
        buttons[1].duration = MilliSeconds(buttons[1].duration.0 + 10)
    } else {
        buttons[1].duration = MilliSeconds(0)
    }

    if pin_b3.is_high().unwrap() {
        buttons[2].duration = MilliSeconds(buttons[2].duration.0 + 10)
    } else {
        buttons[2].duration = MilliSeconds(0)
    }

    if pin_b4.is_high().unwrap() {
        buttons[3].duration = MilliSeconds(buttons[3].duration.0 + 10)
    } else {
        buttons[3].duration = MilliSeconds(0)
    }

    if pin_b5.is_high().unwrap() {
        buttons[4].duration = MilliSeconds(buttons[4].duration.0 + 10)
    } else {
        buttons[4].duration = MilliSeconds(0)
    }

    // Run every 10 ms
    app::task_buttons::spawn_after(Milliseconds(10u32)).unwrap();
}
