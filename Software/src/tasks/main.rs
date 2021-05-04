use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::stm32::RTC;

use crate::app;
use crate::drivers::display::Digit;
use crate::hardware::*;
use crate::utils::delay::{Delay, DelayMs};

pub fn task_main(cx: app::task_main::Context) {
    let display: &mut HwDisplay = &mut *cx.resources.display;
    let clocks: &Clocks = cx.resources.clocks;
    let rtc: &mut RTC = cx.resources.rtc;

    let mut delay = Delay::new(*clocks);

    if rtc.tr.read().st().bits() == 0 && rtc.tr.read().su().bits() == 0 {
        display.roll(&mut delay, [
            Digit::from(rtc.dr.read().yt().bits()),
            Digit::from(rtc.dr.read().yu().bits()),
            Digit::Off,
            Digit::from(rtc.dr.read().mt().bits() as u8),
            Digit::from(rtc.dr.read().mu().bits()),
            Digit::Off,
            Digit::from(rtc.dr.read().dt().bits()),
            Digit::from(rtc.dr.read().du().bits())
        ]);
        delay.delay_ms(3000u32);
    } else {
        let dot = rtc.tr.read().su().bits() % 2 == 0;
        display.clear();
        display.tubes[0].digit = Digit::from(rtc.tr.read().ht().bits());
        display.tubes[1].digit = Digit::from(rtc.tr.read().hu().bits());
        display.tubes[2].show_ldp = dot;
        display.tubes[2].show_rdp = !dot;
        display.tubes[3].digit = Digit::from(rtc.tr.read().mnt().bits());
        display.tubes[4].digit = Digit::from(rtc.tr.read().mnu().bits());
        display.tubes[5].show_ldp = dot;
        display.tubes[5].show_rdp = !dot;
        display.tubes[6].digit = Digit::from(rtc.tr.read().st().bits());
        display.tubes[7].digit = Digit::from(rtc.tr.read().su().bits());
        display.refresh();
    }

    // Run every second
    app::task_main::spawn_after(rtic::time::duration::Seconds(1u32)).unwrap();
}
