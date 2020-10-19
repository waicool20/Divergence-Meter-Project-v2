#![feature(clamp)]
#![no_std]
#![no_main]

#![allow(unused_must_use)]
#![allow(dead_code)]

#[macro_use]
extern crate num_derive;
extern crate panic_halt;
extern crate void;

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use embedded_hal::digital::v2::InputPin;
use rtic::cyccnt::U32Ext;
use shared_bus::I2cProxy;
use stm32f4xx_hal::gpio::{AF4, AlternateOD, Edge, ExtiPin, Floating, GpioExt, Input, Output, PushPull};
use stm32f4xx_hal::gpio::gpioa::*;
use stm32f4xx_hal::gpio::gpiob::*;
use stm32f4xx_hal::gpio::gpioc::*;
use stm32f4xx_hal::gpio::gpioc::PC5;
use stm32f4xx_hal::i2c::I2c;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::pwm::{C2, PwmChannels, tim4};
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::serial::config::Config;
use stm32f4xx_hal::serial::Serial;
use stm32f4xx_hal::stm32;
use stm32f4xx_hal::stm32::{I2C1, RTC, TIM4};
use stm32f4xx_hal::time::MilliSeconds;

use crate::drivers::battery_charger::*;
use crate::drivers::battery_gauge::*;
use crate::drivers::button::Button;
use crate::drivers::display::*;
use crate::utils::delay::Delay;
use crate::utils::units::I32Extensions;

mod drivers;
mod utils;

type _Display = Display<PB4<Output<PushPull>>, PB5<Output<PushPull>>, PB6<Output<PushPull>>, PA15<Output<PushPull>>, PwmChannels<TIM4, C2>>;
type _I2cBus = I2cProxy<'static, Mutex<RefCell<I2c<I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)>>>>;
type _BatteryCharger = BatteryCharger<_I2cBus>;
type _UnsealedBatteryGauge = BatteryGauge<Unsealed, _I2cBus>;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        display: _Display,
        clocks: Clocks,
        rtc: RTC,
        buttons: [Button; 5],
        pin_b1: PA5<Input<Floating>>,
        pin_b2: PA6<Input<Floating>>,
        pin_b3: PA7<Input<Floating>>,
        pin_b4: PC4<Input<Floating>>,
        pin_b5: PC5<Input<Floating>>,
        battery_charger: _BatteryCharger,
        battery_gauge: _UnsealedBatteryGauge,
        vbus_detect: PC9<Input<Floating>>,
    }

    #[init(schedule = [task_main, task_buttons, task_charger])]
    fn init(cx: init::Context) -> init::LateResources {
        let mut cp: rtic::Peripherals = cx.core;
        let mut dp: stm32::Peripherals = cx.device;
        cp.DWT.enable_cycle_counter();

        // Init RTC
        dp.RCC.apb1enr.modify(|_, w| w.pwren().enabled());
        dp.PWR.cr.modify(|_, w| w.dbp().set_bit());
        if dp.RCC.bdcr.read().lserdy().is_not_ready() {
            dp.RCC.bdcr.modify(|_, w| w.lseon().on());
            while dp.RCC.bdcr.read().lserdy().is_not_ready() {}
        }
        if dp.RCC.bdcr.read().rtcen().is_disabled() {
            dp.RCC.bdcr.modify(|_, w| {
                w.rtcsel().lse();
                w.rtcen().enabled()
            });
        }

        dp.RCC.apb2enr.modify(|_, w| w.syscfgen().enabled());

        let clocks = dp.RCC.constrain().cfgr.use_hse(8.mhz())
            .sysclk(48.mhz())
            .freeze();

        let mut gpiob = dp.GPIOB.split();
        let mut gpioa = dp.GPIOA.split();
        let mut gpioc = dp.GPIOC.split();

        // Init Buttons
        let pin_b1 = gpioa.pa5.into_floating_input();
        let pin_b2 = gpioa.pa6.into_floating_input();
        let pin_b3 = gpioa.pa7.into_floating_input();
        let pin_b4 = gpioc.pc4.into_floating_input();
        let pin_b5 = gpioc.pc5.into_floating_input();

        // Init Display
        let pin_le = gpiob.pb6.into_push_pull_output();
        let pin_dat = gpiob.pb4.into_push_pull_output();
        let pin_clk = gpiob.pb5.into_push_pull_output();
        let pin_en = gpioa.pa15.into_push_pull_output();
        let pin_bl = gpiob.pb7.into_alternate_af2();
        let pwm = tim4(dp.TIM4, pin_bl, clocks, 1.khz());
        let mut display = Display::new(pin_dat, pin_clk, pin_le, pin_en, pwm);

        // Init Bluetooth Serial
        let bt_tx = gpioa.pa2.into_alternate_af7();
        let bt_rx = gpioa.pa3.into_alternate_af7();
        let serial = Serial::usart2(dp.USART2, (bt_tx, bt_rx), Config::default().baudrate(9600.bps()), clocks).unwrap();
        let (mut tx, mut rx) = serial.split();

        // Init Battery I2C
        let batt_scl = gpiob.pb8.into_alternate_af4_open_drain();
        let batt_sda = gpiob.pb9.into_alternate_af4_open_drain();
        let i2c = I2c::i2c1(dp.I2C1, (batt_scl, batt_sda), 400.khz(), clocks);
        let bus: &'static _ = shared_bus::new_cortexm!(I2c<I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)> = i2c).unwrap();
        let mut battery_charger = BatteryCharger::new(bus.acquire_i2c()).unwrap();
        let mut battery_gauge = BatteryGauge::new(bus.acquire_i2c()).unwrap()
            .unseal_full_access(DEFAULT_UNSEAL_KEY, DEFAULT_FULL_ACCESS_KEY).ok().unwrap();

        // Init USB
        let mut usb_sel = gpioa.pa8.into_push_pull_output();

        let mut vbus_detect = gpioc.pc9.into_floating_input();
        vbus_detect.make_interrupt_source(&mut dp.SYSCFG);
        vbus_detect.enable_interrupt(&mut dp.EXTI);
        vbus_detect.trigger_on_edge(&mut dp.EXTI, Edge::RISING);

        // Start main
        cx.schedule.task_main(cx.start).unwrap();
        cx.schedule.task_buttons(cx.start).unwrap();
        cx.schedule.task_charger(cx.start).unwrap();

        init::LateResources {
            display,
            clocks,
            rtc: dp.RTC,
            buttons: [Button::new(); 5],
            pin_b1,
            pin_b2,
            pin_b3,
            pin_b4,
            pin_b5,
            battery_charger,
            battery_gauge,
            vbus_detect,
        }
    }

    #[task(schedule = [task_main], resources = [display, & clocks, rtc])]
    fn task_main(cx: task_main::Context) {
        let display: &mut _Display = &mut *cx.resources.display;
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
        cx.schedule.task_main(cx.scheduled + (clocks.sysclk().0 / 2_000 * 1000).cycles()).unwrap();
    }

    #[task(priority = 2, schedule = [task_buttons], resources = [buttons, & clocks, pin_b1, pin_b2, pin_b3, pin_b4, pin_b5])]
    fn task_buttons(cx: task_buttons::Context) {
        let buttons: &mut [Button; 5] = cx.resources.buttons;
        let clocks: &Clocks = cx.resources.clocks;
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
        cx.schedule.task_buttons(cx.scheduled + (clocks.sysclk().0 / 2_000 * 10).cycles()).unwrap();
    }

    #[task(binds = EXTI9_5, schedule = [task_charger], resources = [& clocks, vbus_detect])]
    fn task_vbus_detect(cx: task_vbus_detect::Context) {
        let clocks: &Clocks = cx.resources.clocks;
        let vbus_detect: &mut PC9<Input<Floating>> = cx.resources.vbus_detect;
        vbus_detect.clear_interrupt_pending_bit();
        cx.schedule.task_charger(cx.start + (clocks.sysclk().0 / 2_000 * 5000).cycles()).unwrap();
    }

    #[task(schedule = [task_charger], resources = [& clocks, battery_charger])]
    fn task_charger(cx: task_charger::Context) {
        let battery_charger: &mut _BatteryCharger = cx.resources.battery_charger;
        let clocks: &Clocks = cx.resources.clocks;

        // Try again in 100ms if battery charger is still detecting input source
        if battery_charger.is_detecting_input_source() {
            cx.schedule.task_charger(cx.scheduled + (clocks.sysclk().0 / 2_000 * 100).cycles()).unwrap();
            return;
        }

        if battery_charger.get_vbus_status() != VBusStatus::USB {
            battery_charger.set_input_current_limit(InputCurrentLimit::_3000mA);
            battery_charger.set_input_voltage_limit(4600.mv());
            battery_charger.set_fast_charge_current_limit(3008.ma());
        }

        battery_charger.set_charge_voltage_limit(4250.mv());
        battery_charger.set_charging_safety_timer_enabled(false);
        battery_charger.set_watchdog_timer(WatchdogTimer::Off);
    }

    extern "C" {
        fn SDIO();
        fn OTG_FS_WKUP();
    }
};

/*#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = stm32::Peripherals::take().unwrap();

    dp.RCC.apb1enr.modify(|_, w| w.pwren().enabled());
    dp.PWR.cr.modify(|_, w| w.dbp().set_bit());
    if dp.RCC.bdcr.read().lserdy().is_not_ready() {
        dp.RCC.bdcr.modify(|_, w| w.lseon().on());
        while dp.RCC.bdcr.read().lserdy().is_not_ready() {}
    }
    if dp.RCC.bdcr.read().rtcen().is_disabled() {
        dp.RCC.bdcr.modify(|_, w| {
            w.rtcsel().lse();
            w.rtcen().enabled()
        });
    }

    let mut rcc: rcc::Rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store
    // the frozen frequencies in `clocks`
    let clocks = rcc.cfgr.use_hse(8.mhz())
        .sysclk(48.mhz())
        .hclk(24.mhz())
        .freeze();

    let mut gpiob = dp.GPIOB.split();
    let mut gpioa = dp.GPIOA.split();
    let mut gpioc = dp.GPIOC.split();

    let mut delay = Delay::new(clocks);

    let mut pin_le = gpiob.pb6.into_push_pull_output();
    let mut pin_dat = gpiob.pb4.into_push_pull_output();
    let mut pin_clk = gpiob.pb5.into_push_pull_output();
    let mut pin_en = gpioa.pa15.into_push_pull_output();

    let pin_bl = gpiob.pb7.into_alternate_af2();
    let mut pwm = tim4(dp.TIM4, pin_bl, clocks, 1.khz());

    let mut display = Display::new(&mut pin_dat, &mut pin_clk, &mut pin_le, &mut pin_en, &mut pwm);

    let bt_tx = gpioa.pa2.into_alternate_af7();
    let bt_rx = gpioa.pa3.into_alternate_af7();

    let serial = Serial::usart2(dp.USART2, (bt_tx, bt_rx), Config::default().baudrate(9600.bps()), clocks).unwrap();
    let (mut tx, mut rx) = serial.split();

    let batt_scl = gpiob.pb8.into_alternate_af4_open_drain();
    let batt_sda = gpiob.pb9.into_alternate_af4_open_drain();

    let i2c = RefCell::new(I2c::i2c1(dp.I2C1, (batt_scl, batt_sda), 400.khz(), clocks));

    let mut usb_sel = gpioa.pa8.into_push_pull_output();
    usb_sel.set_low();

    let mut vbus_detect = gpioc.pc9.into_floating_input();

    delay.delay_ms(1000u16);

    let mut battery_charger = BatteryCharger::new(&i2c).unwrap();

    //let mut battery_gauge = BatteryGauge::new(&i2c).unwrap()
    //   .unseal_full_access(DEFAULT_UNSEAL_KEY, DEFAULT_FULL_ACCESS_KEY).ok().unwrap();

    let mut rng = RNG::new([1, 2, 3, 4]);
    //loop {}
    let mut dot = false;
    let mut on_usb = false;
    loop {
        if vbus_detect.is_high().unwrap() {
            if !on_usb {
                display.clear();
                display.refresh();
                battery_charger.reset_registers();
                battery_charger.set_input_current_limit(InputCurrentLimit::_3000mA);
                battery_charger.set_input_voltage_limit(4600.mv());
                battery_charger.set_fast_charge_current_limit(3008.ma());
                battery_charger.set_charge_voltage_limit(4250.mv());
                battery_charger.set_charging_safety_timer_enabled(false);
                battery_charger.set_watchdog_timer(WatchdogTimer::Off);
                delay.delay_ms(1000u16);
            }
            on_usb = true;
        } else {
            on_usb = false;
        }
        if display.tubes[6].digit as u8 == 5 && display.tubes[7].digit as u8 == 9 {
            roll_world_line(&mut display, &mut delay, true, [
                Digit::from(dp.RTC.dr.read().yt().bits()),
                Digit::from(dp.RTC.dr.read().yu().bits()),
                Digit::Off,
                Digit::from(dp.RTC.dr.read().mt().bits() as u8),
                Digit::from(dp.RTC.dr.read().mu().bits()),
                Digit::Off,
                Digit::from(dp.RTC.dr.read().dt().bits()),
                Digit::from(dp.RTC.dr.read().du().bits())
            ]);
            delay.delay_ms(1000u16);
        }

        display.clear();
        display.tubes[0].digit = Digit::from(dp.RTC.tr.read().ht().bits());
        display.tubes[1].digit = Digit::from(dp.RTC.tr.read().hu().bits());
        display.tubes[2].show_ldp = dot;
        display.tubes[2].show_rdp = !dot;
        display.tubes[3].digit = Digit::from(dp.RTC.tr.read().mnt().bits());
        display.tubes[4].digit = Digit::from(dp.RTC.tr.read().mnu().bits());
        display.tubes[5].show_ldp = dot;
        display.tubes[5].show_rdp = !dot;
        display.tubes[6].digit = Digit::from(dp.RTC.tr.read().st().bits());
        display.tubes[7].digit = Digit::from(dp.RTC.tr.read().su().bits());
        display.refresh();
        dot = !dot;
        delay.delay_ms(500u16);
    }
}*/
