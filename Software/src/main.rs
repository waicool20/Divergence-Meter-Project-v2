#![no_std]
#![no_main]

#![allow(unused_must_use)]
#![allow(dead_code)]

#[macro_use]
extern crate num_derive;
extern crate panic_halt;
extern crate void;

mod drivers;
mod utils;
mod tasks;
mod hardware;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [SDIO, OTG_FS_WKUP])]
mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtic::rtic_monotonic::Seconds;
    use stm32f4xx_hal::gpio::*;
    use stm32f4xx_hal::gpio::gpioa::*;
    use stm32f4xx_hal::gpio::gpioc::*;
    use stm32f4xx_hal::gpio::gpioc::PC5;
    use stm32f4xx_hal::i2c::I2c;
    use stm32f4xx_hal::prelude::*;
    use stm32f4xx_hal::pwm::tim4;
    use stm32f4xx_hal::rcc::Clocks;
    use stm32f4xx_hal::serial::config::Config;
    use stm32f4xx_hal::serial::Serial;
    use stm32f4xx_hal::stm32;
    use stm32f4xx_hal::stm32::RTC;

    use crate::drivers::battery_charger::*;
    use crate::drivers::battery_gauge::*;
    use crate::drivers::button::Button;
    use crate::drivers::display::*;
    use crate::hardware::*;
    use crate::tasks::*;

    #[resources]
    struct Resources {
        #[lock_free]
        display: HwDisplay,
        clocks: Clocks,
        #[lock_free]
        rtc: RTC,
        #[lock_free]
        buttons: [Button; 5],
        #[lock_free]
        pin_b1: PA5<Input<Floating>>,
        #[lock_free]
        pin_b2: PA6<Input<Floating>>,
        #[lock_free]
        pin_b3: PA7<Input<Floating>>,
        #[lock_free]
        pin_b4: PC4<Input<Floating>>,
        #[lock_free]
        pin_b5: PC5<Input<Floating>>,
        #[lock_free]
        battery_charger: HwBatteryCharger,
        #[lock_free]
        battery_gauge: HwBatteryGauge,
        #[lock_free]
        vbus_detect: PC9<Input<Floating>>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<8_000_000>;

    #[init]
    fn init(cx: init::Context) -> (init::LateResources, init::Monotonics) {
        let mut cp: cortex_m::Peripherals = cx.core;
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

        let mut syscfg = &mut dp.SYSCFG.constrain();

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
        let i2c = I2c::new(dp.I2C1, (batt_scl, batt_sda), 400.khz(), clocks);
        let bus: &'static _ = shared_bus::new_cortexm!(HwI2c = i2c).unwrap();
        let mut battery_charger = BatteryCharger::new(bus.acquire_i2c()).unwrap();
        let mut battery_gauge = BatteryGauge::new(bus.acquire_i2c()).unwrap()
            .unseal_full_access(DEFAULT_UNSEAL_KEY, DEFAULT_FULL_ACCESS_KEY).ok().unwrap();

        // Init USB
        let mut usb_sel = gpioa.pa8.into_push_pull_output();

        let mut vbus_detect = gpioc.pc9.into_floating_input();
        vbus_detect.make_interrupt_source(syscfg);
        vbus_detect.enable_interrupt(&mut dp.EXTI);
        vbus_detect.trigger_on_edge(&mut dp.EXTI, Edge::RISING);

        let mono = DwtSystick::new(&mut cp.DCB, cp.DWT, cp.SYST, 8_000_000);

        // Start main
        task_main::spawn().unwrap();
        task_buttons::spawn().unwrap();
        task_charger::spawn().unwrap();

        (init::LateResources {
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
        }, init::Monotonics(mono))
    }

    extern "Rust" {
        #[task(resources = [display, & clocks, rtc])]
        fn task_main(_cx: task_main::Context);

        #[task(priority = 2, resources = [buttons, & clocks, pin_b1, pin_b2, pin_b3, pin_b4, pin_b5])]
        fn task_buttons(_cx: task_buttons::Context);

        #[task(resources = [& clocks, battery_charger])]
        fn task_charger(_cx: task_charger::Context);

        #[task(binds = EXTI9_5, resources = [& clocks, vbus_detect])]
        fn task_vbus_detect(cx: task_vbus_detect::Context);
    }
}
