use rtic::time::duration::Milliseconds;

use crate::app;
use crate::drivers::battery_charger::{InputCurrentLimit, VBusStatus, WatchdogTimer};
use crate::hardware::*;
use crate::utils::units::I32Extensions;

pub fn task_charger(cx: app::task_charger::Context) {
    let battery_charger: &mut HwBatteryCharger = cx.resources.battery_charger;

    // Try again in 100ms if battery charger is still detecting input source
    if battery_charger.is_detecting_input_source() {
        app::task_charger::spawn_after(Milliseconds(100u32)).unwrap();
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
