use cortex_m::asm::delay;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use num_traits::FromPrimitive;

use crate::utils::units::{MilliAmps, MilliVolts};

const I2C_ADDRESS: u8 = 0x6B;
const PART_NUMBER: u8 = 0x06;

#[repr(u8)]
enum Register {
    InputSourceControl = 0,
    PowerOnConfig,
    ChargeCurrentControl,
    PreChargeTerminationCurrentControl,
    ChargeVoltageControl,
    ChargeTerminationTimerControl,
    BoostVoltageThermalRegulationControl,
    MiscOperationControl,
    SystemStatus,
    NewFault,
    VendorPartRevisionStatus,
}

#[derive(PartialEq, PartialOrd, Clone, Copy, FromPrimitive)]
pub enum InputCurrentLimit {
    _100mA = 0,
    _150mA,
    _500mA,
    _900mmA,
    _1000mA,
    _1500mA,
    _2000mA,
    _3000mA,
}

#[derive(Copy, Clone)]
pub enum BCold {
    _76Percent,
    _79Percent,
}

#[derive(Copy, Clone)]
pub enum BatteryLowVoltage {
    _2V8,
    _3V,
}

#[derive(Copy, Clone)]
pub enum BatteryRechargeThreshold {
    _0V1,
    _0V3,
}

#[derive(PartialEq, PartialOrd, Clone, Copy, FromPrimitive)]
pub enum WatchdogTimer {
    Off = 0,
    _40s,
    _80s,
    _160s,
}

#[derive(PartialEq, PartialOrd, Clone, Copy, FromPrimitive)]
pub enum FastChargeTimer {
    _5hrs = 0,
    _8hrs,
    _12hrs,
    _20hrs,
}

#[derive(PartialEq, PartialOrd, Clone, Copy, FromPrimitive)]
pub enum BHot {
    _33Percent = 0,
    _36Percent,
    _30Percent,
    VOff,
}

#[derive(PartialEq, PartialOrd, Clone, Copy, FromPrimitive)]
pub enum ThermalRegThreshold {
    _60C = 0,
    _80C,
    _100C,
    _120C,
}

#[derive(PartialEq, PartialOrd, Clone, Copy, FromPrimitive)]
pub enum VBusStatus {
    Unknown = 0,
    USB,
    Adapter,
    OTG,
}

#[derive(PartialEq, PartialOrd, Clone, Copy, FromPrimitive)]
pub enum ChargeStatus {
    NotCharging = 0,
    PreCharging,
    FastCharging,
    DoneCharging,
}

#[derive(PartialEq, PartialOrd, Clone, Copy, FromPrimitive)]
pub enum ChargeFault {
    Normal = 0,
    BadInput,
    ThermalShutdown,
    ChargeTimerExpired,
}

pub struct Faults {
    faults: u8,
}

impl Faults {
    pub fn has_watchdog_fault(&self) -> bool {
        self.faults & 0x80 > 0
    }

    pub fn has_otg_fault(&self) -> bool {
        self.faults & 0x40 > 0
    }

    pub fn get_charge_fault(&self) -> ChargeFault {
        FromPrimitive::from_u8((self.faults & 0x30) >> 4).unwrap()
    }

    pub fn has_battery_fault(&self) -> bool {
        self.faults & 0x08 > 0
    }

    pub fn is_too_cold(&self) -> bool {
        self.faults & 0x02 > 0
    }

    pub fn is_too_hot(&self) -> bool {
        self.faults & 0x01 > 0
    }
}

pub struct BatteryCharger<I> {
    i2c: I,
}

impl<I> BatteryCharger<I> where I: Read + Write + WriteRead {
    pub fn new(i2c: I) -> Option<BatteryCharger<I>> {
        let mut bc = BatteryCharger { i2c };
        if bc.get_part_no() == PART_NUMBER { Some(bc) } else { None }
    }

    pub fn is_hiz(&mut self) -> bool {
        self.read(Register::InputSourceControl) & 0x80 > 0
    }

    pub fn set_hiz(&mut self, en: bool) {
        let mut orig = self.read(Register::InputSourceControl);
        orig = orig & !0x80 | ((en as u8) << 7);
        self.write(Register::InputSourceControl, orig);
    }

    pub fn get_input_voltage_limit(&mut self) -> MilliVolts {
        let data = self.read(Register::InputSourceControl);
        MilliVolts(3880 + ((data & 0x78) >> 3) as i32 * 80)
    }

    pub fn set_input_voltage_limit(&mut self, limit: MilliVolts) {
        let data = limit.0.clamp(3880, 5080);
        let data = ((data - 3880) / 80) as u8;
        let mut orig = self.read(Register::InputSourceControl);
        orig = orig & !0x78 | (data << 3);
        self.write(Register::InputSourceControl, orig);
    }

    pub fn get_input_current_limit(&mut self) -> InputCurrentLimit {
        FromPrimitive::from_u8(self.read(Register::InputSourceControl) & 0x07).unwrap()
    }

    pub fn set_input_current_limit(&mut self, l: InputCurrentLimit) {
        let mut orig = self.read(Register::InputSourceControl);
        orig = orig & !0x07 | l as u8;
        self.write(Register::InputSourceControl, orig);
    }

    pub fn reset_registers(&mut self) {
        let mut orig = self.read(Register::PowerOnConfig);
        orig = orig & !0x80 | (1 << 7);
        self.write(Register::PowerOnConfig, orig);
    }

    pub fn reset_watchdog_timer(&mut self) {
        let mut orig = self.read(Register::PowerOnConfig);
        orig = orig & !0x40 | (1 << 6);
        self.write(Register::PowerOnConfig, orig);
    }

    pub fn is_otg_enabled(&mut self) -> bool {
        self.read(Register::PowerOnConfig) & 0x20 > 0
    }

    pub fn set_otg_enabled(&mut self, en: bool) {
        let mut orig = self.read(Register::PowerOnConfig);
        orig = orig & !0x20 | ((en as u8) << 5);
        self.write(Register::PowerOnConfig, orig);
    }

    pub fn is_charging_enabled(&mut self) -> bool {
        self.read(Register::PowerOnConfig) & 0x10 > 0
    }

    pub fn set_charging_enabled(&mut self, en: bool) {
        let mut orig = self.read(Register::PowerOnConfig);
        orig = orig & !0x10 | ((en as u8) << 4);
        self.write(Register::PowerOnConfig, orig);
    }

    pub fn get_minimum_system_voltage_limit(&mut self) -> MilliVolts {
        let data = self.read(Register::PowerOnConfig);
        MilliVolts(3000 + ((data & 0x0E) >> 1) as i32 * 100)
    }

    pub fn set_minimum_system_voltage_limit(&mut self, threshold: MilliVolts) {
        let data = threshold.0.clamp(3000, 3700);
        let data = ((data - 3000) / 100) as u8;
        let mut orig = self.read(Register::PowerOnConfig);
        orig = orig & !0x0E | (data << 1);
        self.write(Register::PowerOnConfig, orig);
    }

    pub fn get_fast_charge_current_limit(&mut self) -> MilliAmps {
        let data = self.read(Register::ChargeCurrentControl);
        MilliAmps(512 + ((data & 0xFC) >> 2) as i32 * 64)
    }

    pub fn set_fast_charge_current_limit(&mut self, limit: MilliAmps) {
        let data = limit.0.clamp(512, 3008);
        let data = ((data - 512) / 64) as u8;
        let mut orig = self.read(Register::ChargeCurrentControl);
        orig = orig & !0xFC | (data << 2);
        self.write(Register::ChargeCurrentControl, orig);
    }

    pub fn get_boost_mode_cold_temp_threshold_voltage(&mut self) -> BCold {
        let data = self.read(Register::ChargeCurrentControl) & 0x02;
        if data == 0 { BCold::_76Percent } else { BCold::_79Percent }
    }

    pub fn set_boost_mode_cold_temp_threshold_voltage(&mut self, bcold: BCold) {
        let data = match bcold {
            BCold::_76Percent => 0,
            BCold::_79Percent => 1
        };
        let mut orig = self.read(Register::ChargeCurrentControl);
        orig = orig & !0x02 | (data << 1);
        self.write(Register::ChargeCurrentControl, orig);
    }

    pub fn is_force_charge_20_percent(&mut self) -> bool {
        self.read(Register::ChargeCurrentControl) & 0x01 > 0
    }

    pub fn set_force_charge_20_percent(&mut self, en: bool) {
        let mut orig = self.read(Register::ChargeCurrentControl);
        orig = orig & !0x01 | en as u8;
        self.write(Register::ChargeCurrentControl, orig);
    }

    pub fn get_precharge_current_limit(&mut self) -> MilliAmps {
        let data = self.read(Register::PreChargeTerminationCurrentControl);
        MilliAmps(128 + ((data & 0xF0) >> 4) as i32 * 128)
    }

    pub fn set_precharge_current_limit(&mut self, limit: MilliAmps) {
        let data = limit.0.clamp(128, 2048);
        let data = ((data - 128) / 128) as u8;
        let mut orig = self.read(Register::PreChargeTerminationCurrentControl);
        orig = orig & !0xF0 | (data << 4);
        self.write(Register::PreChargeTerminationCurrentControl, orig);
    }

    pub fn get_termination_current_limit(&mut self) -> MilliAmps {
        let data = self.read(Register::PreChargeTerminationCurrentControl);
        MilliAmps(128 + (data & 0x0F) as i32 * 128)
    }

    pub fn set_termination_current_limit(&mut self, limit: MilliAmps) {
        let data = limit.0.clamp(128, 2048);
        let data = ((data - 128) / 128) as u8;
        let mut orig = self.read(Register::PreChargeTerminationCurrentControl);
        orig = orig & !0x0F | data;
        self.write(Register::PreChargeTerminationCurrentControl, orig);
    }

    pub fn get_charge_voltage_limit(&mut self) -> MilliVolts {
        let data = self.read(Register::ChargeVoltageControl);
        MilliVolts(3504 + ((data & 0xFC) >> 2) as i32 * 16)
    }

    pub fn set_charge_voltage_limit(&mut self, limit: MilliVolts) {
        let data = limit.0.clamp(3504, 4400);
        let data = ((data - 3504) / 16) as u8;
        let mut orig = self.read(Register::ChargeVoltageControl);
        orig = orig & !0xFC | (data << 2);
        self.write(Register::ChargeVoltageControl, orig);
    }

    pub fn get_battery_low_voltage(&mut self) -> BatteryLowVoltage {
        let data = self.read(Register::ChargeVoltageControl) & 0x02;
        if data == 0 { BatteryLowVoltage::_2V8 } else { BatteryLowVoltage::_3V }
    }

    pub fn set_battery_low_voltage(&mut self, blv: BatteryLowVoltage) {
        let data = match blv {
            BatteryLowVoltage::_2V8 => 0,
            BatteryLowVoltage::_3V => 1
        };
        let mut orig = self.read(Register::ChargeVoltageControl);
        orig = orig & !0x02 | (data << 1);
        self.write(Register::ChargeVoltageControl, orig);
    }

    pub fn get_battery_recharge_threshold(&mut self) -> BatteryRechargeThreshold {
        let data = self.read(Register::ChargeVoltageControl) & 0x01;
        if data == 0 { BatteryRechargeThreshold::_0V1 } else { BatteryRechargeThreshold::_0V3 }
    }

    pub fn set_battery_recharge_threshold(&mut self, bct: BatteryRechargeThreshold) {
        let data = match bct {
            BatteryRechargeThreshold::_0V1 => 0,
            BatteryRechargeThreshold::_0V3 => 1
        };
        let mut orig = self.read(Register::ChargeVoltageControl);
        orig = orig & !0x01 | data;
        self.write(Register::ChargeVoltageControl, orig);
    }

    pub fn is_charge_termination_enabled(&mut self) -> bool {
        self.read(Register::ChargeTerminationTimerControl) & 0x80 > 0
    }

    pub fn set_charge_termination_enabled(&mut self, en: bool) {
        let mut orig = self.read(Register::ChargeTerminationTimerControl);
        orig = orig & !0x80 | ((en as u8) << 7);
        self.write(Register::ChargeTerminationTimerControl, orig);
    }

    pub fn get_watchdog_timer(&mut self) -> WatchdogTimer {
        let data = self.read(Register::ChargeTerminationTimerControl);
        FromPrimitive::from_u8((data & 0x30) >> 4).unwrap()
    }

    pub fn set_watchdog_timer(&mut self, wt: WatchdogTimer) {
        let mut orig = self.read(Register::ChargeTerminationTimerControl);
        orig = orig & !0x30 | ((wt as u8) << 4);
        self.write(Register::ChargeTerminationTimerControl, orig);
    }

    pub fn is_charging_safety_timer_enabled(&mut self) -> bool {
        self.read(Register::ChargeTerminationTimerControl) & 0x08 > 0
    }

    pub fn set_charging_safety_timer_enabled(&mut self, en: bool) {
        let mut orig = self.read(Register::ChargeTerminationTimerControl);
        orig = orig & !0x08 | ((en as u8) << 3);
        self.write(Register::ChargeTerminationTimerControl, orig);
    }

    pub fn get_fast_charge_timer(&mut self) -> FastChargeTimer {
        let data = self.read(Register::ChargeTerminationTimerControl);
        FromPrimitive::from_u8((data & 0x06) >> 1).unwrap()
    }

    pub fn set_fast_charge_timer(&mut self, fct: FastChargeTimer) {
        let mut orig = self.read(Register::ChargeTerminationTimerControl);
        orig = orig & !0x06 | ((fct as u8) << 1);
        self.write(Register::ChargeTerminationTimerControl, orig);
    }

    pub fn get_boost_voltage(&mut self) -> MilliVolts {
        let data = self.read(Register::BoostVoltageThermalRegulationControl);
        MilliVolts(4550 + ((data & 0xF0) >> 4) as i32 * 64)
    }

    pub fn set_boost_voltage(&mut self, v: MilliVolts) {
        let data = v.0.clamp(4550, 5510);
        let data = ((data - 4550) / 64) as u8;
        let mut orig = self.read(Register::BoostVoltageThermalRegulationControl);
        orig = orig & !0xF0 | (data << 4);
        self.write(Register::BoostVoltageThermalRegulationControl, orig);
    }

    pub fn get_boost_mode_hot_temp_threshold_voltage(&mut self) -> BHot {
        let data = self.read(Register::BoostVoltageThermalRegulationControl);
        FromPrimitive::from_u8((data & 0x0C) >> 2).unwrap()
    }

    pub fn set_boost_mode_hot_temp_threshold_voltage(&mut self, bhot: BHot) {
        let mut orig = self.read(Register::BoostVoltageThermalRegulationControl);
        orig = orig & !0x0C | ((bhot as u8) << 2);
        self.write(Register::BoostVoltageThermalRegulationControl, orig);
    }

    pub fn get_thermal_regulation_threshold(&mut self) -> ThermalRegThreshold {
        let data = self.read(Register::BoostVoltageThermalRegulationControl);
        FromPrimitive::from_u8(data & 0x03).unwrap()
    }

    pub fn set_thermal_regulation_threshold(&mut self, trt: ThermalRegThreshold) {
        let mut orig = self.read(Register::BoostVoltageThermalRegulationControl);
        orig = orig & !0x03 | trt as u8;
        self.write(Register::BoostVoltageThermalRegulationControl, orig);
    }

    pub fn is_detecting_input_source(&mut self) -> bool {
        self.read(Register::MiscOperationControl) & 0x80 > 0
    }

    pub fn force_detect_input_source(&mut self) {
        let mut orig = self.read(Register::MiscOperationControl);
        orig = orig & !0x80 | (1 << 7);
        self.write(Register::MiscOperationControl, orig);
    }

    pub fn is_safety_timer_slowed_2x(&mut self) -> bool {
        self.read(Register::MiscOperationControl) & 0x40 > 0
    }

    pub fn set_safety_timer_slowed_2x(&mut self, en: bool) {
        let mut orig = self.read(Register::MiscOperationControl);
        orig = orig & !0x40 | ((en as u8) << 6);
        self.write(Register::MiscOperationControl, orig);
    }

    pub fn is_batfet_disabled(&mut self) -> bool {
        self.read(Register::MiscOperationControl) & 0x20 > 0
    }

    pub fn set_batfet_disabled(&mut self, d: bool) {
        let mut orig = self.read(Register::MiscOperationControl);
        orig = orig & !0x20 | ((d as u8) << 5);
        self.write(Register::MiscOperationControl, orig);
    }

    pub fn is_charge_fault_interrupt_enabled(&mut self) -> bool {
        self.read(Register::MiscOperationControl) & 0x02 > 0
    }

    pub fn set_charge_fault_interrupt_enabled(&mut self, en: bool) {
        let mut orig = self.read(Register::MiscOperationControl);
        orig = orig & !0x02 | ((en as u8) << 1);
        self.write(Register::MiscOperationControl, orig);
    }

    pub fn is_battery_fault_interrupt_enabled(&mut self) -> bool {
        self.read(Register::MiscOperationControl) & 0x01 > 0
    }

    pub fn set_battery_fault_interrupt_enabled(&mut self, en: bool) {
        let mut orig = self.read(Register::MiscOperationControl);
        orig = orig & !0x01 | en as u8;
        self.write(Register::MiscOperationControl, orig);
    }

    pub fn get_vbus_status(&mut self) -> VBusStatus {
        let data = self.read(Register::SystemStatus);
        FromPrimitive::from_u8((data & 0xC0) >> 6).unwrap()
    }

    pub fn get_charge_status(&mut self) -> ChargeStatus {
        let data = self.read(Register::SystemStatus);
        FromPrimitive::from_u8((data & 0x30) >> 4).unwrap()
    }

    pub fn is_in_dpm_mode(&mut self) -> bool {
        self.read(Register::SystemStatus) & 0x08 > 0
    }

    pub fn is_power_good(&mut self) -> bool {
        self.read(Register::SystemStatus) & 0x04 > 0
    }

    pub fn is_in_thermal_regulation(&mut self) -> bool {
        self.read(Register::SystemStatus) & 0x02 > 0
    }

    pub fn is_in_vsysmin_regulation(&mut self) -> bool {
        self.read(Register::SystemStatus) & 0x01 > 0
    }

    pub fn get_faults(&mut self) -> Faults {
        Faults { faults: self.read(Register::NewFault) }
    }

    pub fn get_part_no(&mut self) -> u8 {
        (self.read(Register::VendorPartRevisionStatus) & 0xE0) >> 5
    }

    pub fn get_part_revision(&mut self) -> u8 {
        self.read(Register::VendorPartRevisionStatus) & 0x07
    }

    fn read(&mut self, address: Register) -> u8 {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDRESS, &[address as u8], &mut buf).ok();
        delay(1 << 16);
        u8::from_le_bytes(buf)
    }

    fn write(&mut self, address: Register, data: u8) {
        self.i2c.write(I2C_ADDRESS, &[address as u8, data]).ok();
        delay(1 << 16);
    }
}
