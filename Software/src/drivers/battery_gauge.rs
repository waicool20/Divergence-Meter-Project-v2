use core::marker::PhantomData;
use core::num::Wrapping;

use cortex_m::asm::delay;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use crate::utils::units::*;

pub const DEFAULT_UNSEAL_KEY: u32 = 0x0414_3672;
pub const DEFAULT_FULL_ACCESS_KEY: u32 = 0xFFFF_FFFF;

const I2C_ADDRESS: u8 = 0x55;
const DEVICE_TYPE: u16 = 0x0510;
const MIN_FIRMWARE_VERSION: u16 = 0x0400;

#[repr(u8)]
enum StandardCommand {
    Control = 0x00,
    AtRate = 0x02,
    AtRateTimeToEmpty = 0x04,
    Temperature = 0x06,
    Voltage = 0x08,
    Flags = 0x0A,
    NominalAvailableCapacity = 0x0C,
    FullAvailableCapacity = 0x0E,
    RemainingCapacity = 0x10,
    FullChargeCapacity = 0x12,
    AverageCurrent = 0x14,
    TimeToEmpty = 0x16,
    StandbyCurrent = 0x18,
    StandbyTimeToEmpty = 0x1A,
    StateOfHealth = 0x1C,
    CycleCount = 0x1E,
    StateOfCharge = 0x20,
    InstantaneousCurrent = 0x22,
    InternalTemperature = 0x28,
    ResistanceScale = 0x2A,
    OperationConfiguration = 0x2C,
    DesignCapacity = 0x2E,
    UnfilteredRM = 0x6C,
    FilteredRM = 0x6E,
    UnfilteredFCC = 0x70,
    FilteredFCC = 0x72,
    TrueSOC = 0x74,
}

#[repr(u16)]
enum ControlSubCommand {
    ControlStatus = 0x0000,
    DeviceType = 0x0001,
    FwVersion = 0x0002,
    PrevMacwrite = 0x0007,
    ChemId = 0x0008,
    OcvCmd = 0x000C,
    BatInsert = 0x000D,
    BatRemove = 0x000E,
    SetHibernate = 0x0011,
    ClearHibernate = 0x0012,
    SetSnooze = 0x0013,
    ClearSnooze = 0x0014,
    DfVersion = 0x001F,
    Sealed = 0x0020,
    ItEnabled = 0x0021,
    Reset = 0x0041,
}

#[repr(u16)]
enum ExtendedDataCommands {
    DataFlashClass = 0x3E,
    DataFlashBlock = 0x3F,
    BlockData = 0x40,
    BlockDataCheckSum = 0x60,
    BlockDataControl = 0x61,
    ApplicationStatus = 0x6A,
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum SubClass {
    Safety = 2,
    ChargeInhibitCfg = 32,
    Charge = 34,
    ChargeTermination = 36,
    ConfigurationData = 48,
    Discharge = 49,
    Registers = 64,
    Power = 68,
    ManufactureInfo = 57,
    ItCfg = 80,
    CurrentThresholds = 81,
    State = 82,
    Ocva0Table = 83,
    Ocva1Table = 84,
    Def0Ra = 87,
    Def1Ra = 88,
    Pack0Ra = 91,
    Pack1Ra = 92,
    Pack0Rax = 93,
    Pack1Rax = 94,
    CalibrationData = 104,
    TempModel = 106,
    Current = 107,
    Codes = 112,
}

pub enum StateOfHealth {
    Invalid,
    Uninitialized,
    Instant { percent: u8 },
    Initial { percent: u8 },
    Normal { percent: u8 },
}

pub enum Error {
    InvalidArgumentError
}

pub struct ControlStatus {
    data: u16,
}

impl ControlStatus {
    pub fn is_full_access_sealed(&self) -> bool {
        self.data & 0x4000 > 0
    }

    pub fn is_sealed(&self) -> bool {
        self.data & 0x2000 > 0
    }

    pub fn is_coulomb_counter_calibration_active(&self) -> bool {
        self.data & 0x0800 > 0
    }

    pub fn is_board_calibration_active(&self) -> bool {
        self.data & 0x0400 > 0
    }

    pub fn has_executed_ocv(&self) -> bool {
        self.data & 0x0200 > 0
    }

    pub fn has_ocv_reading_failed(&self) -> bool {
        self.data & 0x0100 > 0
    }

    pub fn has_initialized(&self) -> bool {
        self.data & 0x0080 > 0
    }

    pub fn is_hibernate_pending(&self) -> bool {
        self.data & 0x0040 > 0
    }

    pub fn is_snooze_mode_enabled(&self) -> bool {
        self.data & 0x0020 > 0
    }

    pub fn is_in_sleep_mode(&self) -> bool {
        self.data & 0x0010 > 0
    }

    pub fn is_impedance_track_algorithm_using_constant_power_model(&self) -> bool {
        self.data & 0x0008 > 0
    }

    pub fn is_resistance_table_updates_disabled(&self) -> bool {
        self.data & 0x0004 > 0
    }

    pub fn is_voltage_ok(&self) -> bool {
        self.data & 0x0002 > 0
    }

    pub fn is_qmax_updates_enabled(&self) -> bool {
        self.data & 0x0001 > 0
    }
}

pub struct Flags {
    data: u16,
}

impl Flags {
    pub fn is_over_temperature_in_charge_detected(&self) -> bool {
        self.data & 0x8000 > 0
    }

    pub fn is_over_temperature_in_discharge_detected(&self) -> bool {
        self.data & 0x4000 > 0
    }

    pub fn is_calibrating(&self) -> bool {
        self.data & 0x1000 > 0
    }

    pub fn is_charge_temperature_inhibited(&self) -> bool {
        self.data & 0x0800 > 0
    }

    pub fn is_charge_suspended(&self) -> bool {
        self.data & 0x0400 > 0
    }

    pub fn is_full_charged_detected(&self) -> bool {
        self.data & 0x0200 > 0
    }

    pub fn is_ok_to_charge(&self) -> bool {
        self.data & 0x0100 > 0
    }

    pub fn good_ocv_taken(&self) -> bool {
        self.data & 0x0020 > 0
    }

    pub fn is_waiting_to_identify_battery(&self) -> bool {
        self.data & 0x0010 > 0
    }

    pub fn is_battery_detected(&self) -> bool {
        self.data & 0x0008 > 0
    }

    pub fn is_soc1_threshold_reached(&self) -> bool {
        self.data & 0x0004 > 0
    }

    pub fn should_system_shutdown(&self) -> bool {
        self.data & 0x0002 > 0
    }

    pub fn is_discharging_detected(&self) -> bool {
        self.data & 0x0001 > 0
    }
}

#[derive(FromPrimitive)]
pub enum PinFunctionCode {
    DedicatedNA = 0,
    DedicatedChargeInhibit,
    SharedNA,
    FollowFlagFCBit,
}

#[derive(FromPrimitive)]
pub enum GpoutInterruptFunction {
    BatLow = 0,
    BatGood,
    SocInterrupt,
}

pub struct OperationConfiguration {
    data: u16,
}

impl OperationConfiguration {
    pub fn new() -> OperationConfiguration {
        OperationConfiguration { data: 0x0975 }
    }

    pub fn is_using_loaded_rate(&self) -> bool {
        self.data & 0x8000 == 0
    }

    pub fn set_using_loaded_rate(&mut self, en: bool) {
        self.data = self.data & !0x8000 | (!en as u16) << 15
    }

    pub fn is_gpout_override(&self) -> bool {
        self.data & 0x4000 > 0
    }

    pub fn set_gpout_override(&mut self, en: bool) {
        self.data = self.data & !0x4000 | (en as u16) << 14
    }

    pub fn is_battery_removal_interrupt_enabled(&self) -> bool {
        self.data & 0x2000 > 0
    }

    pub fn set_battery_removal_interrupt_enabled(&mut self, en: bool) {
        self.data = self.data & !0x2000 | (en as u16) << 13
    }

    pub fn get_pin_function_code(&self) -> PinFunctionCode {
        num::FromPrimitive::from_u16((self.data & 0x1800) >> 11).unwrap()
    }

    pub fn set_pin_function_code(&mut self, pfc: PinFunctionCode) {
        self.data = self.data & !1800 | (pfc as u16) << 11;
    }

    pub fn is_first_ocv_interrupt_enabled(&self) -> bool {
        self.data & 0x0080 > 0
    }

    pub fn set_first_ocv_interrupt_enabled(&mut self, en: bool) {
        self.data = self.data & !0x0080 | (en as u16) << 7
    }

    pub fn is_cell_identification_enabled(&self) -> bool {
        self.data & 0x0040 > 0
    }

    pub fn set_cell_identification_enabled(&mut self, en: bool) {
        self.data = self.data & !0x0040 | (en as u16) << 6
    }

    pub fn is_sleep_enabled(&self) -> bool {
        self.data & 0x0020 > 0
    }

    pub fn set_sleep_enabled(&mut self, en: bool) {
        self.data = self.data & !0x0020 | (en as u16) << 5
    }

    pub fn is_rm_updated_from_fcc(&self) -> bool {
        self.data & 0x0010 > 0
    }

    pub fn set_rm_updated_from_fcc(&mut self, en: bool) {
        self.data = self.data & !0x0010 | (en as u16) << 4
    }

    pub fn get_gpout_polarity(&self) -> bool {
        self.data & 0x0008 > 0
    }

    pub fn set_gpout_polarity(&mut self, active_high: bool) {
        self.data = self.data & !0x0008 | (active_high as u16) << 3
    }

    pub fn get_gpout_interrupt_function(&self) -> GpoutInterruptFunction {
        num::FromPrimitive::from_u16((self.data & 0x0006) >> 1).unwrap()
    }

    pub fn set_gpout_interrupt_function(&mut self, f: GpoutInterruptFunction) {
        self.data = self.data & !0x0006 | (f as u16) << 1;
    }

    pub fn is_using_external_thermistor(&self) -> bool {
        self.data & 0x0001 > 0
    }

    pub fn set_using_external_thermistor(&mut self, en: bool) {
        self.data = self.data & !0x0001 | en as u16;
    }
}

pub struct ApplicationStatus {
    data: u8,
}

impl ApplicationStatus {
    pub fn is_unsupported_battery(&self) -> bool {
        self.data & 0x02 > 0
    }

    pub fn last_used_cell_profile(&self) -> u8 {
        self.data & 0x01
    }
}

pub struct Sealed;

pub struct Unsealed;

pub struct BatteryGauge<SealState, I> {
    i2c: I,
    _state: PhantomData<SealState>,
}

impl<SealState, I> BatteryGauge<SealState, I> where I: Read + Write + WriteRead {
    pub fn get_control_status(&mut self) -> ControlStatus {
        ControlStatus { data: self.read_control_command(ControlSubCommand::ControlStatus) }
    }

    pub fn get_device_type(&mut self) -> u16 {
        self.read_control_command(ControlSubCommand::DeviceType)
    }

    pub fn get_firmware_version(&mut self) -> u16 {
        self.read_control_command(ControlSubCommand::FwVersion)
    }

    pub fn get_prev_macwrite(&mut self) -> u16 {
        self.read_control_command(ControlSubCommand::PrevMacwrite)
    }

    pub fn get_chem_id(&mut self) -> u16 {
        self.read_control_command(ControlSubCommand::ChemId)
    }

    pub fn do_ocv(&mut self) {
        self.execute_control_command(ControlSubCommand::OcvCmd);
    }

    pub fn do_bat_insert(&mut self) {
        self.execute_control_command(ControlSubCommand::BatInsert);
    }

    pub fn do_bat_remove(&mut self) {
        self.execute_control_command(ControlSubCommand::BatRemove);
    }

    pub fn enable_hibernate(&mut self, en: bool) {
        if en {
            self.execute_control_command(ControlSubCommand::SetHibernate)
        } else {
            self.execute_control_command(ControlSubCommand::ClearHibernate)
        }
    }

    pub fn enable_snooze(&mut self, en: bool) {
        if en {
            self.execute_control_command(ControlSubCommand::SetSnooze)
        } else {
            self.execute_control_command(ControlSubCommand::ClearSnooze)
        }
    }

    pub fn get_df_version(&mut self) -> u16 {
        self.read_control_command(ControlSubCommand::DfVersion)
    }

    pub fn get_at_rate(&mut self) -> MilliAmps {
        MilliAmps(self.read_command(StandardCommand::AtRate) as i16 as i32)
    }

    pub fn set_at_rate(&mut self, ma: MilliAmps) {
        self.write_command(StandardCommand::AtRate, ma.0 as u16)
    }

    pub fn get_at_rate_time_to_empty(&mut self) -> Minutes {
        Minutes(self.read_command(StandardCommand::AtRateTimeToEmpty) as u32)
    }

    pub fn get_temperature(&mut self) -> Celcius {
        // TODO Verify
        Celcius(self.read_command(StandardCommand::Temperature) as f32 * 10.0 - 273.15)
    }

    pub fn set_temperature(&mut self, t: Celcius) {
        // TODO Verify
        self.write_command(StandardCommand::Temperature, ((t.0 + 273.15) / 10.0) as u16)
    }

    pub fn get_voltage(&mut self) -> MilliVolts {
        MilliVolts(self.read_command(StandardCommand::Voltage) as i32)
    }

    pub fn get_flags(&mut self) -> Flags {
        Flags { data: self.read_command(StandardCommand::Flags) }
    }

    pub fn get_nominal_available_capacity(&mut self) -> MilliAmpHours {
        MilliAmpHours(self.read_command(StandardCommand::NominalAvailableCapacity) as i32)
    }

    pub fn get_full_available_capacity(&mut self) -> MilliAmpHours {
        MilliAmpHours(self.read_command(StandardCommand::FullAvailableCapacity) as i32)
    }

    pub fn get_remaining_capacity(&mut self) -> MilliAmpHours {
        MilliAmpHours(self.read_command(StandardCommand::RemainingCapacity) as i32)
    }

    pub fn get_full_charge_capacity(&mut self) -> MilliAmpHours {
        MilliAmpHours(self.read_command(StandardCommand::FullChargeCapacity) as i32)
    }

    pub fn get_average_current(&mut self) -> MilliAmps {
        MilliAmps(self.read_command(StandardCommand::AverageCurrent) as i16 as i32)
    }

    pub fn get_time_to_empty(&mut self) -> Minutes {
        Minutes(self.read_command(StandardCommand::TimeToEmpty) as u32)
    }

    pub fn get_standby_current(&mut self) -> MilliAmps {
        MilliAmps(self.read_command(StandardCommand::StandbyCurrent) as i16 as i32)
    }

    pub fn get_standby_time_to_empty(&mut self) -> Minutes {
        Minutes(self.read_command(StandardCommand::StandbyTimeToEmpty) as u32)
    }

    pub fn get_state_of_health(&mut self) -> StateOfHealth {
        // TODO Verify
        let data = self.read_command(StandardCommand::StateOfHealth);
        let percent = (data & 0xFF) as u8;
        let status = (data >> 8) as u8;
        match status {
            0x00 => StateOfHealth::Uninitialized,
            0x01 => StateOfHealth::Instant { percent },
            0x02 => StateOfHealth::Initial { percent },
            0x03 => StateOfHealth::Normal { percent },
            _ => StateOfHealth::Invalid
        }
    }

    pub fn get_cycle_count(&mut self) -> u16 {
        self.read_command(StandardCommand::CycleCount)
    }

    pub fn get_state_of_charge(&mut self) -> u16 {
        self.read_command(StandardCommand::StateOfCharge)
    }

    pub fn get_instantaneous_current(&mut self) -> MilliAmps {
        MilliAmps(self.read_command(StandardCommand::InstantaneousCurrent) as i16 as i32)
    }

    pub fn get_internal_temperature(&mut self) -> Celcius {
        // TODO Verify
        Celcius(self.read_command(StandardCommand::InternalTemperature) as f32 * 10.0 - 273.15)
    }

    pub fn get_resistance_scale(&mut self) -> u16 {
        self.read_command(StandardCommand::ResistanceScale)
    }

    pub fn get_operation_configuration(&mut self) -> OperationConfiguration {
        OperationConfiguration { data: self.read_command(StandardCommand::OperationConfiguration) }
    }

    pub fn get_design_capacity(&mut self) -> MilliAmpHours {
        MilliAmpHours(self.read_command(StandardCommand::DesignCapacity) as i32)
    }

    pub fn get_unfiltered_rm(&mut self) -> MilliAmpHours {
        MilliAmpHours(self.read_command(StandardCommand::UnfilteredRM) as i32)
    }

    pub fn get_filtered_rm(&mut self) -> MilliAmpHours {
        MilliAmpHours(self.read_command(StandardCommand::FilteredRM) as i32)
    }

    pub fn get_unfiltered_fcc(&mut self) -> MilliAmpHours {
        MilliAmpHours(self.read_command(StandardCommand::UnfilteredFCC) as i32)
    }

    pub fn get_filtered_fcc(&mut self) -> MilliAmpHours {
        MilliAmpHours(self.read_command(StandardCommand::FilteredFCC) as i32)
    }

    pub fn get_true_soc(&mut self) -> u16 {
        self.read_command(StandardCommand::TrueSOC)
    }

    pub fn get_application_status(&mut self) -> ApplicationStatus {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDRESS, &[ExtendedDataCommands::ApplicationStatus as u8], &mut buf);
        ApplicationStatus { data: u8::from_le_bytes(buf) }
    }

    fn read_command(&mut self, sc: StandardCommand) -> u16 {
        let mut buf = [0u8; 2];
        self.i2c.write_read(I2C_ADDRESS, &[sc as u8], &mut buf);
        delay(1 << 16);
        u16::from_le_bytes(buf)
    }

    fn write_command(&mut self, sc: StandardCommand, data: u16) {
        let bytes = data.to_le_bytes();
        self.i2c.write(I2C_ADDRESS, &[sc as u8, bytes[0], bytes[1]]);
        delay(1 << 16);
    }

    fn execute_control_command(&mut self, csc: ControlSubCommand) {
        let cmd = (csc as u16).to_le_bytes();
        self.i2c.write(I2C_ADDRESS, &[StandardCommand::Control as u8, cmd[0], cmd[1]]);
        delay(1 << 16);
    }

    fn read_control_command(&mut self, csc: ControlSubCommand) -> u16 {
        let mut buf = [0u8; 2];
        self.execute_control_command(csc);
        self.i2c.write_read(I2C_ADDRESS, &[StandardCommand::Control as u8], &mut buf);
        delay(1 << 16);
        u16::from_le_bytes(buf)
    }
}

impl<I> BatteryGauge<Sealed, I> where I: Read + Write + WriteRead {
    pub fn new(i2c: I) -> Option<BatteryGauge<Sealed, I>> {
        let mut bg = BatteryGauge { i2c, _state: PhantomData };
        if bg.get_device_type() == DEVICE_TYPE && bg.get_firmware_version() >= MIN_FIRMWARE_VERSION {
            Some(bg)
        } else {
            None
        }
    }

    pub fn unseal_device(mut self, unseal_key: u32) -> Result<BatteryGauge<Unsealed, I>, BatteryGauge<Sealed, I>> {
        if !self.get_control_status().is_sealed() {
            return Ok(BatteryGauge { i2c: self.i2c, _state: PhantomData });
        }
        let uc = unseal_key.to_le_bytes();
        self.i2c.write(I2C_ADDRESS, &[StandardCommand::Control as u8, uc[2], uc[3]]);
        delay(1 << 16);
        self.i2c.write(I2C_ADDRESS, &[StandardCommand::Control as u8, uc[0], uc[1]]);
        delay(1 << 16);
        if self.get_control_status().is_sealed() {
            Err(self)
        } else {
            Ok(BatteryGauge { i2c: self.i2c, _state: PhantomData })
        }
    }

    pub fn unseal_full_access(self, unseal_key: u32, full_access_key: u32) -> Result<BatteryGauge<Unsealed, I>, BatteryGauge<Sealed, I>> {
        match self.unseal_device(unseal_key) {
            Ok(mut bg) => {
                if !bg.get_control_status().is_full_access_sealed() {
                    return Ok(bg);
                }
                let fac = full_access_key.to_le_bytes();
                bg.i2c.write(I2C_ADDRESS, &[StandardCommand::Control as u8, fac[2], fac[3]]);
                delay(1 << 16);
                bg.i2c.write(I2C_ADDRESS, &[StandardCommand::Control as u8, fac[0], fac[1]]);
                delay(1 << 16);
                if bg.get_control_status().is_full_access_sealed() {
                    Err(BatteryGauge { i2c: bg.i2c, _state: PhantomData })
                } else {
                    Ok(bg)
                }
            }
            Err(bg) => Err(bg)
        }
    }
}

impl<I> BatteryGauge<Unsealed, I> where I: Read + Write + WriteRead {
    pub fn seal_device(mut self) -> BatteryGauge<Sealed, I> {
        self.execute_control_command(ControlSubCommand::Sealed);
        BatteryGauge { i2c: self.i2c, _state: PhantomData }
    }

    pub fn reset(mut self) -> BatteryGauge<Sealed, I> {
        self.execute_control_command(ControlSubCommand::Reset);
        BatteryGauge { i2c: self.i2c, _state: PhantomData }
    }

    pub fn do_impedance_track_algorithm(&mut self) {
        self.execute_control_command(ControlSubCommand::ItEnabled)
    }

    pub fn get_unseal_key(&mut self) -> u32 {
        let mut buf = [0u8; 4];
        self.read_data_flash(SubClass::Codes, 0, &mut buf);
        u32::from_le_bytes([buf[1], buf[0], buf[3], buf[2]])
    }

    pub fn get_full_access_key(&mut self) -> u32 {
        let mut buf = [0u8; 4];
        self.read_data_flash(SubClass::Codes, 4, &mut buf);
        u32::from_le_bytes([buf[1], buf[0], buf[3], buf[2]])
    }

    pub fn get_charging_voltage(&mut self) -> MilliVolts {
        let mut buf = [0u8; 2];
        self.read_data_flash(SubClass::Charge, 2, &mut buf);
        MilliVolts(u16::from_be_bytes(buf) as i16 as i32)
    }

    pub fn set_charging_voltage(&mut self, v: MilliVolts) {
        self.write_data_flash(SubClass::Charge, 2, &(v.0 as i16).to_be_bytes());
    }

    pub fn get_cycle_count_threshold(&mut self) -> MilliAmpHours {
        let mut buf = [0u8; 2];
        self.read_data_flash(SubClass::ConfigurationData, 7, &mut buf);
        MilliAmpHours(u16::from_be_bytes(buf) as i16 as i32)
    }

    pub fn set_cycle_count_threshold(&mut self, c: MilliAmpHours) {
        self.write_data_flash(SubClass::ConfigurationData, 7, &(c.0 as i16).to_be_bytes());
    }

    pub fn get_design_capacity_flash(&mut self) -> MilliAmpHours {
        let mut buf = [0u8; 2];
        self.read_data_flash(SubClass::ConfigurationData, 10, &mut buf);
        MilliAmpHours(u16::from_be_bytes(buf) as i16 as i32)
    }

    pub fn set_design_capacity(&mut self, c: MilliAmpHours) {
        self.write_data_flash(SubClass::ConfigurationData, 10, &(c.0 as i16).to_be_bytes());
    }

    pub fn get_design_energy_scale(&mut self) -> u8 {
        let mut buf = [0u8; 1];
        self.read_data_flash(SubClass::ConfigurationData, 12, &mut buf);
        u8::from_be_bytes(buf)
    }

    pub fn set_design_energy_scale(&mut self, scale: u8) -> Result<(), Error> {
        if scale != 1 && scale != 10 {
            return Err(Error::InvalidArgumentError);
        }
        self.write_data_flash(SubClass::ConfigurationData, 12, &[scale]);
        Ok(())
    }

    pub fn get_system_shutdown_voltage_threshold(&mut self) -> MilliVolts {
        let mut buf = [0u8; 2];
        self.read_data_flash(SubClass::Discharge, 5, &mut buf);
        MilliVolts(u16::from_be_bytes(buf) as i16 as i32)
    }

    pub fn set_system_shutdown_voltage_threshold(&mut self, v: MilliVolts) {
        self.write_data_flash(SubClass::Discharge, 5, &(v.0 as i16).to_be_bytes());
        self.write_data_flash(SubClass::Discharge, 8, &(v.0 as i16 + 200).to_be_bytes())
    }

    pub fn set_operation_configuration(&mut self, oc: OperationConfiguration) {
        self.write_data_flash(SubClass::Registers, 0, &oc.data.to_be_bytes());
    }

    fn configure_data_flash(&mut self, sc: SubClass, offset: u8) {
        self.i2c.write(I2C_ADDRESS, &[ExtendedDataCommands::BlockDataControl as u8, 0]);
        delay(1 << 16);
        self.i2c.write(I2C_ADDRESS, &[ExtendedDataCommands::DataFlashClass as u8, sc as u8]);
        delay(1 << 16);
        self.i2c.write(I2C_ADDRESS, &[ExtendedDataCommands::DataFlashBlock as u8, offset / 32]);
        delay(1 << 16);
    }

    fn read_data_chksum(&mut self) -> u8 {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDRESS, &[ExtendedDataCommands::BlockDataCheckSum as u8], &mut buf);
        u8::from_le_bytes(buf)
    }

    fn read_data_flash(&mut self, sc: SubClass, offset: u8, buf: &mut [u8]) {
        self.configure_data_flash(sc, offset);
        let block_offset = (ExtendedDataCommands::BlockData as u8) + offset % 32;
        self.i2c.write_read(I2C_ADDRESS, &[block_offset], buf);
        delay(1 << 16);
    }

    fn write_data_flash(&mut self, sc: SubClass, offset: u8, data: &[u8]) {
        let block_offset = offset % 32;

        let mut block_data = [0u8; 33];
        self.configure_data_flash(sc, offset);
        self.i2c.write_read(I2C_ADDRESS, &[ExtendedDataCommands::BlockData as u8], &mut block_data[0..31]);
        delay(1 << 16);

        for (i, &d) in data.iter().enumerate() {
            block_data[block_offset as usize + i] = d
        }

        let chksum = Wrapping(255u8) - block_data.iter()
            .map(|&x| Wrapping(x)).sum::<Wrapping<u8>>();

        block_data.rotate_right(1);
        block_data[0] = ExtendedDataCommands::BlockData as u8;

        self.configure_data_flash(sc, offset);
        self.i2c.write(I2C_ADDRESS, &mut block_data);
        delay(1 << 16);

        self.i2c.write(I2C_ADDRESS, &[ExtendedDataCommands::BlockDataCheckSum as u8, chksum.0]);
        delay(1 << 16);
    }
}
