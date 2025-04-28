#include "RS485_Region_handler.h"

#include "FreeRTOS.h"
#include "task.h"
#include "RS485.h"

#ifdef CDU_RS485
#include "PID.h"
#include "main_task.h"
#include "power_task.h"
#include "pt100_task.h"
#include "pump_task.h"
#include "sensor_task.h"
#include "side_card_task.h"
#elif SENS_RS485
#include "pressure_task.h"
#include "pt100_task.h"
#elif FAN_RS485
#include "FG_task.h"
#include "PWM_task.h"
#include "fan_main_task.h"
#endif

#define CASE_RW(member) return ((func) == READ_HOLDING_REGISTERS ? member : (member = (data))) & 0xFFFF

#ifdef CDU_RS485
void RsRegHdle_Init() {
  RsRegHdle(SysBasicInform_Handler, SYS_BASIC_INFORM_REG_START, SYS_BASIC_INFORM_REG_END);
  RsRegHdle(MainSysParaSet_Handler, MAIN_SYS_PARA_SET_REG_START, MAIN_SYS_PARA_SET_REG_END);
  RsRegHdle(MainSysAlarmParaSet_Handler, MAIN_SYS_ALARM_SET_REG_START, MAIN_SYS_ALARM_SET_REG_END);
  RsRegHdle(MainSysParaDisp_Handler, MAIN_SYS_STAT_DISP_REG_START, MAIN_SYS_STAT_DISP_REG_END);
  RsRegHdle(MainDataRead_Handler, MAIN_DATA_READ_REG_START, MAIN_DATA_READ_REG_END);
  RsRegHdle(MainDataCalib_Handler, MAIN_DATA_CALIB_REG_START, MAIN_DATA_CALIB_REG_END);
  RsRegHdle(MainDevCtrl_Handler, MAIN_DEV_CTRL_REG_START, MAIN_DEV_CTRL_REG_END);
  RsRegHdle(CDU_PowerRead_Handler, CDU_POWER_READ_REG_START, CDU_POWER_READ_REG_END);
  RsRegHdle(SensDataRead_Handler, SENS_DATA_READ_REG_START, SENS_DATA_READ_REG_END);
  RsRegHdle(SensDataCalib_Handler, SENS_DATA_CALIB_REG_START, SENS_DATA_CALIB_REG_END);
  RsRegHdle(SensDevCtrl_Handler, SENS_DEV_CTRL_REG_START, SENS_DEV_CTRL_REG_END);
  RsRegHdle(SideCarPowerRead_Handler, SIDECAR_POWER_READ_REG_START, SIDECAR_POWER_READ_REG_END);
  RsRegHdle(FanSysSet_Handler, FANS_SYS_SET_REG_START, FANS_SYS_SET_REG_END);
  RsRegHdle(FanSysDisp_Handler, FANS_SYS_DISP_REG_START, FANS_SYS_DISP_REG_END);
  RsRegHdle(Fans_FG_PWM_Set_Handler, FANS_FG_PWM_SET_REG_START, FANS_FG_PWM_SET_REG_END);
}
#elif SENS_RS485
void RsRegHdle_Init() {}
#elif FAN_RS485
void RsRegHdle_Init() {}
#endif

#if defined(CDU_RS485)
uint32_t SysBasicInform_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS || (func) == WRITE_SINGLE_REGISTER || (func) == WRITE_MULTIPLE_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case POWER_ON_SETTING:
      CASE_RW(SysInform.power_on_setting);
    case KP:
      if (func == READ_HOLDING_REGISTERS) {
        return (uint16_t)(CUD_PID.kp * 100) & 0xFFFF;
      } else {
        CUD_PID.kp = ((float)data / 100.0);
        return data & 0xFFFF;
      }
    case KI:
      if (func == READ_HOLDING_REGISTERS) {
        return (uint16_t)(CUD_PID.ki * 100) & 0xFFFF;
      } else {
        CUD_PID.ki = ((float)data / 100.0);
        return data & 0xFFFF;
      }
    case KD:
      if (func == READ_HOLDING_REGISTERS) {
        return (uint16_t)(CUD_PID.kd * 100) & 0xFFFF;
      } else {
        CUD_PID.kd = ((float)data / 100.0);
        return data & 0xFFFF;
      }
    default:
      return 0;
  }
}

uint32_t MainSysParaSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS || (func) == WRITE_SINGLE_REGISTER || (func) == WRITE_MULTIPLE_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case AUTOMATIC_MODE:
      CASE_RW(SysParaSet.ctrl_mode);
    case TEMPERATURE_SET_POINT:
      CASE_RW(SysParaSet.temp_set);
    case FLOW_SET_POINT:
      CASE_RW(SysParaSet.flow_set);
    case DIFFERENTIAL_PRESSURE_SET_POINT:
      CASE_RW(SysParaSet.press_set);
    case MIN_PUMP_SPEED:
      CASE_RW(SysParaSet.pump_min_duty);
    case PUMP_STOP_DELAY:
      CASE_RW(SysParaSet.pump_stop_delay);
    case CDU_DEW_POINT_CALIBRATION:
      CASE_RW(SysParaSet.dew_temp_calib);
    case CDU_TEMP_COMPENSATION:
      CASE_RW(SysParaSet.ambient_temp_calib);
    case CDU_FLUID_INLET_TEMP_CALIBRATION:
      CASE_RW(SysParaSet.inlet_temp_calib);
    case CDU_FLUID_OUTLET_TEMP_CALIBRATION:
      CASE_RW(SysParaSet.outlet_temp_calib);
    case PT100_ABNORMAL_TEMPERATURE_LOW:
      CASE_RW(SysParaSet.pt100_abnl_temp_low_m);
    case PT100_ABNORMAL_TEMPERATURE_HIGH:
      CASE_RW(SysParaSet.pt100_abnl_temp_high_m);
    case NTC_ABNORMAL_TEMPERATURE_LOW:
      CASE_RW(SysParaSet.ntc_abnl_temp_low_m);
    case NTC_ABNORMAL_TEMPERATURE_HIGH:
      CASE_RW(SysParaSet.ntc_abnl_temp_high_m);
    case ABNORMAL_PRESSURE_LOW:
      CASE_RW(SysParaSet.press_abnl_val_low_kpa);
    case ABNORMAL_PRESSURE_HIGH:
      CASE_RW(SysParaSet.press_abnl_val_high_kpa);
    default:
      return 0;
  }
}

uint32_t MainSysAlarmParaSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS || (func) == WRITE_SINGLE_REGISTER || (func) == WRITE_MULTIPLE_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case PRESSURE_ALARM_MODE:
      CASE_RW(SysParaSet.press_warn.act);
    case PRESSURE_ALARM_DELAY:
      CASE_RW(SysParaSet.press_warn.delay);
    case OUTLET_FLUID_LOW_PRESSURE_ALARM:
      CASE_RW(SysParaSet.press_warn.outlet_low);
    case OUTLET_FLUID_HIGH_PRESSURE_ALARM:
      CASE_RW(SysParaSet.press_warn.outlet_high);
    case INLET_FLUID_LOW_PRESSURE_ALARM:
      CASE_RW(SysParaSet.press_warn.inlet_low);
    case INLET_FLUID_HIGH_PRESSURE_ALARM:
      CASE_RW(SysParaSet.press_warn.inlet_high);
    case SIDECAR_RETURN_FLUID_LOW_PRESSURE_ALARM:
      CASE_RW(SysParaSet.press_warn.return_low);
    case SIDECAR_RETURN_FLUID_HIGH_PRESSURE_ALARM:
      CASE_RW(SysParaSet.press_warn.return_high);
    case FLOW_ALARM_MODE:
      CASE_RW(SysParaSet.flow_warn.act);
    case LOW_FLOW_ALARM:
      CASE_RW(SysParaSet.flow_warn.low_val);
    case LOW_FLOW_DELAY_PERIOD:
      CASE_RW(SysParaSet.flow_warn.low_delay);
    case OVER_FLOW_ALARM:
      CASE_RW(SysParaSet.flow_warn.high_val);
    case OVER_FLOW_DELAY_PERIOD:
      CASE_RW(SysParaSet.flow_warn.high_delay);
    case TEMP_ALARM_MODE:
      CASE_RW(SysParaSet.temp_warn.act);
    case TEMP_ALARM_DELAY_PERIOD:
      CASE_RW(SysParaSet.temp_warn.delay);
    case LOW_OUTLET_FLUID_TEMP_ALARM:
      CASE_RW(SysParaSet.temp_warn.outlet_low);
    case HIGH_OUTLET_FLUID_TEMP_ALARM:
      CASE_RW(SysParaSet.temp_warn.outlet_high);
    case HIGH_AMBIENT_TEMP_ALARM:
      CASE_RW(SysParaSet.temp_warn.ambient_high);
    case LEAK_DETECTION_MODE:
      CASE_RW(SysParaSet.leak_warn.act);
    case CDU_DETECTING_DELAY_PERIOD:
      CASE_RW(SysParaSet.leak_warn.CUD_delay);
    case SERVER_DETECTING_DELAY_PERIOD:
      CASE_RW(SysParaSet.leak_warn.server_delay);
    case SIDECAR_DETECTING_DELAY_PERIOD:
      CASE_RW(SysParaSet.leak_warn.sidecar_delay);
    default:
      return 0;
  }
}

uint32_t MainSysParaDisp_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case SYSTEM_ALARM_STATUS_1:
      return SysParaDisp.warn_stat & 0xFFFF;
    case SYSTEM_UPTIME_HIGH:
      return SysParaDisp.sys_startup_time_high & 0xFFFF;
    case SYSTEM_UPTIME_LOW:
      return SysParaDisp.sys_startup_time_low & 0xFFFF;
    case PUMP1_RUNTIME_HIGH:
      return SysParaDisp.pump_1_startup_time_high & 0xFFFF;
    case PUMP1_RUNTIME_LOW:
      return SysParaDisp.pump_1_startup_time_low & 0xFFFF;
    case PUMP2_RUNTIME_HIGH:
      return SysParaDisp.pump_2_startup_time_high & 0xFFFF;
    case PUMP2_RUNTIME_LOW:
      return SysParaDisp.pump_2_startup_time_low & 0xFFFF;
    case PERIPHERAL_STATUS_1:
      return SysParaDisp.periphery_component_exist & 0xFFFF;
    default:
      return 0;
  }
}

uint32_t MainDataRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  bool isRead = (func == READ_HOLDING_REGISTERS);
  bool isWrite =
      (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) && root && (addr >= AMBIRNT_TEMPERATURE && addr <= AMBIRNT_HUMIDITY);

  if (!isRead && !isWrite) {
    return ILLIGAL_FUNC << 16;
  }

  switch (addr) {
    case PT100_1_TEMPERATURE:
      return Pt100Stat.pt100_temp_x10[0] & 0xFFFF;
    case PT100_2_TEMPERATURE:
      return Pt100Stat.pt100_temp_x10[1] & 0xFFFF;
    case PT100_3_TEMPERATURE:
      return Pt100Stat.pt100_temp_x10[2] & 0xFFFF;
    case PT100_4_TEMPERATURE:
      return Pt100Stat.pt100_temp_x10[3] & 0xFFFF;
    case NTC_1_TEMPERATURE:
      return (SensStat.ntc_temp_x10[0]) & 0xFFFF;
    case NTC_2_TEMPERATURE:
      return (SensStat.ntc_temp_x10[1]) & 0xFFFF;
    case NTC_3_TEMPERATURE:
      return (SensStat.ntc_temp_x10[2]) & 0xFFFF;
    case NTC_4_TEMPERATURE:
      return (SensStat.ntc_temp_x10[3]) & 0xFFFF;
    case PRESSURE_1_VALUE:
      return (SensStat.press_val_kpa[0]) & 0xFFFF;
    case PRESSURE_2_VALUE:
      return (SensStat.press_val_kpa[1]) & 0xFFFF;
    case PRESSURE_3_VALUE:
      return (SensStat.press_val_kpa[2]) & 0xFFFF;
    case PRESSURE_4_VALUE:
      return (SensStat.press_val_kpa[3]) & 0xFFFF;
    case FLOW_VALUE:
      return (SensStat.Flow_val) & 0xFFFF;
    case PUMP_1_FEEDBACK:
      return pump_status.pump_1_FB & 0xFFFF;
    case PUMP_2_FEEDBACK:
      return pump_status.pump_2_FB & 0xFFFF;
    case LEAK_SENSOR:
      return SensStat.leak_sensor & 0xFFFF;
    case AMBIRNT_TEMPERATURE:
      CASE_RW(SensStat.temperature);
    case AMBIRNT_HUMIDITY:
      CASE_RW(SensStat.humidity);
    case DEW_TEMPERATURE:
      CASE_RW(SensStat.dew_temp);
    case FAN_1_FEEDBACK_TEMP:
      return FansCardStat.fan_fg[0] & 0xFFFF;
    case DEVICE_CONNECTED:
      return 0;
    default:
      return 0;
  }
}

uint32_t MainDataCalib_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS || (func) == WRITE_SINGLE_REGISTER || (func) == WRITE_MULTIPLE_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case PT100_IDEAL_LOW:
      CASE_RW(Pt100TwoCal.pt100_ideal_l_val);
    case PT100_IDEAL_HIGH:
      CASE_RW(Pt100TwoCal.pt100_ideal_h_val);
    case PT100_1_RAW_LOW:
      CASE_RW(Pt100TwoCal.pt100_raw_l_val[0]);
    case PT100_2_RAW_LOW:
      CASE_RW(Pt100TwoCal.pt100_raw_l_val[1]);
    case PT100_3_RAW_LOW:
      CASE_RW(Pt100TwoCal.pt100_raw_l_val[2]);
    case PT100_4_RAW_LOW:
      CASE_RW(Pt100TwoCal.pt100_raw_l_val[3]);
    case PT100_1_RAW_HIGH:
      CASE_RW(Pt100TwoCal.pt100_raw_h_val[0]);
    case PT100_2_RAW_HIGH:
      CASE_RW(Pt100TwoCal.pt100_raw_h_val[1]);
    case PT100_3_RAW_HIGH:
      CASE_RW(Pt100TwoCal.pt100_raw_h_val[2]);
    case PT100_4_RAW_HIGH:
      CASE_RW(Pt100TwoCal.pt100_raw_h_val[3]);
    case NTC_IDEAL_LOW:
      CASE_RW(NtcTwoCal.ntc_ideal_l_val);
    case NTC_IDEAL_HIGH:
      CASE_RW(NtcTwoCal.ntc_ideal_h_val);
    case NTC_1_RAW_LOW:
      CASE_RW(NtcTwoCal.ntc_raw_l_val[0]);
    case NTC_2_RAW_LOW:
      CASE_RW(NtcTwoCal.ntc_raw_l_val[1]);
    case NTC_3_RAW_LOW:
      CASE_RW(NtcTwoCal.ntc_raw_l_val[2]);
    case NTC_4_RAW_LOW:
      CASE_RW(NtcTwoCal.ntc_raw_l_val[3]);
    case PRESSURE_IDEAL_LOW:
      CASE_RW(PressTwoCal.press_ideal_l_val);
    case PRESSURE_IDEAL_HIGH:
      CASE_RW(PressTwoCal.press_ideal_h_val);
    case PRESSURE_1_RAW_LOW:
      CASE_RW(PressTwoCal.press_raw_l_val[0]);
    case PRESSURE_2_RAW_LOW:
      CASE_RW(PressTwoCal.press_raw_l_val[1]);
    case PRESSURE_3_RAW_LOW:
      CASE_RW(PressTwoCal.press_raw_l_val[2]);
    case PRESSURE_4_RAW_LOW:
      CASE_RW(PressTwoCal.press_raw_l_val[3]);
    case PRESSURE_1_RAW_HIGH:
      CASE_RW(PressTwoCal.press_raw_h_val[0]);
    case PRESSURE_2_RAW_HIGH:
      CASE_RW(PressTwoCal.press_raw_h_val[1]);
    case PRESSURE_3_RAW_HIGH:
      CASE_RW(PressTwoCal.press_raw_h_val[2]);
    case PRESSURE_4_RAW_HIGH:
      CASE_RW(PressTwoCal.press_raw_h_val[3]);
    default:
      return 0;
  }
}

uint32_t CDU_PowerRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case POWER_1_STATUS:
      return (Power_Stat[0].power_status) & 0xFFFF;
    case POWER_1_VOLTAGE_INPUT:
      return (Power_Stat[0].voltage_input) & 0xFFFF;
    case POWER_1_CURRENT_INPUT:
      return (Power_Stat[0].current_input) & 0xFFFF;
    case POWER_1_TEMPERATURE:
      return (Power_Stat[0].power_temp) & 0xFFFF;
    case POWER_2_STATUS:
      return (Power_Stat[1].power_status) & 0xFFFF;
    case POWER_2_VOLTAGE_INPUT:
      return (Power_Stat[1].voltage_input) & 0xFFFF;
    case POWER_2_CURRENT_INPUT:
      return (Power_Stat[1].current_input) & 0xFFFF;
    case POWER_2_TEMPERATURE:
      return (Power_Stat[1].power_temp) & 0xFFFF;
    default:
      return 0;
  }
}

uint32_t MainDevCtrl_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS || (func) == WRITE_SINGLE_REGISTER || (func) == WRITE_MULTIPLE_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case PUMP_1_RPM:
      CASE_RW(pump_control.pump_1_rpm);
    case PUMP_2_RPM:
      CASE_RW(pump_control.pump_2_rpm);
    case PROPORTIONAL_VALVE_1_DUTY:
      CASE_RW(SensCtrl.porpo_1_duty);
    case PROPORTIONAL_VALVE_2_DUTY:
      CASE_RW(SensCtrl.porpo_2_duty);
    case PT100_CHANNEL_ENABLE_SETTING:
      CASE_RW(Pt100Stat.pt100_enable);
    default:
      return 0;
  }
}
#endif

#if defined(SENS_RS485) || defined(CDU_RS485)
uint32_t SensDataRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  bool isRead = (func == READ_HOLDING_REGISTERS);
  bool isWrite = (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) && root;

  if (!isRead && !isWrite) {
    return ILLIGAL_FUNC << 16;
  }

  switch (addr) {
    case SIDECAR_PT100_1_TEMPERATURE:
      CASE_RW(SensCardStat.pt100_temp_x10[0]);
    case SIDECAR_PT100_2_TEMPERATURE:
      CASE_RW(SensCardStat.pt100_temp_x10[1]);
    case SIDECAR_PT100_3_TEMPERATURE:
      CASE_RW(SensCardStat.pt100_temp_x10[2]);
    case SIDECAR_PT100_4_TEMPERATURE:
      CASE_RW(SensCardStat.pt100_temp_x10[3]);
    case SIDECAR_PRESSURE_1_VALUE:
      CASE_RW(SensCardStat.press_val_kpa[0]);
    case SIDECAR_PRESSURE_2_VALUE:
      CASE_RW(SensCardStat.press_val_kpa[1]);
    case SIDECAR_LEAK_SENSOR:
      CASE_RW(SensCardStat.leak_sensor);
    case SIDECAR_TEMPERATURE:
      CASE_RW(SensCardStat.temperature);
    case SIDECAR_HUMIDITY:
      CASE_RW(SensCardStat.humidity);
    default:
      return 0;
  }
}

uint32_t SensDataCalib_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS || (func) == WRITE_SINGLE_REGISTER || (func) == WRITE_MULTIPLE_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case SIDECAR_PT100_IDEAL_LOW:
      CASE_RW(SensPt100TwoCal.pt100_ideal_l_val);
    case SIDECAR_PT100_IDEAL_HIGH:
      CASE_RW(SensPt100TwoCal.pt100_ideal_h_val);
    case SIDECAR_PT100_1_RAW_LOW:
      CASE_RW(SensPt100TwoCal.pt100_raw_l_val[0]);
    case SIDECAR_PT100_2_RAW_LOW:
      CASE_RW(SensPt100TwoCal.pt100_raw_l_val[1]);
    case SIDECAR_PT100_3_RAW_LOW:
      CASE_RW(SensPt100TwoCal.pt100_raw_l_val[2]);
    case SIDECAR_PT100_4_RAW_LOW:
      CASE_RW(SensPt100TwoCal.pt100_raw_l_val[3]);
    case SIDECAR_PT100_1_RAW_HIGH:
      CASE_RW(SensPt100TwoCal.pt100_raw_h_val[0]);
    case SIDECAR_PT100_2_RAW_HIGH:
      CASE_RW(SensPt100TwoCal.pt100_raw_h_val[1]);
    case SIDECAR_PT100_3_RAW_HIGH:
      CASE_RW(SensPt100TwoCal.pt100_raw_h_val[2]);
    case SIDECAR_PT100_4_RAW_HIGH:
      CASE_RW(SensPt100TwoCal.pt100_raw_h_val[3]);
    case SIDECAR_PRESSURE_IDEAL_LOW:
      CASE_RW(SensPressTwoCal.press_ideal_l_val);
    case SIDECAR_PRESSURE_IDEAL_HIGH:
      CASE_RW(SensPressTwoCal.press_ideal_h_val);
    case SIDECAR_PRESSURE_1_RAW_LOW:
      CASE_RW(SensPressTwoCal.press_raw_l_val[0]);
    case SIDECAR_PRESSURE_2_RAW_LOW:
      CASE_RW(SensPressTwoCal.press_raw_l_val[1]);
    case SIDECAR_PRESSURE_1_RAW_HIGH:
      CASE_RW(SensPressTwoCal.press_raw_h_val[0]);
    case SIDECAR_PRESSURE_2_RAW_HIGH:
      CASE_RW(SensPressTwoCal.press_raw_h_val[1]);
    default:
      return 0;
  }
}

uint32_t SensDevCtrl_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS || (func) == WRITE_SINGLE_REGISTER || (func) == WRITE_MULTIPLE_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case SIDECAR_PRESSURE_PUMP:
      CASE_RW(SensCardCtrl.pressure_pump);
    case SIDECAR_PT100_CHANNEL_ENABLE_SETTING:
      CASE_RW(SensCardCtrl.pt100_enable);
    default:
      return 0;
  }
}

uint32_t SideCarPowerRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case SIDECAR_POWER_1_STATUS:
      return (SideCarPower_Stat[0].power_status) & 0xFFFF;
    case SIDECAR_POWER_1_VOLTAGE:
      return (SideCarPower_Stat[0].voltage_output) & 0xFFFF;
    case SIDECAR_POWER_1_CURRENT:
      return (SideCarPower_Stat[0].current_output) & 0xFFFF;
    case SIDECAR_POWER_1_TEMPERATURE:
      return (SideCarPower_Stat[0].temperature) & 0xFFFF;
    case SIDECAR_POWER_2_STATUS:
      return (SideCarPower_Stat[1].power_status) & 0xFFFF;
    case SIDECAR_POWER_2_VOLTAGE:
      return (SideCarPower_Stat[1].voltage_output) & 0xFFFF;
    case SIDECAR_POWER_2_CURRENT:
      return (SideCarPower_Stat[1].current_output) & 0xFFFF;
    case SIDECAR_POWER_2_TEMPERATURE:
      return (SideCarPower_Stat[1].temperature) & 0xFFFF;
    case SIDECAR_POWER_3_STATUS:
      return (SideCarPower_Stat[2].power_status) & 0xFFFF;
    case SIDECAR_POWER_3_VOLTAGE:
      return (SideCarPower_Stat[2].voltage_output) & 0xFFFF;
    case SIDECAR_POWER_3_CURRENT:
      return (SideCarPower_Stat[2].current_output) & 0xFFFF;
    case SIDECAR_POWER_3_TEMPERATURE:
      return (SideCarPower_Stat[2].temperature) & 0xFFFF;
    case SIDECAR_POWER_4_STATUS:
      return (SideCarPower_Stat[3].power_status) & 0xFFFF;
    case SIDECAR_POWER_4_VOLTAGE:
      return (SideCarPower_Stat[3].voltage_output) & 0xFFFF;
    case SIDECAR_POWER_4_CURRENT:
      return (SideCarPower_Stat[3].current_output) & 0xFFFF;
    case SIDECAR_POWER_4_TEMPERATURE:
      return (SideCarPower_Stat[3].temperature) & 0xFFFF;
    default:
      return 0;
  }
}
#endif

#if defined(FAN_RS485) || defined(CDU_RS485)

uint32_t FanSysSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (!((func) == READ_HOLDING_REGISTERS || (func) == WRITE_SINGLE_REGISTER || (func) == WRITE_MULTIPLE_REGISTERS)) return ILLIGAL_FUNC << 16;

  switch (addr) {
    case FAN_ALARM_MODE:
      CASE_RW(FanCardSysSet.fan_alarm.act);
    case FAN_ALARM_DELAY:
      CASE_RW(FanCardSysSet.fan_alarm.delay);
    case FAN_BOARD_AUTO_CONTROL:
      CASE_RW(FanCardSysSet.auto_control);
    case AUTO_CONTROL_TARGET_SPEED:
      CASE_RW(FanCardSysSet.auto_control_target_speed);
    case FAN_INSTALLATION_STATUS:
      CASE_RW(FanCardSysSet.fan_installation_status);
    case FAN_LOW_SPEED_WARNING_THRESHOLD:
      CASE_RW(FanCardSysSet.fan_alarm.fan_low_speed_warning_threshold);
    case FAN_FG_DIFFERENCE_WARNING_THRESHOLD:
      CASE_RW(FanCardSysSet.fan_alarm.fan_fg_difference_warning_threshold);
    case FAN_SPEED_SAMPLING_INTERVAL_MS:
      CASE_RW(FanCardSysSet.fan_speed_sampling_interval_ms);
    case WEIGHTED_MOVING_AVERAGE_COUNT:
      if (func == READ_HOLDING_REGISTERS) {
        return FanCardSysSet.weighted_moving_average_count & 0xFFFF;
      } else {
        if (((data & (data - 1)) == 0) && (data <= FG_SAMPLE_COUNT_MAX)) {
          return (FanCardSysSet.weighted_moving_average_count = data) & 0xFFFF;
        } else {
          return 0 << 16;
        }
      }
    default:
      return 0;
  }
}

uint32_t FanSysDisp_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  bool isRead = (func == READ_HOLDING_REGISTERS);
  bool isWrite = (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) && root;

  if (!isRead && !isWrite) {
    return ILLIGAL_FUNC << 16;
  }

  switch (addr) {
    case FAN_FAULT_STATUS:
      CASE_RW(FanCardSysDisp.fan_fault_status);
    case FAN_STATUS_ON_FAN_BOARD_BITFIELD_0_15:
      CASE_RW(FanCardSysDisp.fan_status_on_fan_board_bitfield_0_15);
    case FAN_COUNT:
      CASE_RW(FanCardSysDisp.fan_count);
    default:
      return 0;
  }
}

uint32_t Fans_FG_PWM_Set_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  bool isRead = (func == READ_HOLDING_REGISTERS);
  bool isRootWrite = (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) && root;
  bool isNoRootWrite = ((func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) && !root) && (addr >= FAN_1_DUTY && addr <= FAN_16_DUTY);

  if (!isRead && !isRootWrite && !isRootWrite) {
    return ILLIGAL_FUNC << 16;
  }

  if (isNoRootWrite) {
    if (data >= 0 && data <= 1000) {
      return ILLIGAL_DATA_VALUE << 16;
    }
  }

  switch (addr) {
    case FAN_1_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[0]);
    case FAN_2_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[1]);
    case FAN_3_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[2]);
    case FAN_4_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[3]);
    case FAN_5_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[4]);
    case FAN_6_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[5]);
    case FAN_7_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[6]);
    case FAN_8_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[7]);
    case FAN_9_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[8]);
    case FAN_10_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[9]);
    case FAN_11_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[10]);
    case FAN_12_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[11]);
    case FAN_13_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[12]);
    case FAN_14_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[13]);
    case FAN_15_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[14]);
    case FAN_16_FEEDBACK:
      CASE_RW(FansCardStat.fan_fg[15]);
    case FAN_1_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[0]);
    case FAN_2_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[1]);
    case FAN_3_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[2]);
    case FAN_4_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[3]);
    case FAN_5_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[4]);
    case FAN_6_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[5]);
    case FAN_7_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[6]);
    case FAN_8_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[7]);
    case FAN_9_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[8]);
    case FAN_10_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[9]);
    case FAN_11_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[10]);
    case FAN_12_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[11]);
    case FAN_13_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[12]);
    case FAN_14_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[13]);
    case FAN_15_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[14]);
    case FAN_16_DUTY:
      CASE_RW(FansCardCtrl.fan_pwm[15]);
    default:
      return 0;
  }
}
#endif