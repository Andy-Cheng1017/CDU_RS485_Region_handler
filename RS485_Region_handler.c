#include "RS485_Region_handler.h"

#include "FreeRTOS.h"
#include "task.h"

#ifdef CDU_RS485
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

#if defined(CDU_RS485)
uint32_t SysInform_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case POWER_ON_SETTING:
        return SysInform.power_on_setting & 0xFFFF;
      default:
        return 0;
    }
  } else if (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) {
    switch (addr) {
      case POWER_ON_SETTING:
        return (SysInform.power_on_setting = data) & 0xFFFF;
      default:
        return 0;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}

uint32_t SysParaSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case AUTOMATIC_MODE:
        return SysParaSet.ctrl_mode & 0xFFFF;
      case TEMPERATURE_SET_POINT:
        return SysParaSet.temp_set / 100 & 0xFFFF;
      case FLOW_SET_POINT:
        return SysParaSet.flow_set & 0xFFFF;
      case DIFFERENTIAL_PRESSURE_SET_POINT:
        return SysParaSet.press_set & 0xFFFF;
      case MIN_PUMP_SPEED:
        return SysParaSet.pump_min_duty & 0xFFFF;
      case PUMP_STOP_DELAY:
        return SysParaSet.pump_stop_delay & 0xFFFF;
      case PRESSURE_ALARM_MODE:
        return SysParaSet.press_warn.act & 0xFFFF;
      case PRESSURE_ALARM_DELAY:
        return SysParaSet.press_warn.delay & 0xFFFF;
      case OUTLET_FLUID_LOW_PRESSURE_ALARM:
        return SysParaSet.press_warn.outlet_low & 0xFFFF;
      case OUTLET_FLUID_HIGH_PRESSURE_ALARM:
        return SysParaSet.press_warn.outlet_high & 0xFFFF;
      case INLET_FLUID_LOW_PRESSURE_ALARM:
        return SysParaSet.press_warn.inlet_low & 0xFFFF;
      case INLET_FLUID_HIGH_PRESSURE_ALARM:
        return SysParaSet.press_warn.inlet_high & 0xFFFF;
      case SIDECAR_RETURN_FLUID_LOW_PRESSURE_ALARM:
        return SysParaSet.press_warn.return_low & 0xFFFF;
      case SIDECAR_RETURN_FLUID_HIGH_PRESSURE_ALARM:
        return SysParaSet.press_warn.return_high & 0xFFFF;
      case FLOW_ALARM_MODE:
        return SysParaSet.flow_warn.act & 0xFFFF;
      case LOW_FLOW_ALARM:
        return SysParaSet.flow_warn.low_val & 0xFFFF;
      case LOW_FLOW_DELAY_PERIOD:
        return SysParaSet.flow_warn.low_delay & 0xFFFF;
      case OVER_FLOW_ALARM:
        return SysParaSet.flow_warn.high_val & 0xFFFF;
      case OVER_FLOW_DELAY_PERIOD:
        return SysParaSet.flow_warn.high_delay & 0xFFFF;
      case TEMP_ALARM_MODE:
        return SysParaSet.temp_warn.act & 0xFFFF;
      case TEMP_ALARM_DELAY_PERIOD:
        return SysParaSet.temp_warn.delay & 0xFFFF;
      case LOW_OUTLET_FLUID_TEMP_ALARM:
        return SysParaSet.temp_warn.outlet_low & 0xFFFF;
      case HIGH_OUTLET_FLUID_TEMP_ALARM:
        return SysParaSet.temp_warn.outlet_high & 0xFFFF;
      case HIGH_AMBIENT_TEMP_ALARM:
        return SysParaSet.temp_warn.ambient_high & 0xFFFF;
      case LEAK_DETECTION_MODE:
        return SysParaSet.leak_warn.act & 0xFFFF;
      case CDU_DETECTING_DELAY_PERIOD:
        return SysParaSet.leak_warn.CUD_delay & 0xFFFF;
      case SERVER_DETECTING_DELAY_PERIOD:
        return SysParaSet.leak_warn.server_delay & 0xFFFF;
      case SIDECAR_DETECTING_DELAY_PERIOD:
        return SysParaSet.leak_warn.sidecar_delay & 0xFFFF;
      case CDU_DEW_POINT_CALIBRATION:
        return SysParaSet.dew_temp_calib & 0xFFFF;
      case CDU_TEMP_COMPENSATION:
        return SysParaSet.ambient_temp_calib & 0xFFFF;
      case CDU_FLUID_INLET_TEMP_CALIBRATION:
        return SysParaSet.inlet_temp_calib & 0xFFFF;
      case CDU_FLUID_OUTLET_TEMP_CALIBRATION:
        return SysParaSet.outlet_temp_calib & 0xFFFF;
      case PT100_ABNORMAL_TEMPERATURE_LOW:
        return (SysParaSet.pt100_abnl_temp_low_m / 100) & 0xFFFF;
      case PT100_ABNORMAL_TEMPERATURE_HIGH:
        return (SysParaSet.pt100_abnl_temp_high_m / 100) & 0xFFFF;
      case NTC_ABNORMAL_TEMPERATURE_LOW:
        return (SysParaSet.ntc_abnl_temp_low_m / 100) & 0xFFFF;
      case NTC_ABNORMAL_TEMPERATURE_HIGH:
        return (SysParaSet.ntc_abnl_temp_high_m / 100) & 0xFFFF;
      case ABNORMAL_PRESSURE_LOW:
        return SysParaSet.press_abnl_val_low_kpa & 0xFFFF;
      case ABNORMAL_PRESSURE_HIGH:
        return SysParaSet.press_abnl_val_high_kpa & 0xFFFF;
      default:
        return 0;
    }
  } else if (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) {
    switch (addr) {
      case AUTOMATIC_MODE:
        return (SysParaSet.ctrl_mode = data) & 0xFFFF;
      case TEMPERATURE_SET_POINT:
        return (SysParaSet.temp_set = data * 100) & 0xFFFF;
      case FLOW_SET_POINT:
        return (SysParaSet.flow_set = data) & 0xFFFF;
      case DIFFERENTIAL_PRESSURE_SET_POINT:
        return (SysParaSet.press_set = data) & 0xFFFF;
      case MIN_PUMP_SPEED:
        return (SysParaSet.pump_min_duty = data) & 0xFFFF;
      case PUMP_STOP_DELAY:
        return (SysParaSet.pump_stop_delay = data) & 0xFFFF;
      case PRESSURE_ALARM_MODE:
        return (SysParaSet.press_warn.act = data) & 0xFFFF;
      case PRESSURE_ALARM_DELAY:
        return (SysParaSet.press_warn.delay = data) & 0xFFFF;
      case OUTLET_FLUID_LOW_PRESSURE_ALARM:
        return (SysParaSet.press_warn.outlet_low = data) & 0xFFFF;
      case OUTLET_FLUID_HIGH_PRESSURE_ALARM:
        return (SysParaSet.press_warn.outlet_high = data) & 0xFFFF;
      case INLET_FLUID_LOW_PRESSURE_ALARM:
        return (SysParaSet.press_warn.inlet_low = data) & 0xFFFF;
      case INLET_FLUID_HIGH_PRESSURE_ALARM:
        return (SysParaSet.press_warn.inlet_high = data) & 0xFFFF;
      case SIDECAR_RETURN_FLUID_LOW_PRESSURE_ALARM:
        return (SysParaSet.press_warn.return_low = data) & 0xFFFF;
      case SIDECAR_RETURN_FLUID_HIGH_PRESSURE_ALARM:
        return (SysParaSet.press_warn.return_high = data) & 0xFFFF;
      case FLOW_ALARM_MODE:
        return (SysParaSet.flow_warn.act = data) & 0xFFFF;
      case LOW_FLOW_ALARM:
        return (SysParaSet.flow_warn.low_val = data) & 0xFFFF;
      case LOW_FLOW_DELAY_PERIOD:
        return (SysParaSet.flow_warn.low_delay = data) & 0xFFFF;
      case OVER_FLOW_ALARM:
        return (SysParaSet.flow_warn.high_val = data) & 0xFFFF;
      case OVER_FLOW_DELAY_PERIOD:
        return (SysParaSet.flow_warn.high_delay = data) & 0xFFFF;
      case TEMP_ALARM_MODE:
        return (SysParaSet.temp_warn.act = data) & 0xFFFF;
      case TEMP_ALARM_DELAY_PERIOD:
        return (SysParaSet.temp_warn.delay = data) & 0xFFFF;
      case LOW_OUTLET_FLUID_TEMP_ALARM:
        return (SysParaSet.temp_warn.outlet_low = data) & 0xFFFF;
      case HIGH_OUTLET_FLUID_TEMP_ALARM:
        return (SysParaSet.temp_warn.outlet_high = data) & 0xFFFF;
      case HIGH_AMBIENT_TEMP_ALARM:
        return (SysParaSet.temp_warn.ambient_high = data) & 0xFFFF;
      case LEAK_DETECTION_MODE:
        return (SysParaSet.leak_warn.act = data) & 0xFFFF;
      case CDU_DETECTING_DELAY_PERIOD:
        return (SysParaSet.leak_warn.CUD_delay = data) & 0xFFFF;
      case SERVER_DETECTING_DELAY_PERIOD:
        return (SysParaSet.leak_warn.server_delay = data) & 0xFFFF;
      case SIDECAR_DETECTING_DELAY_PERIOD:
        return (SysParaSet.leak_warn.sidecar_delay = data) & 0xFFFF;
      case CDU_DEW_POINT_CALIBRATION:
        return (SysParaSet.dew_temp_calib = data) & 0xFFFF;
      case CDU_TEMP_COMPENSATION:
        return (SysParaSet.ambient_temp_calib = data) & 0xFFFF;
      case CDU_FLUID_INLET_TEMP_CALIBRATION:
        return (SysParaSet.inlet_temp_calib = data) & 0xFFFF;
      case CDU_FLUID_OUTLET_TEMP_CALIBRATION:
        return (SysParaSet.outlet_temp_calib = data) & 0xFFFF;
      case PT100_ABNORMAL_TEMPERATURE_LOW:
        return (SysParaSet.pt100_abnl_temp_low_m = data * 100) & 0xFFFF;
      case PT100_ABNORMAL_TEMPERATURE_HIGH:
        return (SysParaSet.pt100_abnl_temp_high_m = data * 100) & 0xFFFF;
      case NTC_ABNORMAL_TEMPERATURE_LOW:
        return (SysParaSet.ntc_abnl_temp_low_m = data * 100) & 0xFFFF;
      case NTC_ABNORMAL_TEMPERATURE_HIGH:
        return (SysParaSet.ntc_abnl_temp_high_m = data * 100) & 0xFFFF;
      case ABNORMAL_PRESSURE_LOW:
        return (SysParaSet.press_abnl_val_low_kpa = data) & 0xFFFF;
      case ABNORMAL_PRESSURE_HIGH:
        return (SysParaSet.press_abnl_val_high_kpa = data) & 0xFFFF;
      default:
        return 0;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}

uint32_t SysParaDisp_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case SYSTEM_ALARM_STATUS:
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
      case PERIPHERAL_STATUS:
        return SysParaDisp.periphery_component_exist & 0xFFFF;
      default:
        return 0;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}

uint32_t DataRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case PT100_1_TEMPERATURE:
        return (Pt100Stat.pt100_1_temp_m / 100) & 0xFFFF;
      case PT100_2_TEMPERATURE:
        return (Pt100Stat.pt100_2_temp_m / 100) & 0xFFFF;
      case PT100_3_TEMPERATURE:
        return (Pt100Stat.pt100_3_temp_m / 100) & 0xFFFF;
      case PT100_4_TEMPERATURE:
        return (Pt100Stat.pt100_4_temp_m / 100) & 0xFFFF;
      case NTC_1_TEMPERATURE:
        return (SensStat.ntc_1_temp_m / 100) & 0xFFFF;
      case NTC_2_TEMPERATURE:
        return (SensStat.ntc_2_temp_m / 100) & 0xFFFF;
      case NTC_3_TEMPERATURE:
        return (SensStat.ntc_3_temp_m / 100) & 0xFFFF;
      case NTC_4_TEMPERATURE:
        return (SensStat.ntc_4_temp_m / 100) & 0xFFFF;
      case PRESSURE_1_VALUE:
        return (SensStat.press_1_val_kpa) & 0xFFFF;
      case PRESSURE_2_VALUE:
        return (SensStat.press_2_val_kpa) & 0xFFFF;
      case PRESSURE_3_VALUE:
        return (SensStat.press_3_val_kpa) & 0xFFFF;
      case PRESSURE_4_VALUE:
        return (SensStat.press_4_val_kpa) & 0xFFFF;
      case FLOW_VALUE:
        return (SensStat.Flow_val) & 0xFFFF;
      case PUMP_1_FEEDBACK:
        return pump_status.pump_1_FB & 0xFFFF;
      case PUMP_2_FEEDBACK:
        return pump_status.pump_2_FB & 0xFFFF;
      case LEAK_SENSOR:
        return SensStat.leak_sensor & 0xFFFF;
      case POWER_1_STATUS:
        return Power_1_Stat.power_status & 0xFFFF;
      case POWER_1_VOLTAGE_INPUT:
        return Power_1_Stat.voltage_input & 0xFFFF;
      case POWER_1_CURRENT_INPUT:
        return Power_1_Stat.current_input & 0xFFFF;
      case POWER_1_TEMPERATURE:
        return Power_1_Stat.power_temp & 0xFFFF;
      case POWER_2_STATUS:
        return Power_2_Stat.power_status & 0xFFFF;
      case POWER_2_VOLTAGE_INPUT:
        return Power_2_Stat.voltage_input & 0xFFFF;
      case POWER_2_CURRENT_INPUT:
        return Power_2_Stat.current_input & 0xFFFF;
      case POWER_2_TEMPERATURE:
        return Power_2_Stat.power_temp & 0xFFFF;
      case TEMPERATURE:
        return SensStat.temperature & 0xFFFF;
      case HUMIDITY:
        return SensStat.humidity & 0xFFFF;
      case DEW_TEMPERATURE:
        return SensStat.dew_temp & 0xFFFF;
      case FAN_1_FEEDBACK_TEMP:
        return FansCardStat.fan_fg[0] & 0xFFFF;
      case DEVICE_CONNECTED:
        return SensStat.device_connected & 0xFFFF;
      case RESERVED_1:
        return 0;
      case RESERVED_2:
        return 0;
      case RESERVED_3:
        return 0;
      default:
        return 0;
    }
  } else if (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) {
    if (root) {
      switch (addr) {
        case PT100_1_TEMPERATURE:
          return 0;
        case PT100_2_TEMPERATURE:
          return 0;
        case PT100_3_TEMPERATURE:
          return 0;
        case PT100_4_TEMPERATURE:
          return 0;
        case NTC_1_TEMPERATURE:
          return 0;
        case NTC_2_TEMPERATURE:
          return 0;
        case NTC_3_TEMPERATURE:
          return 0;
        case NTC_4_TEMPERATURE:
          return 0;
        case PRESSURE_1_VALUE:
          return 0;
        case PRESSURE_2_VALUE:
          return 0;
        case PRESSURE_3_VALUE:
          return 0;
        case PRESSURE_4_VALUE:
          return 0;
        case FLOW_VALUE:
          return 0;
        case PUMP_1_FEEDBACK:
          return 0;
        case PUMP_2_FEEDBACK:
          return 0;
        case LEAK_SENSOR:
          return 0;
        case POWER_1_STATUS:
          return 0;
        case POWER_1_VOLTAGE_INPUT:
          return 0;
        case POWER_1_CURRENT_INPUT:
          return 0;
        case POWER_1_TEMPERATURE:
          return 0;
        case POWER_2_STATUS:
          return 0;
        case POWER_2_VOLTAGE_INPUT:
          return 0;
        case POWER_2_CURRENT_INPUT:
          return 0;
        case POWER_2_TEMPERATURE:
          return 0;
        case TEMPERATURE:
          return (SensStat.temperature = data) & 0xFFFF;
        case HUMIDITY:
          return (SensStat.humidity = data) & 0xFFFF;
        case DEW_TEMPERATURE:
          return (SensStat.dew_temp = data) & 0xFFFF;
        case FAN_1_FEEDBACK_TEMP:
          return 0;
        case DEVICE_CONNECTED:
          return 0;
        case RESERVED_1:
          return 0;
        case RESERVED_2:
          return 0;
        case RESERVED_3:
          return 0;
        default:
          return 0;
      }
    } else {
      return ILLIGAL_FUNC << 16;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}

uint32_t DevCtrl_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) {
    if (data >= 0 && data <= 1000) {
      switch (addr) {
        case PUMP_1_RPM:
          return (pump_control.pump_1_rpm = data) & 0xFFFF;
        case PUMP_2_RPM:
          return (pump_control.pump_2_rpm = data) & 0xFFFF;
        case PROPORTIONAL_VALVE_1_DUTY:
          return (SensCtrl.porpo_1_duty = data) & 0xFFFF;
        case PROPORTIONAL_VALVE_2_DUTY:
          return (SensCtrl.porpo_2_duty = data) & 0xFFFF;
        case RESERVED_CTRL_1:
          return 0;
        case RESERVED_CTRL_2:
          return 0;
        case RESERVED_CTRL_3:
          return 0;
        case RESERVED_CTRL_4:
          return 0;
        case RESERVED_CTRL_5:
          return 0;
        case RESERVED_CTRL_6:
          return 0;
        case RESERVED_CTRL_7:
          return 0;
        case RESERVED_CTRL_8:
          return 0;
        case RESERVED_CTRL_9:
          return 0;
        case RESERVED_CTRL_10:
          return 0;
        case RESERVED_CTRL_11:
          return 0;
        case RESERVED_CTRL_12:
          return 0;
        default:
          return 0;
      }
    } else {
      return ILLIGAL_DATA_VALUE << 16;
    }
  } else if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case PUMP_1_RPM:
        return (pump_control.pump_1_rpm & 0xFFFF);
      case PUMP_2_RPM:
        return (pump_control.pump_2_rpm & 0xFFFF);
      case PROPORTIONAL_VALVE_1_DUTY:
        return (SensCtrl.porpo_1_duty & 0xFFFF);
      case PROPORTIONAL_VALVE_2_DUTY:
        return (SensCtrl.porpo_2_duty & 0xFFFF);
      case RESERVED_CTRL_1:
        return 0;
      case RESERVED_CTRL_2:
        return 0;
      case RESERVED_CTRL_3:
        return 0;
      case RESERVED_CTRL_4:
        return 0;
      case RESERVED_CTRL_5:
        return 0;
      case RESERVED_CTRL_6:
        return 0;
      case RESERVED_CTRL_7:
        return 0;
      case RESERVED_CTRL_8:
        return 0;
      case RESERVED_CTRL_9:
        return 0;
      case RESERVED_CTRL_10:
        return 0;
      case RESERVED_CTRL_11:
        return 0;
      case RESERVED_CTRL_12:
        return 0;
      default:
        return 0;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}
#endif

#if defined(SENS_RS485) || defined(CDU_RS485)
uint32_t SideCar_Sens_DataRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case SIDECAR_PT100_1_TEMPERATURE:
        return (SensCardStat.pt100_1_temp_m / 100) & 0xFFFF;
      case SIDECAR_PT100_2_TEMPERATURE:
        return (SensCardStat.pt100_2_temp_m / 100) & 0xFFFF;
      case SIDECAR_PT100_3_TEMPERATURE:
        return (SensCardStat.pt100_3_temp_m / 100) & 0xFFFF;
      case SIDECAR_PT100_4_TEMPERATURE:
        return (SensCardStat.pt100_4_temp_m / 100) & 0xFFFF;
      case SIDECAR_PRESSURE_1_VALUE:
        return (SensCardStat.press_1_val_kpa) & 0xFFFF;
      case SIDECAR_PRESSURE_2_VALUE:
        return (SensCardStat.press_2_val_kpa) & 0xFFFF;
      case SIDECAR_LEAK_SENSOR:
        return SensCardStat.leak_sensor & 0xFFFF;
      case SIDECAR_TEMPERATURE:
        return SensCardStat.temperature & 0xFFFF;
      case SIDECAR_HUMIDITY:
        return SensCardStat.humidity & 0xFFFF;
      case SIDECAR_RESERVED_1:
        return 0;
      case SIDECAR_RESERVED_2:
        return 0;
      case SIDECAR_RESERVED_3:
        return 0;
      case SIDECAR_RESERVED_4:
        return 0;
      case SIDECAR_RESERVED_5:
        return 0;
      case SIDECAR_RESERVED_6:
        return 0;
      case SIDECAR_RESERVED_7:
        return 0;
      default:
        return 0;
    }
  } else if (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) {
    if (root) {
      switch (addr) {
        case SIDECAR_PT100_1_TEMPERATURE:
          return (SensCardStat.pt100_1_temp_m = data * 100) & 0xFFFF;
        case SIDECAR_PT100_2_TEMPERATURE:
          return (SensCardStat.pt100_2_temp_m = data * 100) & 0xFFFF;
        case SIDECAR_PT100_3_TEMPERATURE:
          return (SensCardStat.pt100_3_temp_m = data * 100) & 0xFFFF;
        case SIDECAR_PT100_4_TEMPERATURE:
          return (SensCardStat.pt100_4_temp_m = data * 100) & 0xFFFF;
        case SIDECAR_PRESSURE_1_VALUE:
          return (SensCardStat.press_1_val_kpa = data) & 0xFFFF;
        case SIDECAR_PRESSURE_2_VALUE:
          return (SensCardStat.press_2_val_kpa = data) & 0xFFFF;
        case SIDECAR_LEAK_SENSOR:
          return (SensCardStat.leak_sensor = data) & 0xFFFF;
        case SIDECAR_TEMPERATURE:
          return (SensCardStat.temperature = data) & 0xFFFF;
        case SIDECAR_HUMIDITY:
          return (SensCardStat.humidity = data) & 0xFFFF;
        case SIDECAR_RESERVED_1:
          return 0;
        case SIDECAR_RESERVED_2:
          return 0;
        case SIDECAR_RESERVED_3:
          return 0;
        case SIDECAR_RESERVED_4:
          return 0;
        case SIDECAR_RESERVED_5:
          return 0;
        case SIDECAR_RESERVED_6:
          return 0;
        case SIDECAR_RESERVED_7:
          return 0;
        default:
          return 0;
      }
    } else {
      return ILLIGAL_FUNC << 16;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}

uint32_t SideCar_Sens_DevCtrl_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
#ifdef CDU_RS485
  if (func == WRITE_SINGLE_REGISTER && !root) {
    write_ip = SENS_RS485_ADDR;
    write_card_address = addr;
    write_card_data = data;
    xTaskNotifyGive(WriteCardHandler);
    return 0;
  }
#endif
  if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case SIDECAR_PRESSURE_PUMP:
        return SensCardCtrl.pressure_pump & 0xFFFF;
      case SIDECAR_RESERVED_CTRL_1:
        return 0;
      case SIDECAR_RESERVED_CTRL_2:
        return 0;
      case SIDECAR_RESERVED_CTRL_3:
        return 0;
      case SIDECAR_RESERVED_CTRL_4:
        return 0;
      case SIDECAR_RESERVED_CTRL_5:
        return 0;
      case SIDECAR_RESERVED_CTRL_6:
        return 0;
      case SIDECAR_RESERVED_CTRL_7:
        return 0;
      case SIDECAR_RESERVED_CTRL_8:
        return 0;
      case SIDECAR_RESERVED_CTRL_9:
        return 0;
      case SIDECAR_RESERVED_CTRL_10:
        return 0;
      case SIDECAR_RESERVED_CTRL_11:
        return 0;
      case SIDECAR_RESERVED_CTRL_12:
        return 0;
      case SIDECAR_RESERVED_CTRL_13:
        return 0;
      case SIDECAR_RESERVED_CTRL_14:
        return 0;
      case SIDECAR_RESERVED_CTRL_15:
        return 0;
      default:
        return 0;
    }
  } else if (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) {
    if (root) {
      switch (addr) {
        case SIDECAR_PRESSURE_PUMP:
          return (SensCardCtrl.pressure_pump = data) & 0xFFFF;
        case SIDECAR_RESERVED_CTRL_1:
          return 0;
        case SIDECAR_RESERVED_CTRL_2:
          return 0;
        case SIDECAR_RESERVED_CTRL_3:
          return 0;
        case SIDECAR_RESERVED_CTRL_4:
          return 0;
        case SIDECAR_RESERVED_CTRL_5:
          return 0;
        case SIDECAR_RESERVED_CTRL_6:
          return 0;
        case SIDECAR_RESERVED_CTRL_7:
          return 0;
        case SIDECAR_RESERVED_CTRL_8:
          return 0;
        case SIDECAR_RESERVED_CTRL_9:
          return 0;
        case SIDECAR_RESERVED_CTRL_10:
          return 0;
        case SIDECAR_RESERVED_CTRL_11:
          return 0;
        case SIDECAR_RESERVED_CTRL_12:
          return 0;
        case SIDECAR_RESERVED_CTRL_13:
          return 0;
        case SIDECAR_RESERVED_CTRL_14:
          return 0;
        case SIDECAR_RESERVED_CTRL_15:
          return 0;
        default:
          return ILLIGAL_DATA_ADDR << 16;
      }
    } else {
      return ILLIGAL_FUNC << 16;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}

#endif

#if defined(FAN_RS485) || defined(CDU_RS485)

uint32_t FansCard_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
#ifdef CDU_RS485
  if (func == WRITE_SINGLE_REGISTER && !root) {
    write_ip = FAN_RS485_ADDR;
    write_card_address = addr;
    write_card_data = data;
    xTaskNotifyGive(WriteCardHandler);
    return 0;
  }
#endif
  if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case FAN_1_FEEDBACK:
        return FansCardStat.fan_fg[0] & 0xFFFF;
      case FAN_2_FEEDBACK:
        return FansCardStat.fan_fg[1] & 0xFFFF;
      case FAN_3_FEEDBACK:
        return FansCardStat.fan_fg[2] & 0xFFFF;
      case FAN_4_FEEDBACK:
        return FansCardStat.fan_fg[3] & 0xFFFF;
      case FAN_5_FEEDBACK:
        return FansCardStat.fan_fg[4] & 0xFFFF;
      case FAN_6_FEEDBACK:
        return FansCardStat.fan_fg[5] & 0xFFFF;
      case FAN_7_FEEDBACK:
        return FansCardStat.fan_fg[6] & 0xFFFF;
      case FAN_8_FEEDBACK:
        return FansCardStat.fan_fg[7] & 0xFFFF;
      case FAN_9_FEEDBACK:
        return FansCardStat.fan_fg[8] & 0xFFFF;
      case FAN_10_FEEDBACK:
        return FansCardStat.fan_fg[9] & 0xFFFF;
      case FAN_11_FEEDBACK:
        return FansCardStat.fan_fg[10] & 0xFFFF;
      case FAN_12_FEEDBACK:
        return FansCardStat.fan_fg[11] & 0xFFFF;
      case FAN_13_FEEDBACK:
        return FansCardStat.fan_fg[12] & 0xFFFF;
      case FAN_14_FEEDBACK:
        return FansCardStat.fan_fg[13] & 0xFFFF;
      case FAN_15_FEEDBACK:
        return FansCardStat.fan_fg[14] & 0xFFFF;
      case FAN_16_FEEDBACK:
        return FansCardStat.fan_fg[15] & 0xFFFF;
      case FAN_1_DUTY:
        return FansCardCtrl.fan_pwm[0] & 0xFFFF;
      case FAN_2_DUTY:
        return FansCardCtrl.fan_pwm[1] & 0xFFFF;
      case FAN_3_DUTY:
        return FansCardCtrl.fan_pwm[2] & 0xFFFF;
      case FAN_4_DUTY:
        return FansCardCtrl.fan_pwm[3] & 0xFFFF;
      case FAN_5_DUTY:
        return FansCardCtrl.fan_pwm[4] & 0xFFFF;
      case FAN_6_DUTY:
        return FansCardCtrl.fan_pwm[5] & 0xFFFF;
      case FAN_7_DUTY:
        return FansCardCtrl.fan_pwm[6] & 0xFFFF;
      case FAN_8_DUTY:
        return FansCardCtrl.fan_pwm[7] & 0xFFFF;
      case FAN_9_DUTY:
        return FansCardCtrl.fan_pwm[8] & 0xFFFF;
      case FAN_10_DUTY:
        return FansCardCtrl.fan_pwm[9] & 0xFFFF;
      case FAN_11_DUTY:
        return FansCardCtrl.fan_pwm[10] & 0xFFFF;
      case FAN_12_DUTY:
        return FansCardCtrl.fan_pwm[11] & 0xFFFF;
      case FAN_13_DUTY:
        return FansCardCtrl.fan_pwm[12] & 0xFFFF;
      case FAN_14_DUTY:
        return FansCardCtrl.fan_pwm[13] & 0xFFFF;
      case FAN_15_DUTY:
        return FansCardCtrl.fan_pwm[14] & 0xFFFF;
      case FAN_16_DUTY:
        return FansCardCtrl.fan_pwm[15] & 0xFFFF;
      default:
        return 0;
    }
  } else if (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) {
    if (addr >= FAN_1_FEEDBACK && addr <= FAN_16_FEEDBACK) {
      if (root) {
        switch (addr) {
          case FAN_1_FEEDBACK:
            return (FansCardStat.fan_fg[0] = data) & 0xFFFF;
          case FAN_2_FEEDBACK:
            return (FansCardStat.fan_fg[1] = data) & 0xFFFF;
          case FAN_3_FEEDBACK:
            return (FansCardStat.fan_fg[2] = data) & 0xFFFF;
          case FAN_4_FEEDBACK:
            return (FansCardStat.fan_fg[3] = data) & 0xFFFF;
          case FAN_5_FEEDBACK:
            return (FansCardStat.fan_fg[4] = data) & 0xFFFF;
          case FAN_6_FEEDBACK:
            return (FansCardStat.fan_fg[5] = data) & 0xFFFF;
          case FAN_7_FEEDBACK:
            return (FansCardStat.fan_fg[6] = data) & 0xFFFF;
          case FAN_8_FEEDBACK:
            return (FansCardStat.fan_fg[7] = data) & 0xFFFF;
          case FAN_9_FEEDBACK:
            return (FansCardStat.fan_fg[8] = data) & 0xFFFF;
          case FAN_10_FEEDBACK:
            return (FansCardStat.fan_fg[9] = data) & 0xFFFF;
          case FAN_11_FEEDBACK:
            return (FansCardStat.fan_fg[10] = data) & 0xFFFF;
          case FAN_12_FEEDBACK:
            return (FansCardStat.fan_fg[11] = data) & 0xFFFF;
          case FAN_13_FEEDBACK:
            return (FansCardStat.fan_fg[12] = data) & 0xFFFF;
          case FAN_14_FEEDBACK:
            return (FansCardStat.fan_fg[13] = data) & 0xFFFF;
          case FAN_15_FEEDBACK:
            return (FansCardStat.fan_fg[14] = data) & 0xFFFF;
          case FAN_16_FEEDBACK:
            return (FansCardStat.fan_fg[15] = data) & 0xFFFF;
          default:
            return ILLIGAL_DATA_ADDR << 16;
        }
      } else {
        return ILLIGAL_FUNC << 16;
      }
    } else if (addr >= FAN_1_DUTY && addr <= FAN_16_DUTY) {
      if (data >= 0 && data <= 1000) {
        switch (addr) {
          case FAN_1_DUTY:
            return (FansCardCtrl.fan_pwm[0] = data) & 0xFFFF;
          case FAN_2_DUTY:
            return (FansCardCtrl.fan_pwm[1] = data) & 0xFFFF;
          case FAN_3_DUTY:
            return (FansCardCtrl.fan_pwm[2] = data) & 0xFFFF;
          case FAN_4_DUTY:
            return (FansCardCtrl.fan_pwm[3] = data) & 0xFFFF;
          case FAN_5_DUTY:
            return (FansCardCtrl.fan_pwm[4] = data) & 0xFFFF;
          case FAN_6_DUTY:
            return (FansCardCtrl.fan_pwm[5] = data) & 0xFFFF;
          case FAN_7_DUTY:
            return (FansCardCtrl.fan_pwm[6] = data) & 0xFFFF;
          case FAN_8_DUTY:
            return (FansCardCtrl.fan_pwm[7] = data) & 0xFFFF;
          case FAN_9_DUTY:
            return (FansCardCtrl.fan_pwm[8] = data) & 0xFFFF;
          case FAN_10_DUTY:
            return (FansCardCtrl.fan_pwm[9] = data) & 0xFFFF;
          case FAN_11_DUTY:
            return (FansCardCtrl.fan_pwm[10] = data) & 0xFFFF;
          case FAN_12_DUTY:
            return (FansCardCtrl.fan_pwm[11] = data) & 0xFFFF;
          case FAN_13_DUTY:
            return (FansCardCtrl.fan_pwm[12] = data) & 0xFFFF;
          case FAN_14_DUTY:
            return (FansCardCtrl.fan_pwm[13] = data) & 0xFFFF;
          case FAN_15_DUTY:
            return (FansCardCtrl.fan_pwm[14] = data) & 0xFFFF;
          case FAN_16_DUTY:
            return (FansCardCtrl.fan_pwm[15] = data) & 0xFFFF;
          default:
            return ILLIGAL_DATA_ADDR << 16;
        }
      } else {
        return ILLIGAL_DATA_VALUE << 16;
      }
    } else {
      return ILLIGAL_DATA_ADDR << 16;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}

uint32_t FanCardSysSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
#ifdef CDU_RS485
  if (func == WRITE_SINGLE_REGISTER && !root) {
    write_ip = FAN_RS485_ADDR;
    write_card_address = addr;
    write_card_data = data;
    xTaskNotifyGive(WriteCardHandler);
    return 0;
  }
#endif

  if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case FAN_ALARM_MODE:
        return FanCardSysSet.fan_alarm.act & 0xFFFF;
      case FAN_ALARM_DELAY:
        return FanCardSysSet.fan_alarm.delay & 0xFFFF;
      case FAN_BOARD_AUTO_CONTROL:
        return FanCardSysSet.auto_control & 0xFFFF;
      case AUTO_CONTROL_TARGET_SPEED:
        return FanCardSysSet.auto_control_target_speed & 0xFFFF;
      case FAN_INSTALLATION_STATUS:
        return FanCardSysSet.fan_installation_status & 0xFFFF;
      case FAN_LOW_SPEED_WARNING_THRESHOLD:
        return FanCardSysSet.fan_alarm.fan_low_speed_warning_threshold & 0xFFFF;
      case FAN_FG_DIFFERENCE_WARNING_THRESHOLD:
        return FanCardSysSet.fan_alarm.fan_fg_difference_warning_threshold & 0xFFFF;
      default:
        return 0;
    }
  } else if (func == WRITE_SINGLE_REGISTER || func == WRITE_MULTIPLE_REGISTERS) {
    switch (addr) {
      case FAN_ALARM_MODE:
        return (FanCardSysSet.fan_alarm.act = data) & 0xFFFF;
      case FAN_ALARM_DELAY:
        return (FanCardSysSet.fan_alarm.delay = data) & 0xFFFF;
      case FAN_BOARD_AUTO_CONTROL:
        return (FanCardSysSet.auto_control = data) & 0xFFFF;
      case AUTO_CONTROL_TARGET_SPEED:
        return (FanCardSysSet.auto_control_target_speed = data) & 0xFFFF;
      case FAN_INSTALLATION_STATUS:
        return (FanCardSysSet.fan_installation_status = data) & 0xFFFF;
      case FAN_LOW_SPEED_WARNING_THRESHOLD:
        return (FanCardSysSet.fan_alarm.fan_low_speed_warning_threshold = data) & 0xFFFF;
      case FAN_FG_DIFFERENCE_WARNING_THRESHOLD:
        return (FanCardSysSet.fan_alarm.fan_fg_difference_warning_threshold = data) & 0xFFFF;
      default:
        return ILLIGAL_DATA_ADDR << 16;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}

uint32_t FanCardSysDisp_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root) {
  if (func == READ_HOLDING_REGISTERS) {
    switch (addr) {
      case FAN_BOARD_FAULT_STATUS:
        return FanCardSysDisp.fan_board_fault_status & 0xFFFF;
      case FAN_STATUS_ON_FAN_BOARD_BITFIELD_0_15:
        return FanCardSysDisp.fan_status_on_fan_board_bitfield_0_15 & 0xFFFF;
      default:
        return 0;
    }
  } else {
    return ILLIGAL_FUNC << 16;
  }
}
#endif