#ifndef __RS485_REGION_H_
#define __RS485_REGION_H_
#include <stdbool.h>
#include <stdint.h>

#include "RS485.h"
#include "RS485_Region_handler_enum.h"

void RsRegHdle_Init(void);

uint32_t SysBasicInform_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t MainSysParaSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t MainSysAlarmParaSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t MainSysParaDisp_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t MainDataRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t MainDataCalib_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t CDU_PowerRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t MainDevCtrl_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t SensDataRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t SensDataCalib_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t SensDevCtrl_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t SideCarPowerRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t FanSysSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t FanSysDisp_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t Fans_FG_PWM_Set_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);

#endif  // __RS485_REGION_H_