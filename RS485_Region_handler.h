#ifndef __RS485_REGION_H_
#define __RS485_REGION_H_
#include <stdbool.h>
#include <stdint.h>

#include "RS485.h"
#include "RS485_Region_handler_enum.h"

uint32_t SysInform_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t SysParaSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t SysParaDisp_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t DataRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t DevCtrl_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t SideCar_Sens_DataRead_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t SideCar_Sens_DevCtrl_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t FansCard_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t FanCardSysSet_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);
uint32_t FanCardSysDisp_Handler(RsFunc_t func, uint16_t addr, uint16_t data, uint8_t len, bool root);

#endif  // __RS485_REGION_H_