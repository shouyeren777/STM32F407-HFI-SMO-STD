/**
 * @file        drv8301.h
 * @brief       自 TI DRV8301_test1 移植的寄存器与 SPI 原语（逻辑与 TI 一致，适配 STM32）
 */
#ifndef __DRV8301_H
#define __DRV8301_H

#include <stdint.h>

typedef struct _DRV8301
{
    uint16_t reg0;
    uint16_t reg1;
    uint16_t reg2;
    uint16_t reg3;

    uint8_t set_flag;
    uint8_t down;
    uint8_t up;
    uint8_t ok;
    /* STM32 / LCD：非阻塞编辑（对应 TI 中内层 while+ScanKey） */
    uint8_t editing;
    uint8_t edit_temp;
} DRV8301_handl;

extern DRV8301_handl drv8301;
extern DRV8301_handl *drv8301_handl;

void DRV8301_Init(void);
void DRV8301_Read(DRV8301_handl *handl);
void DRV8301_Send(uint16_t data);
void DRV8301_Senddata(DRV8301_handl *handl);

#endif
