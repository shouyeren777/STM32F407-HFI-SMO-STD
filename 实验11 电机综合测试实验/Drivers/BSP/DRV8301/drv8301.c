/**
 * @file        drv8301.c
 * @brief       
 */
#include "./BSP/DRV8301/drv8301.h"
#include "./BSP/SPI/spi.h"
#include <string.h>

DRV8301_handl drv8301;
DRV8301_handl *drv8301_handl = &drv8301;

void DRV8301_Init(void)
{
    memset(drv8301_handl, 0, sizeof(DRV8301_handl));

    /* TI 版还配置 EN_GATE / DC_CAL / EN_BUCK 等 GPIO；本板若未引出可仅在硬件上固定电平 */
    spi2_init();
    spi2_set_speed(DRV8301_SPI_SPEED_INDEX);
}

void DRV8301_Send(uint16_t data)
{
    (void)spi2_transfer16(data);
}

void DRV8301_Senddata(DRV8301_handl *handl)
{
    DRV8301_Send(handl->reg0);
    DRV8301_Send(handl->reg1);
    DRV8301_Send(handl->reg2);
    DRV8301_Send(handl->reg3);
}

void DRV8301_Read(DRV8301_handl *handl)
{
    /*
     * SLOS719 7.5.1.1：SDO 第 N 帧输出的是对上一帧 SDI(N-1) 的响应；
     * 每个读地址需 16 SCLK 发命令 + 下一帧 16 SCLK 取回数据。
     */
    (void)spi2_transfer16(0x8000U);
    handl->reg0 = spi2_transfer16(0x8800U);
    handl->reg1 = spi2_transfer16(0x9000U);
    handl->reg2 = spi2_transfer16(0x9800U);
    handl->reg3 = spi2_transfer16(0x8000U);
}
