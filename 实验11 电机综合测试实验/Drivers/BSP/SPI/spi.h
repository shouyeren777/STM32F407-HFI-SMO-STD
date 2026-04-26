/**
 ****************************************************************************************************
 * @file        spi.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-26
 * @brief       SPI 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台：正点原子 F407电机开发板
 * 在线视频：www.yuanzige.com
 * 技术论坛：http://www.openedv.com/forum.php
 * 公司网址：www.alientek.com
 * 购买地址：zhengdianyuanzi.tmall.com
 *
 * 修改说明
 * V1.0 20211026
 * 第一次发布
 *
 ****************************************************************************************************
 */

 #ifndef __SPI_H
 #define __SPI_H
 
 #include "./SYSTEM/sys/sys.h"
 
 
 /******************************************************************************************/
 /* SPI2 引脚 定义 */
 
 #define SPI2_CS_GPIO_PORT               GPIOI
 #define SPI2_CS_GPIO_PIN                GPIO_PIN_0
 #define SPI2_CS_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)   /* PI口时钟使能 */
 
 #define SPI2_SCK_GPIO_PORT              GPIOI
 #define SPI2_SCK_GPIO_PIN               GPIO_PIN_1
 #define SPI2_SCK_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)   /* PI口时钟使能 */
 
 #define SPI2_MISO_GPIO_PORT             GPIOI
 #define SPI2_MISO_GPIO_PIN              GPIO_PIN_2
 #define SPI2_MISO_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)   /* PI口时钟使能 */
 
 #define SPI2_MOSI_GPIO_PORT             GPIOI
 #define SPI2_MOSI_GPIO_PIN              GPIO_PIN_3
 #define SPI2_MOSI_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)   /* PI口时钟使能 */
 
 /* SPI2相关定义 */
 #define SPI2_SPI                        SPI2
 #define SPI2_SPI_CLK_ENABLE()           do{ __HAL_RCC_SPI2_CLK_ENABLE(); }while(0)    /* SPI2时钟使能 */

 /*
  * CS 与 DRV8301 nSCS（芯片侧：nSCS 高空闲、低有效通信）
  * 0（默认）：MCU 直连 nSCS — 空闲高、传输拉低
  * 1：MCU 空闲低、传输拉高（经反相后再接 nSCS）
  */
 #ifndef SPI2_CS_GPIO_ACTS_HIGH_DURING_XFER
 #define SPI2_CS_GPIO_ACTS_HIGH_DURING_XFER  0
 #endif

 /******************************************************************************************/
 
 
 /* SPI总线速度设置 */
 #define SPI_SPEED_2         0
 #define SPI_SPEED_4         1
 #define SPI_SPEED_8         2
 #define SPI_SPEED_16        3
 #define SPI_SPEED_32        4
 #define SPI_SPEED_64        5
 #define SPI_SPEED_128       6
 #define SPI_SPEED_256       7

 /*
  * DRV8301 SLOS719 §6.7 SPI：tCLK 最小周期 100 ns → f_SCLK ≤ 10 MHz；tCLKH、tCLKL 各 ≥ 40 ns。
  * 本工程 SYSCLK=168 MHz、APB1÷4 → SPI2 PCLK1=42 MHz 时：
  *   ÷4 (SPI_SPEED_4) ≈ 10.5 MHz 已超手册上限，禁止使用；
  *   目标 SCK ≈200 kHz：硬件 BR 仅支持 2^n 分频，42 MHz÷256≈164 kHz（最接近 200 kHz 的可配值；÷128≈328 kHz）。
  */
 #ifndef DRV8301_SPI_SPEED_INDEX
 #define DRV8301_SPI_SPEED_INDEX   SPI_SPEED_256
 #endif

void spi2_init(void);
void spi2_set_speed(uint8_t speed);
uint8_t spi2_read_write_byte(uint8_t txdata);
/** DRV8301：nSCS 低有效期间单次 16 位帧（HAL 16bit 模式，非拆成地址/数据两字节 API） */
uint16_t spi2_transfer16(uint16_t tx_word);

#endif
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 