#ifndef _MAIN_H
#define _MAIN_H

#include "../BSP/SYSTEM/sys/sys.h"
#include "../BSP/SYSTEM/delay/delay.h"
#include <stdint.h>
#include <math.h>

#include "../BLDC_PMSM_CONTROL/User/Platform/f407_bldc_platform.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef signed long long s64;

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern ADC_HandleTypeDef hadc3;

/* Compatibility shims for migrated modules on F407 HAL. */
#ifndef ADC_SINGLE_ENDED
#define ADC_SINGLE_ENDED 0U
#endif

#ifndef HAL_ADCEx_Calibration_Start
#define HAL_ADCEx_Calibration_Start(_hadc, _mode) ((void)0)
#endif

void Error_Handler(void);

#endif
