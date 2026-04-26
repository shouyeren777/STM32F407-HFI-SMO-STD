#ifndef F407_BLDC_PLATFORM_H
#define F407_BLDC_PLATFORM_H

#include "../../../BSP/SYSTEM/sys/sys.h"

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc2;

extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern TIM_HandleTypeDef htim1; /* Mapped to TIM8 for F407 PWM. */
extern TIM_HandleTypeDef htim2; /* 10 kHz task scheduler timer. */
extern TIM_HandleTypeDef htim6; /* Free-running base timer. */

void bldc_key_input_init(void);
void bldc_adc_bemf_amp_input_init(void);
void bldc_pwm_gpio_init(void);
void bldc_registers_init(void);
void bldc_platform_init(void);

#endif /* F407_BLDC_PLATFORM_H */
