/**
 ****************************************************************************************************
 * @file        bldc_tim.h
 * @brief       BLDC — TIM8 三相 PWM（PI5/6/7 = CH1/2/3，PH13/14/15 = CH1N/2N/3N）
 ****************************************************************************************************
 */

#ifndef __BLDC_TIM_H
#define __BLDC_TIM_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* TIM8:上桥 PI5/6/7，下桥互补 PH13/14/15 */

#define BLDC_TIMX_PWM_CH1_GPIO_PORT            GPIOI
#define BLDC_TIMX_PWM_CH1_GPIO_PIN             GPIO_PIN_5
#define BLDC_TIMX_PWM_CH1_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)

#define BLDC_TIMX_PWM_CH2_GPIO_PORT            GPIOI
#define BLDC_TIMX_PWM_CH2_GPIO_PIN             GPIO_PIN_6
#define BLDC_TIMX_PWM_CH2_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)

#define BLDC_TIMX_PWM_CH3_GPIO_PORT            GPIOI
#define BLDC_TIMX_PWM_CH3_GPIO_PIN             GPIO_PIN_7
#define BLDC_TIMX_PWM_CH3_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)

#define BLDC_TIMX_PWM_CH1N_GPIO_PORT           GPIOH
#define BLDC_TIMX_PWM_CH1N_GPIO_PIN            GPIO_PIN_13
#define BLDC_TIMX_PWM_CH2N_GPIO_PORT           GPIOH
#define BLDC_TIMX_PWM_CH2N_GPIO_PIN            GPIO_PIN_14
#define BLDC_TIMX_PWM_CH3N_GPIO_PORT           GPIOH
#define BLDC_TIMX_PWM_CH3N_GPIO_PIN            GPIO_PIN_15
#define BLDC_TIMX_PWM_CHXN_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)

#define BLDC_TIMX_PWM_CHY_GPIO_AF              GPIO_AF3_TIM8

#define BLDC_TIMX_PWM                          TIM8
#define BLDC_TIMX_PWM_IRQn                     TIM8_UP_TIM13_IRQn
#define BLDC_TIMX_PWM_IRQHandler               TIM8_UP_TIM13_IRQHandler
#define BLDC_TIMX_PWM_CH1                      TIM_CHANNEL_1
#define BLDC_TIMX_PWM_CH2                      TIM_CHANNEL_2
#define BLDC_TIMX_PWM_CH3                      TIM_CHANNEL_3
#define BLDC_TIMX_PWM_CHY_CLK_ENABLE()         do{ __HAL_RCC_TIM8_CLK_ENABLE(); }while(0)

extern TIM_HandleTypeDef g_bldc_timx_handle;

void bldc_timx_oc_chy_init(uint16_t arr, uint16_t psc);
void bldc_motor1_pwm_commit_and_kick(void);

#endif
