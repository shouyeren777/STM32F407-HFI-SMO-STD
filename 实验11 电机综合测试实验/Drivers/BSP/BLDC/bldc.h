/**
 ****************************************************************************************************
 * @file        bldc.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       BLDC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */
#ifndef __BLDC_H
#define __BLDC_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
typedef struct {
    __IO uint8_t    run_flag;       /* 运行标志 */
    __IO uint8_t    locked_rotor;   /* 堵转标记 */
    __IO uint8_t    step_sta;       /* 本次霍尔状态 */
    __IO uint8_t    hall_single_sta;/* 单个霍尔状态 */
    __IO uint8_t    hall_sta_edge;  /* 单个霍尔状态跳变 */
    __IO uint8_t    step_last;      /* 上次霍尔状态 */
    __IO uint8_t    dir;            /* 电机旋转方向 */
    __IO int32_t    pos;            /* 电机位置 */
    __IO int32_t    speed;          /* 电机速度 */
    __IO int16_t    current;        /* 电机速度 */
    __IO uint16_t   pwm_duty;       /* 电机占空比 */
    __IO uint32_t   hall_keep_t;    /* 霍尔保持时间 */
    __IO uint32_t   hall_pul_num;   /* 霍尔传感器脉冲数 */
    __IO uint32_t   lock_time;      /* 电机堵转时间 */
    __IO uint32_t   no_single;
    __IO uint32_t   count_j;
} _bldc_obj;

/******************************************************************************************/
#define MOTOR_1                     1



extern _bldc_obj g_bldc_motor1;
/******************************************************************************************/
/*霍尔传感器接口*/

#define HALL1_TIM_CH1_PIN           GPIO_PIN_10     /* U */
#define HALL1_TIM_CH1_GPIO          GPIOH
#define HALL1_U_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PC口时钟使能 */

#define HALL1_TIM_CH2_PIN           GPIO_PIN_7      /* V */
#define HALL1_TIM_CH2_GPIO          GPIOC
#define HALL1_V_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)    /* PC口时钟使能 */

#define HALL1_TIM_CH3_PIN           GPIO_PIN_6      /* W */
#define HALL1_TIM_CH3_GPIO          GPIOC
#define HALL1_W_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)    /* PC口时钟使能 */

/********************************************************************************************/
#define MAX_PWM_DUTY    (((10000) - 1)*0.96)        /* 最大占空比限制 */

#define H_PWM_L_ON
#ifndef H_PWM_L_ON
#define H_PWM_L_PWM
#endif


#define RUN                         (1)
#define STOP                        (0)

#define CW                          (0) /* 正转 */
#define CCW                         (1)     /* 反转 */

#define SPEED_COEFF      (uint32_t)((18000/4)*60) /* 旋转一圈变化4个信号，2对级永磁体特性，NSNS共4级数*/
#define FirstOrderRC_LPF(Yn_1,Xn,a) Yn_1 = (1-a)*Yn_1 + a*Xn; //Yn:out;Xn:in;a:系数

typedef void(*pctr) (void);

void stop_motor1(void);
void start_motor1(void);

/******************************************************************************************/
/* 外部接口函数*/
void bldc_init(uint16_t arr, uint16_t psc);   /* 初始化 */
extern pctr pfunclist_m1[6];
void bldc_ctrl(uint8_t motor_id,int32_t dir,float duty);
void hall_gpio_init(void);
uint32_t hallsensor_get_state(uint8_t motor_id);
uint8_t uemf_edge(uint8_t val);

void m1_uhvl(void);
void m1_uhwl(void);
void m1_vhwl(void);
void m1_vhul(void);
void m1_whul(void);
void m1_whvl(void);

#endif
