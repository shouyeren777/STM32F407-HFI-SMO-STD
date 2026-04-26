/**
 ****************************************************************************************************
 * @file        bldc_tim.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-19
 * @brief       定时器 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211019
 * 第一次发布
 *
 ****************************************************************************************************
 */


#include "./BSP/TIMER/bldc_tim.h"
#include "./BSP/BLDC/bldc.h"
/******************************************************************************************/
/* 定时器配置句柄 定义 */

/* 高级定时器PWM */
TIM_HandleTypeDef g_bldc_timx_handle;      /* 定时器x句柄 */
TIM_OC_InitTypeDef g_bldc_time_handle;  /* 定时器输出句柄 */
extern _bldc_obj g_bldc_motor1;

/******************************************************************************************/
void bldc_io_init(void);
/**
 * @brief       高级定时器TIMX PWM输出初始化函数
 * @note
 *              高级定时器的时钟来自APB2, 而PCLK2 = 168Mhz, 我们设置PPRE2不分频, 因此
 *              高级定时器时钟 = 168Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void bldc_timx_oc_chy_init(uint16_t arr, uint16_t psc)
{
    BLDC_TIMX_PWM_CHY_CLK_ENABLE();                             /* TIMX 时钟使能 */


    g_bldc_timx_handle.Instance = BLDC_TIMX_PWM;                    /* 定时器x */
    g_bldc_timx_handle.Init.Prescaler = psc;                        /* 定时器分频 */
    g_bldc_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 向上计数模式 */
    g_bldc_timx_handle.Init.Period = arr;                           /* 自动重装载值 */
    g_bldc_timx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* 分频因子 */
    g_bldc_timx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /*使能TIMx_ARR进行缓冲*/
    g_bldc_timx_handle.Init.RepetitionCounter = 0;                  /* 开始时不计数*/
    HAL_TIM_PWM_Init(&g_bldc_timx_handle);                          /* 初始化PWM */
    bldc_io_init();
    g_bldc_time_handle.OCMode = TIM_OCMODE_PWM1;             /* 模式选择PWM1 */
    g_bldc_time_handle.Pulse = 0;
    g_bldc_time_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* 输出比较极性为低 */
    g_bldc_time_handle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    g_bldc_time_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_bldc_time_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_bldc_time_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_bldc_timx_handle, &g_bldc_time_handle, BLDC_TIMX_PWM_CH1); /* 配置TIMx通道y */
    HAL_TIM_PWM_ConfigChannel(&g_bldc_timx_handle, &g_bldc_time_handle, BLDC_TIMX_PWM_CH2); /* 配置TIMx通道y */
    HAL_TIM_PWM_ConfigChannel(&g_bldc_timx_handle, &g_bldc_time_handle, BLDC_TIMX_PWM_CH3); /* 配置TIMx通道y */

    /* ConfigChannel 会打开 OCxPE：CCR 仅预装载，须等 UEV 才到影子寄存器；换相里直接写 CCRx 则引脚可能一直无 PWM */
    g_bldc_timx_handle.Instance->CCMR1 &= (uint32_t)~(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    g_bldc_timx_handle.Instance->CCMR2 &= (uint32_t)~TIM_CCMR2_OC3PE;

    /* 死区 DTG（8bit），量产可按 MOSFET/驱动器数据手册微调 */
    {
        uint32_t bdtr = g_bldc_timx_handle.Instance->BDTR;
        bdtr = (bdtr & ~(uint32_t)0xFFU) | (uint32_t)32U;
        g_bldc_timx_handle.Instance->BDTR = bdtr;
    }

    /* 不在此处启动 PWM：由 start_motor1() 统一启动，避免通道已 BUSY 时再次 Start 返回 HAL_ERROR */
    stop_motor1(); /* 关驱动、CCR 清零，与上电停机状态一致 */
}

/**
 * @brief  打开 MOE/计数器，并按当前霍尔执行一次六步换向（TIM8 六路由 CCER/CCR 控制，不调用 HAL_PWM_Start 以免全开 CCxE）
 */
void bldc_motor1_pwm_commit_and_kick(void)
{
    uint8_t step;

    __HAL_TIM_MOE_ENABLE(&g_bldc_timx_handle);
    __HAL_TIM_ENABLE(&g_bldc_timx_handle);

    if (g_bldc_motor1.dir == CW)
        step = (uint8_t)hallsensor_get_state(MOTOR_1);
    else
        step = (uint8_t)(7U - hallsensor_get_state(MOTOR_1));
    if (step >= 1U && step <= 6U)
        pfunclist_m1[step - 1U]();

    /* 确保 BDTR 主输出使能（部分板在 Break 管脚悬空时需关闭 BKE，此处保持默认并强制 MOE） */
    g_bldc_timx_handle.Instance->BDTR &= (uint32_t)~TIM_BDTR_BKE;
    __HAL_TIM_MOE_ENABLE(&g_bldc_timx_handle);
    /* 产生一次更新，装载 ARR/CCR 影子寄存器（若仍有预装载相关位） */
    g_bldc_timx_handle.Instance->EGR = TIM_EGR_UG;
}


/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void bldc_io_init(void)
{
        GPIO_InitTypeDef gpio_init_struct;
        BLDC_TIMX_PWM_CHY_CLK_ENABLE();

        BLDC_TIMX_PWM_CH1_GPIO_CLK_ENABLE();
        BLDC_TIMX_PWM_CH2_GPIO_CLK_ENABLE();
        BLDC_TIMX_PWM_CH3_GPIO_CLK_ENABLE();
        BLDC_TIMX_PWM_CHXN_GPIO_CLK_ENABLE();

        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = BLDC_TIMX_PWM_CHY_GPIO_AF;

        gpio_init_struct.Pin = BLDC_TIMX_PWM_CH1_GPIO_PIN;
        HAL_GPIO_Init(BLDC_TIMX_PWM_CH1_GPIO_PORT, &gpio_init_struct);
        gpio_init_struct.Pin = BLDC_TIMX_PWM_CH2_GPIO_PIN;
        HAL_GPIO_Init(BLDC_TIMX_PWM_CH2_GPIO_PORT, &gpio_init_struct);
        gpio_init_struct.Pin = BLDC_TIMX_PWM_CH3_GPIO_PIN;
        HAL_GPIO_Init(BLDC_TIMX_PWM_CH3_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = BLDC_TIMX_PWM_CH1N_GPIO_PIN;
        HAL_GPIO_Init(BLDC_TIMX_PWM_CH1N_GPIO_PORT, &gpio_init_struct);
        gpio_init_struct.Pin = BLDC_TIMX_PWM_CH2N_GPIO_PIN;
        HAL_GPIO_Init(BLDC_TIMX_PWM_CH2N_GPIO_PORT, &gpio_init_struct);
        gpio_init_struct.Pin = BLDC_TIMX_PWM_CH3N_GPIO_PIN;
        HAL_GPIO_Init(BLDC_TIMX_PWM_CH3N_GPIO_PORT, &gpio_init_struct);

        HAL_NVIC_SetPriority(BLDC_TIMX_PWM_IRQn, 2, 2);
        HAL_NVIC_EnableIRQ(BLDC_TIMX_PWM_IRQn);
}


void TIM8_UP_TIM13_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_bldc_timx_handle);
}
/**
 * @brief       定时器中断回调
 * @param       无
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    int16_t temp_speed=0;                           /* 临时速度存储 */
    static uint8_t s_tim8_run_latched;
    if(htim->Instance == BLDC_TIMX_PWM)             /* 55us */
    {
#ifdef H_PWM_L_ON
        if(g_bldc_motor1.run_flag == RUN)
        {
            if (s_tim8_run_latched == 0U)
            {
                s_tim8_run_latched = 1U;
                g_bldc_motor1.step_last = 0U;
            }

            /* Ported from DRV8353Apply1.0: periodic Hall sampling + six-step commutation. */
            {
                if(g_bldc_motor1.dir == CW)                   /* 正转 */
                {
                    g_bldc_motor1.step_sta = (uint8_t)hallsensor_get_state(MOTOR_1);     /* 顺序6,2,3,1,5,4 */
                }
                else                                           /* 反转 */
                {
                    g_bldc_motor1.step_sta = (uint8_t)(7U - hallsensor_get_state(MOTOR_1)); /* 顺序5,1,3,2,6,4 */
                }

                if((g_bldc_motor1.step_sta <= 6U) && (g_bldc_motor1.step_sta >= 1U)) /* 霍尔组合值正常 */
                {
                    g_bldc_motor1.step_last = g_bldc_motor1.step_sta;
                    pfunclist_m1[g_bldc_motor1.step_sta - 1U]();
                }
                else                                                            /* 霍尔传感器异常 */
                {
                    stop_motor1();
                    g_bldc_motor1.run_flag = STOP;
                }
            }
            /******************************* 速度计算 *******************************/
            g_bldc_motor1.count_j++;                /* 计算速度专用计数值 */
            g_bldc_motor1.hall_sta_edge = uemf_edge(g_bldc_motor1.hall_single_sta);/* 检测单个霍尔信号的变化 */
            if(g_bldc_motor1.hall_sta_edge == 0)    /* 统计单个霍尔信号的高电平时间，当只有一对级的时候，旋转一圈为一个完整脉冲。一高一低相加即旋转一圈所花的时间*/
            {
                /*计算速度*/
                if(g_bldc_motor1.dir == CW)
                    temp_speed = (SPEED_COEFF/g_bldc_motor1.count_j);
                else
                    temp_speed = -(SPEED_COEFF/g_bldc_motor1.count_j);
                FirstOrderRC_LPF(g_bldc_motor1.speed,temp_speed,0.2379f);   /* 一阶滤波 */
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 1)   
            {
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 2)    /* 霍尔值一直不变代表未换向 */
            {
                g_bldc_motor1.no_single++;          /* 不换相时间累计 超时则判定速度为0 */
                
                if(g_bldc_motor1.no_single > 15000)
                {
                    
                    g_bldc_motor1.no_single = 0;
                    g_bldc_motor1.speed = 0;        /* 超时换向 判定为停止 速度为0 */
                }
            }
            
 }
        else
        {
            s_tim8_run_latched = 0U;
        }

#endif
    }
}


