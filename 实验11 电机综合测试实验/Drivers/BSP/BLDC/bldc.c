/**
 ****************************************************************************************************
 * @file        bldc.c
 * @author      ALIENTEK
 * @version     V1.0
 * @date        2021-10-14
 * @brief       BLDC motor driver (TIM8 six-step, complementary low-side on PH13-15)
 ****************************************************************************************************
 */

#include "./BSP/BLDC/bldc.h"
#include "./BSP/TIMER/bldc_tim.h"

#define M1_CCER_OUT_MASK  (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)

static uint16_t m1_lowside_full_on_ccr(void)
{
    return (uint16_t)(g_bldc_timx_handle.Instance->ARR + 1U);
}

static void m1_apply_outputs(uint32_t ccer_on, uint16_t c1, uint16_t c2, uint16_t c3)
{
    TIM_TypeDef *T = g_bldc_timx_handle.Instance;
    T->CCR1 = c1;
    T->CCR2 = c2;
    T->CCR3 = c3;
    uint32_t ce = T->CCER;
    ce &= ~M1_CCER_OUT_MASK;
    ce |= (ccer_on & M1_CCER_OUT_MASK);
    T->CCER = ce;
}

_bldc_obj g_bldc_motor1 = {STOP,0,0,CCW,0,0,0,0,0,0};

void bldc_init(uint16_t arr, uint16_t psc)
{
    hall_gpio_init();
    bldc_timx_oc_chy_init(arr,  psc);
}

void bldc_ctrl(uint8_t motor_id,int32_t dir,float duty)
{
    if(motor_id == MOTOR_1)
    {
        g_bldc_motor1.dir = dir;
        g_bldc_motor1.pwm_duty = duty;
    }
}

void hall_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    HALL1_U_GPIO_CLK_ENABLE();
    HALL1_V_GPIO_CLK_ENABLE();
    HALL1_W_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin = HALL1_TIM_CH1_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HALL1_TIM_CH1_GPIO, &gpio_init_struct);

    gpio_init_struct.Pin = HALL1_TIM_CH2_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH2_GPIO, &gpio_init_struct);

    gpio_init_struct.Pin = HALL1_TIM_CH3_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH3_GPIO, &gpio_init_struct);
}

uint32_t hallsensor_get_state(uint8_t motor_id)
{
    __IO static uint32_t State ;
    State  = 0;
    if(motor_id == MOTOR_1)
    {
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH1_GPIO,HALL1_TIM_CH1_PIN) != GPIO_PIN_RESET)
        {
            State |= 0x01U;
        }
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH2_GPIO,HALL1_TIM_CH2_PIN) != GPIO_PIN_RESET)
        {
            State |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH3_GPIO,HALL1_TIM_CH3_PIN) != GPIO_PIN_RESET)
        {
            State |= 0x04U;
            g_bldc_motor1.hall_single_sta=1;
        }
        else
            g_bldc_motor1.hall_single_sta=0;
    }
    return State;
}

void stop_motor1(void)
{
    TIM_TypeDef *T = g_bldc_timx_handle.Instance;
    T->CCR1 = 0;
    T->CCR2 = 0;
    T->CCR3 = 0;
    T->CCER &= ~(uint32_t)M1_CCER_OUT_MASK;
    __HAL_TIM_MOE_DISABLE(&g_bldc_timx_handle);
    __HAL_TIM_DISABLE(&g_bldc_timx_handle);
    g_bldc_motor1.step_last = 0U;
    g_bldc_motor1.step_sta = 0U;
}

void start_motor1(void)
{
    g_bldc_motor1.step_last = 0U;
    bldc_motor1_pwm_commit_and_kick();
}

pctr pfunclist_m1[6] =
{
    /* Ported from DRV8353Apply1.0 six-step sequence. */
    &m1_uhwl, &m1_vhul, &m1_vhwl,
    &m1_whvl, &m1_uhvl, &m1_whul
};

/* TIM8: PI5/6/7 high CH1/2/3, PH13/14/15 low CH1N/2N/3N; H_PWM_L_ON */
void m1_uhvl(void)
{
    uint16_t d = (uint16_t)g_bldc_motor1.pwm_duty;
    uint16_t f = m1_lowside_full_on_ccr();
    m1_apply_outputs(TIM_CCER_CC1E | TIM_CCER_CC2NE, d, f, 0);
}

void m1_uhwl(void)
{
    uint16_t d = (uint16_t)g_bldc_motor1.pwm_duty;
    uint16_t f = m1_lowside_full_on_ccr();
    m1_apply_outputs(TIM_CCER_CC1E | TIM_CCER_CC3NE, d, 0, f);
}

void m1_vhwl(void)
{
    uint16_t d = (uint16_t)g_bldc_motor1.pwm_duty;
    uint16_t f = m1_lowside_full_on_ccr();
    m1_apply_outputs(TIM_CCER_CC2E | TIM_CCER_CC3NE, 0, d, f);
}

void m1_vhul(void)
{
    uint16_t d = (uint16_t)g_bldc_motor1.pwm_duty;
    uint16_t f = m1_lowside_full_on_ccr();
    m1_apply_outputs(TIM_CCER_CC2E | TIM_CCER_CC1NE, f, d, 0);
}

void m1_whul(void)
{
    uint16_t d = (uint16_t)g_bldc_motor1.pwm_duty;
    uint16_t f = m1_lowside_full_on_ccr();
    m1_apply_outputs(TIM_CCER_CC3E | TIM_CCER_CC1NE, f, 0, d);
}

void m1_whvl(void)
{
    uint16_t d = (uint16_t)g_bldc_motor1.pwm_duty;
    uint16_t f = m1_lowside_full_on_ccr();
    m1_apply_outputs(TIM_CCER_CC3E | TIM_CCER_CC2NE, 0, f, d);
}

uint8_t uemf_edge(uint8_t val)
{
    static uint8_t oldval=0;
    if(oldval != val)
    {
        oldval = val;
        if(val == 0) return 0;
        else return 1;
    }
    return 2;
}

