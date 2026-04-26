/**
 ****************************************************************************************************
 * @file        main.c
 * @brief       无刷直流电机(BLDC) — 480x800 LCD；DRV8301 SPI 区 + 触摸/按键与 TI 流程一致
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/TOUCH/touch.h"
#include "./BSP/KEY/key.h"
#include "./BSP/TIMER/btim.h"
#include "./BSP/BLDC/bldc.h"
#include "./BSP/TIMER/bldc_tim.h"
#include "./BSP/DRV8301/drv8301.h"
#include "./BSP/DRV8301/drv8301_ui.h"

#include <stdio.h>
#include <string.h>

/* 竖屏 480 x 800：顶三行 + speed + DRV8301 区 + pwm_duty + 底栏（与线框图一致） */
#define UI_TOP_ROW_H     48U
#define UI_SPEED_Y       (UI_TOP_ROW_H * 3U) /* 144 */
#define UI_SPEED_H       56U
#define UI_MID_Y         (UI_SPEED_Y + UI_SPEED_H) /* 200 */
#define UI_MID_H         400U
#define UI_SPI_BODY_TOP  (UI_MID_Y + 1U)
#define UI_SPI_BODY_H    (UI_MID_H - 2U)
#define UI_PWM_Y         (UI_MID_Y + UI_MID_H) /* 600 */
#define UI_PWM_ROW_H     48U
#define UI_BTN_Y         (UI_PWM_Y + UI_PWM_ROW_H) /* 648 */
#define UI_BTN_H         (800U - UI_BTN_Y)

#define UI_PAD_X         10U
#define UI_VAL_X         200U
/* PWM 触摸/条：自左缘 0 起 400 像素对应 0~100%，x>=400 均为 100% */
#define UI_PWM_FULL_PX   400U

static int16_t s_pwm_duty_temp = 1000;
static void bldc_apply_pwm_from_temp(void);

/* 屏显 0~100% 对应实际 CCR 上限 = ARR*96/100，与 bldc.h 中 MAX_PWM_DUTY 思路一致 */
static uint16_t ui_pwm_max_compare(void)
{
    uint16_t per = g_bldc_timx_handle.Init.Period;
    return (uint16_t)(((uint32_t)per * 96U) / 100U);
}

static uint8_t ui_pwm_percent_from_touch_x(uint16_t tx)
{
    if (tx >= UI_PWM_FULL_PX)
        return 100U;
    return (uint8_t)(((uint32_t)tx * 100U + (UI_PWM_FULL_PX / 2U)) / (uint32_t)UI_PWM_FULL_PX);
}

static uint8_t ui_percent_from_pwm_raw(uint16_t raw)
{
    uint16_t maxc = ui_pwm_max_compare();
    if (maxc == 0U)
        return 0;
    uint32_t p = ((uint32_t)raw * 100U + maxc / 2U) / (uint32_t)maxc;
    if (p > 100U)
        p = 100U;
    return (uint8_t)p;
}

static void bldc_set_pwm_duty_percent(uint8_t pct)
{
    uint16_t maxc = ui_pwm_max_compare();
    if (maxc == 0U)
        return;
    if (pct > 100U)
        pct = 100;
    uint16_t raw = (uint16_t)(((uint32_t)pct * (uint32_t)maxc + 50U) / 100U);
    if (raw > maxc)
        raw = maxc;

    if (g_bldc_motor1.dir == CCW)
        s_pwm_duty_temp = -(int16_t)raw;
    else
        s_pwm_duty_temp = (int16_t)raw;
    bldc_apply_pwm_from_temp();
}

/* lcd_show_string 中英文字符宽为 size/2 */
static uint16_t ui_ascii_str_width(uint8_t size, const char *s)
{
    return (uint16_t)(strlen(s) * (size / 2U));
}

static void ui_show_hcenter(uint16_t y, uint8_t size, char *s, uint16_t color)
{
    uint16_t tw = ui_ascii_str_width(size, s);
    uint16_t x = (lcddev.width > tw) ? (uint16_t)((lcddev.width - tw) / 2U) : 0U;
    lcd_show_string(x, y, (uint16_t)(lcddev.width - x), (uint16_t)(size + 4U), size, s, color);
}

static void ui_show_hcenter_in(uint16_t x0, uint16_t box_w, uint16_t y, uint8_t size, char *s, uint16_t color)
{
    uint16_t tw = ui_ascii_str_width(size, s);
    uint16_t x = (box_w > tw) ? (uint16_t)(x0 + (box_w - tw) / 2U) : x0;
    lcd_show_string(x, y, (uint16_t)(x0 + box_w - x), (uint16_t)(size + 4U), size, s, color);
}

static void ui_draw_panel_borders(void)
{
    uint16_t w = lcddev.width;
    uint16_t y;
    uint16_t i;
    uint16_t half = (uint16_t)(w / 2U);

    lcd_clear(WHITE);

    for (i = 0; i < 3U; i++)
    {
        y = (uint16_t)(UI_TOP_ROW_H * i);
        lcd_draw_rectangle(0, y, (uint16_t)(w - 1U), (uint16_t)(y + UI_TOP_ROW_H - 1U), BLACK);
    }

    lcd_draw_rectangle(0, UI_SPEED_Y, (uint16_t)(w - 1U), (uint16_t)(UI_SPEED_Y + UI_SPEED_H - 1U), BLACK);

    lcd_draw_rectangle(0, UI_MID_Y, (uint16_t)(w - 1U), (uint16_t)(UI_MID_Y + UI_MID_H - 1U), BLACK);
    /* DRV8301 区内 5 行双列网格在 drv8301_ui_draw_panel 中与数据一并绘制 */

    lcd_draw_rectangle(0, UI_PWM_Y, (uint16_t)(w - 1U), (uint16_t)(UI_PWM_Y + UI_PWM_ROW_H - 1U), BLACK);

    lcd_draw_rectangle(0, UI_BTN_Y, (uint16_t)(w - 1U), 799U, BLACK);
    lcd_draw_line(half, UI_BTN_Y, half, 799U, BLACK);

    /* 顶栏：标签左对齐（示意图拼写 motor_diretion） */
    lcd_show_string(UI_PAD_X, (uint16_t)(UI_TOP_ROW_H * 0U + 12U), 460U, 24, 24, "motor_state:", BLACK);
    lcd_show_string(UI_PAD_X, (uint16_t)(UI_TOP_ROW_H * 1U + 12U), 460U, 24, 24, "motor_diretion:", BLACK);
    lcd_show_string(UI_PAD_X, (uint16_t)(UI_TOP_ROW_H * 2U + 12U), 460U, 24, 24, "hall_state:", BLACK);

    lcd_fill(1U, (uint16_t)(UI_MID_Y + 1U), (uint16_t)(w - 2U), (uint16_t)(UI_MID_Y + UI_MID_H - 2U), WHITE);

    lcd_fill(1U, (uint16_t)(UI_SPEED_Y + 1U), (uint16_t)(w - 2U), (uint16_t)(UI_SPEED_Y + UI_SPEED_H - 2U), WHITE);

    lcd_fill(1U, (uint16_t)(UI_PWM_Y + 1U), (uint16_t)(w - 2U), (uint16_t)(UI_PWM_Y + UI_PWM_ROW_H - 2U), WHITE);
    lcd_show_string(UI_PAD_X, (uint16_t)(UI_PWM_Y + 12U), 460U, 24, 24, "pwm_duty:", BLACK);

    lcd_fill(0, UI_BTN_Y + 1U, (uint16_t)(half - 1U), 798U, LGRAY);
    lcd_fill((uint16_t)(half + 1U), UI_BTN_Y + 1U, (uint16_t)(w - 1U), 798U, LGRAY);

    ui_show_hcenter_in(0, half, (uint16_t)(UI_BTN_Y + (UI_BTN_H - 24U) / 2U), 24, "start/stop", BLACK);
    ui_show_hcenter_in(half, half, (uint16_t)(UI_BTN_Y + (UI_BTN_H - 24U) / 2U), 24, "cw/ccw", BLACK);
}

static void ui_row_clear_value(uint16_t y_top, uint16_t h)
{
    lcd_fill(UI_VAL_X, (uint16_t)(y_top + 4U), (uint16_t)(lcddev.width - 4U), (uint16_t)(y_top + h - 5U), WHITE);
}

/* pwm_duty 行：左缘起 400px 为满量程，之后为 100%；淡蓝条与触摸同一比例 */
static void ui_pwm_row_refresh(void)
{
    char buf[32];
    uint16_t w = lcddev.width;
    uint16_t sx = 1U;
    uint16_t ex_full = (uint16_t)(w - 2U);
    uint16_t sy = (uint16_t)(UI_PWM_Y + 1U);
    uint16_t ey = (uint16_t)(UI_PWM_Y + UI_PWM_ROW_H - 2U);
    uint8_t show_pct = ui_percent_from_pwm_raw(g_bldc_motor1.pwm_duty);

    lcd_fill(sx, sy, ex_full, ey, WHITE);
    if (show_pct > 0U)
    {
        uint16_t fill_ex;
        if (show_pct >= 100U)
        {
            fill_ex = ex_full;
        }
        else
        {
            uint32_t px = ((uint32_t)show_pct * (uint32_t)UI_PWM_FULL_PX + 50U) / 100U;
            if (px == 0U)
                px = 1U;
            fill_ex = (uint16_t)(sx + px - 1U);
            if (fill_ex > ex_full)
                fill_ex = ex_full;
        }
        lcd_fill(sx, sy, fill_ex, ey, LIGHTBLUE);
    }
    lcd_show_string(UI_PAD_X, (uint16_t)(UI_PWM_Y + 12U), 180U, 24, 24, "pwm_duty:", BLACK);
    sprintf(buf, "%u%%", (unsigned)show_pct);
    lcd_show_string(UI_VAL_X, (uint16_t)(UI_PWM_Y + 12U), (uint16_t)(w - UI_VAL_X), 24, 24, buf, BLACK);
}

static void bldc_lcd_refresh(void)
{
    char buf[56];
    uint32_t hall;
    uint16_t w = lcddev.width;

    /* 顶栏数值 */
    ui_row_clear_value((uint16_t)(UI_TOP_ROW_H * 0U), UI_TOP_ROW_H);
    sprintf(buf, "%s", (g_bldc_motor1.run_flag == RUN) ? "RUN" : "STOP");
    lcd_show_string(UI_VAL_X, (uint16_t)(UI_TOP_ROW_H * 0U + 12U), 260U, 24, 24, buf,
                    (g_bldc_motor1.run_flag == RUN) ? GREEN : RED);

    ui_row_clear_value((uint16_t)(UI_TOP_ROW_H * 1U), UI_TOP_ROW_H);
    sprintf(buf, "%s", (g_bldc_motor1.dir == CW) ? "CW" : "CCW");
    lcd_show_string(UI_VAL_X, (uint16_t)(UI_TOP_ROW_H * 1U + 12U), 260U, 24, 24, buf, BLUE);

    ui_row_clear_value((uint16_t)(UI_TOP_ROW_H * 2U), UI_TOP_ROW_H);
    hall = hallsensor_get_state(MOTOR_1);
    sprintf(buf, "raw:%lu step:%u", (unsigned long)hall, (unsigned)g_bldc_motor1.step_sta);
    lcd_show_string(UI_VAL_X, (uint16_t)(UI_TOP_ROW_H * 2U + 8U), 280U, 24, 16, buf, BLACK);

    /* speed 行：整行居中刷新（线框要求该行文案居中） */
    lcd_fill(1U, (uint16_t)(UI_SPEED_Y + 1U), (uint16_t)(w - 2U), (uint16_t)(UI_SPEED_Y + UI_SPEED_H - 2U), WHITE);
    sprintf(buf, "speed: %.1f RPM", (float)g_bldc_motor1.speed);
    ui_show_hcenter((uint16_t)(UI_SPEED_Y + (UI_SPEED_H - 24U) / 2U), 24, buf, BLUE);
    /* 触摸调速：左半区减速、右半区加速（与原先 KEY 逻辑一致） */
    lcd_show_string(UI_PAD_X, (uint16_t)(UI_SPEED_Y + UI_SPEED_H - 20U), 200U, 16, 12, "<-", DARKBLUE);
    lcd_show_string((uint16_t)(w - 30U), (uint16_t)(UI_SPEED_Y + UI_SPEED_H - 20U), 40U, 16, 12, "->", DARKBLUE);

    /* 先刷新 PWM/速度等电机相关显示，再画 SPI 区（SPI 读可能被节流） */
    ui_pwm_row_refresh();
    drv8301_ui_draw_panel(&drv8301, UI_SPI_BODY_TOP, UI_SPI_BODY_H, lcddev.width);
}

static void bldc_speed_delta(int16_t delta)
{
    if (g_bldc_motor1.run_flag != RUN)
        return;

    if (delta > 0)
    {
        s_pwm_duty_temp += 500;
        if (s_pwm_duty_temp >= (int16_t)(MAX_PWM_DUTY / 2))
            s_pwm_duty_temp = (int16_t)(MAX_PWM_DUTY / 2);
    }
    else
    {
        s_pwm_duty_temp -= 500;
        if (s_pwm_duty_temp <= -(int16_t)(MAX_PWM_DUTY / 2))
            s_pwm_duty_temp = -(int16_t)(MAX_PWM_DUTY / 2);
    }

    if (s_pwm_duty_temp > 0)
    {
        g_bldc_motor1.pwm_duty = (uint16_t)s_pwm_duty_temp;
        g_bldc_motor1.dir = CW;
    }
    else
    {
        g_bldc_motor1.pwm_duty = (uint16_t)(-s_pwm_duty_temp);
        g_bldc_motor1.dir = CCW;
    }
}

static void bldc_apply_pwm_from_temp(void)
{
    if (s_pwm_duty_temp > 0)
    {
        g_bldc_motor1.pwm_duty = (uint16_t)s_pwm_duty_temp;
        g_bldc_motor1.dir = CW;
    }
    else if (s_pwm_duty_temp < 0)
    {
        g_bldc_motor1.pwm_duty = (uint16_t)(-s_pwm_duty_temp);
        g_bldc_motor1.dir = CCW;
    }
    else
    {
        g_bldc_motor1.pwm_duty = 0;
    }
}

static void bldc_key_start(void)
{
    bldc_apply_pwm_from_temp();
    if (g_bldc_motor1.pwm_duty == 0U)
    {
        s_pwm_duty_temp = 1000;
        bldc_apply_pwm_from_temp();
    }
    g_bldc_motor1.run_flag = RUN;
    start_motor1();
}

static void bldc_key_stop(void)
{
    stop_motor1();
    g_bldc_motor1.run_flag = STOP;
    g_bldc_motor1.pwm_duty = 0;
    g_bldc_motor1.speed = 0;
    s_pwm_duty_temp = 1000;
}

static void bldc_toggle_run(void)
{
    if (g_bldc_motor1.run_flag == RUN)
        bldc_key_stop();
    else
        bldc_key_start();
}

static void bldc_toggle_dir(void)
{
    if (s_pwm_duty_temp == 0)
        s_pwm_duty_temp = 1000;
    else
        s_pwm_duty_temp = (int16_t)(-s_pwm_duty_temp);

    if (g_bldc_motor1.run_flag == RUN)
        bldc_apply_pwm_from_temp();
}

int main(void)
{
    HAL_Init();
    /* HSE=25MHz: PLLM=25(N分频后1MHz进VCO), N=336, P=2 -> SYSCLK=168MHz; Q=7 -> 48MHz */
    sys_stm32_clock_init(336, 25, 2, 7);
    delay_init(168);
    usart_init(115200);
    led_init();
    lcd_init();
    tp_init();
    key_init();
    btim_timx_int_init(1000, 84 - 1);

    bldc_init(168000 / 18 - 1, 0);
    bldc_ctrl(MOTOR_1, CW, 0);
    HAL_TIM_Base_Start_IT(&g_bldc_timx_handle);

    drv8301_ui_handle_init(&drv8301);

    ui_draw_panel_borders();
    bldc_lcd_refresh();

    while (1)
    {
        static uint8_t disp_div;
        static uint8_t s_touch_was_down;
        static uint32_t s_ui_btn_last_ms;

        /* 物理按键：DRV8301 区 — KEY0=确认、KEY1=减、KEY2=加（与 TI 一致） */
        {
            uint8_t key = key_scan(0);
            if (key != 0U)
            {
                uint8_t was_spi = drv8301_ui_is_active(&drv8301);
                (void)drv8301_ui_process_key(&drv8301, key);
                if (was_spi || drv8301_ui_is_active(&drv8301))
                    drv8301_ui_draw_panel(&drv8301, UI_SPI_BODY_TOP, UI_SPI_BODY_H, lcddev.width);
            }
        }

        /* 触摸屏：仅在「按下瞬间」处理一次。
         * 注意：GT9xxx 的 scan() 返回值 res 在节流周期内常为 0，但 tp_dev.sta 仍保持按下位；
         * 若用返回值当按下态，会与真实 sta 不同步，每圈都误判一次「松手再按下」→ 连点 toggle。 */
        {
            (void)tp_dev.scan(0);
            uint8_t touch_down =
                (uint8_t)(((tp_dev.sta & TP_PRES_DOWN) != 0U) ? 1U : 0U);
            uint8_t touch_edge = (uint8_t)(touch_down && (s_touch_was_down == 0U));
            s_touch_was_down = touch_down;

            if (touch_edge)
        {
            uint16_t tx = tp_dev.x[0];
            uint16_t ty = tp_dev.y[0];
            uint16_t w = lcddev.width;
            uint16_t half = (uint16_t)(w / 2U);

            if (tx < w && ty >= UI_SPEED_Y && ty < (uint16_t)(UI_SPEED_Y + UI_SPEED_H))
            {
                if (drv8301_ui_is_active(&drv8301))
                    drv8301_ui_abort(&drv8301);
                if (tx < half)
                    bldc_speed_delta(-500);
                else
                    bldc_speed_delta(+500);
                bldc_lcd_refresh();
            }
            else if (tx < w && ty >= UI_PWM_Y && ty < (UI_PWM_Y + UI_PWM_ROW_H))
            {
                if (drv8301_ui_is_active(&drv8301))
                    drv8301_ui_abort(&drv8301);
                bldc_set_pwm_duty_percent(ui_pwm_percent_from_touch_x(tx));
                ui_pwm_row_refresh();
            }
            else if (tx < w && ty >= UI_BTN_Y && ty <= 799U)
            {
                uint32_t now = HAL_GetTick();
                /* 底栏防抖：触摸 IC 偶发连边沿时，避免 RUN/STOP 被连切两次 */
                if ((uint32_t)(now - s_ui_btn_last_ms) >= 350U)
                {
                    s_ui_btn_last_ms = now;
                    if (drv8301_ui_is_active(&drv8301))
                        drv8301_ui_abort(&drv8301);
                    if (tx < half)
                        bldc_toggle_run();
                    else
                        bldc_toggle_dir();
                    bldc_lcd_refresh();
                }
            }
            else if (tx < w && ty >= UI_SPI_BODY_TOP && ty < (uint16_t)(UI_SPI_BODY_TOP + UI_SPI_BODY_H))
            {
                drv8301_ui_touch_select(&drv8301, tx, ty, UI_SPI_BODY_TOP, UI_SPI_BODY_H, w);
                drv8301_ui_draw_panel(&drv8301, UI_SPI_BODY_TOP, UI_SPI_BODY_H, lcddev.width);
            }
        }
        }

        if (++disp_div >= 8)
        {
            disp_div = 0;
            bldc_lcd_refresh();
        }

        static uint8_t led_div;
        if (++led_div >= 20)
        {
            led_div = 0;
            LED0_TOGGLE();
        }
        delay_ms(25);
    }
}
