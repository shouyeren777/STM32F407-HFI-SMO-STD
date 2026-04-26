#include "lcd_mid.h"
#include "motor_system.h"
#include "../../LCD/lcd.h"

#define HMI_W                480U
#define HMI_H                800U
#define HMI_ROW_H            60U
#define HMI_SPEED_TOP        180U
#define HMI_SPEED_BOTTOM     299U
#define HMI_BTN_TOP          740U
#define HMI_BTN_BOTTOM       799U

static uint8_t hmi_inited = 0;

static const char *hmi_state_text(void)
{
    switch (MC.Motor.RunState) {
    case ADC_CALIB:
        return "adc_calib";
    case MOTOR_STOP:
        return "stop";
    case MOTOR_ERROR:
        return "error";
    case MOTOR_IDENTIFY:
        return "identify";
    case MOTOR_SENSORLESS:
        return "run";
    default:
        return "unknown";
    }
}

static const char *hmi_dir_text(void)
{
    return (Speed_Set_Dir >= 0) ? "cw" : "ccw";
}

static void hmi_draw_frame(void)
{
    lcd_clear(WHITE);
    lcd_draw_rectangle(20, 20, 460, 780, BLACK);
    lcd_draw_line(20, 80, 460, 80, BLACK);
    lcd_draw_line(20, 140, 460, 140, BLACK);
    lcd_draw_line(20, 200, 460, 200, BLACK);
    lcd_draw_line(20, 300, 460, 300, BLACK);
    lcd_draw_line(20, 720, 460, 720, BLACK);
    lcd_draw_line(240, 720, 240, 780, BLACK);

    lcd_show_string(36, 40, 220, 24, 24, "motor_state:", BLACK);
    lcd_show_string(36, 100, 260, 24, 24, "motor_diretion:", BLACK);
    lcd_show_string(120, 240, 120, 32, 32, "speed:", BLACK);
    lcd_show_string(70, 742, 150, 32, 32, "start/stop", BLACK);
    lcd_show_string(290, 742, 120, 32, 32, "cw/ccw", BLACK);
}

void LCD_Clear(void)
{
    lcd_clear(WHITE);
}

void LCD_Display_Logo(void)
{
    if (hmi_inited == 0) {
        lcd_init();
        hmi_inited = 1;
    }
    lcd_clear(WHITE);
    lcd_show_string(150, 380, 200, 32, 32, "BLDC FOC", BLUE);
    HAL_Delay(600);
    hmi_draw_frame();
}

void LCD_Display_Page1(void)
{
    char speed_buf[24];

    if (hmi_inited == 0) {
        return;
    }

    lcd_fill(270, 26, 450, 74, WHITE);
    lcd_show_string(272, 40, 170, 24, 24, (char *)hmi_state_text(), BLUE);

    lcd_fill(300, 86, 450, 134, WHITE);
    lcd_show_string(302, 100, 130, 24, 24, (char *)hmi_dir_text(), BLUE);

    lcd_fill(150, 210, 430, 290, WHITE);
    if (MC.Speed.MechanicalSpeed < 0) {
        lcd_show_string(152, 242, 32, 32, 32, "-", RED);
        lcd_show_num(184, 242, (uint32_t)(-MC.Speed.MechanicalSpeed), 5, 32, RED);
    } else {
        lcd_show_num(184, 242, (uint32_t)MC.Speed.MechanicalSpeed, 5, 32, RED);
    }

    (void)speed_buf;
}

void LCD_Display_Page2(void)
{
    LCD_Display_Page1();
}

void HMI_Toggle_RunStop(void)
{
    if (MC.Motor.RunState == MOTOR_SENSORLESS) {
        MC.Motor.RunState = MOTOR_STOP;
    } else if (MC.Motor.RunState == MOTOR_STOP) {
        MC.Motor.RunState = MOTOR_SENSORLESS;
    }
}

void HMI_Toggle_Direction(void)
{
    Speed_Set_Dir = -Speed_Set_Dir;
}
