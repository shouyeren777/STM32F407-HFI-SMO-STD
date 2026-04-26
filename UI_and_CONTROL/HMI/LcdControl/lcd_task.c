/* 包含头文件 ---------------------------------------------------------------*/
#include "lcd_task.h"
#include "lcd_mid.h"
#include "../../TOUCH/touch.h"

volatile u16 LcdTaskId = 10;
volatile u16 LcdTaskTim = 0;

extern u8 KeyNum;
static u8 hmi_touch_inited = 0;
static u8 hmi_touch_was_down = 0;

static void hmi_handle_key_event(u8 key)
{
    if (key == 0) {
        return;
    }
    if (key == 1 || key == 2 || key == 5 || key == 6) {
        HMI_Toggle_RunStop();
    } else if (key == 3 || key == 4) {
        HMI_Toggle_Direction();
    }

    KeyNum = 0;
}

static void hmi_handle_touch_press(u16 x, u16 y)
{
    if (y >= 720 && y <= 799) {
        if (x < 240) {
            HMI_Toggle_RunStop();
        } else {
            HMI_Toggle_Direction();
        }
    }
}

static void hmi_poll_touch(void)
{
    u8 down;

    if (hmi_touch_inited == 0) {
        (void)tp_init();
        hmi_touch_inited = 1;
    }

    if (tp_dev.scan == 0) {
        return;
    }

    (void)tp_dev.scan(0);
    down = ((tp_dev.sta & TP_PRES_DOWN) != 0U) ? 1U : 0U;
    if (down && (hmi_touch_was_down == 0U)) {
        hmi_handle_touch_press(tp_dev.x[0], tp_dev.y[0]);
    }
    hmi_touch_was_down = down;
}


/**
  * 函数功能: 定时进行lcd的显示任务
  * 输入参数:
  * 返 回 值: 
  * 说    明:
  */
void Lcd_Task(void)
{
    hmi_handle_key_event(KeyNum);
    hmi_poll_touch();

    switch(LcdTaskId)
    {
        case 10:
        {
            if(LcdTaskTim>=1000)        //200ms
            {
                LcdTaskTim = 0;
                LcdTaskId = 20;
            }
        }
        break;
        
        case 20:
        {                
            LCD_Display_Page1();
            LcdTaskId = 10;
        }
        break;

    default:
      break;            
    }
}



