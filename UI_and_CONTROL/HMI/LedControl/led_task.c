/* 包含头文件 ----------------------------------------------------------------*/
#include "led_task.h"
#include "../../../BSP/BLDC_PMSM_CONTROL/f407_bldc_pinmap.h"

volatile u16 LedTaskId = 10;
volatile u16 LedTaskTim = 0;

/**
 * 函数功能: LED闪烁任务（周期性翻转LED状态）
 * 输入参数: 无（使用全局变量 LedTaskId、LedTaskTim）
 * 返回参数: 无
 * 说    明: 
 *         1. 该函数应在固定的时间中断中调用（例如每 100us 或 1ms 一次），
 *            通过状态机实现周期性翻转 LED，产生闪烁效果。
 *         2. 状态流程：
 *            - 状态 10：延时等待阶段。每次调用需确保 LedTaskTim 在外部中断中累加，
 *              当 LedTaskTim >= 1000 时（即达到设定延时），复位计时器并切换到状态 20。
 *            - 状态 20：执行 LED 翻转（改变亮灭状态），然后立即切回状态 10，开始下一轮等待。
 *         3. 时间计算：若中断周期为 T（例如 100us），则 LedTaskTim 每 T 时间加 1，
 *            阈值 1000 对应总延时 1000 * T。当 T=100us 时，1000 次 = 100ms，即每 100ms 翻转一次 LED，
 *            因此闪烁周期为 200ms（亮100ms，灭100ms）。
 *         4. 注意：本代码中 LedTaskTim 的累加未在此函数内实现，需确保外部（如定时器中断）调用本函数前已对 LedTaskTim 递增，
 *            或者本函数应包含自增逻辑。当前版本仅作状态判断，实际使用时可能需要修改。
 *         5. 引脚说明：LED1 -> PB4，LED2 -> PB5。使用 HAL_GPIO_TogglePin 实现电平翻转。
 */
void Led_Task(void)
{
    switch (LedTaskId)
    {
        case 10:    // 延时等待状态
        {
            if (LedTaskTim >= 1000)        // 计时达到 1000 个周期（例如 100ms）
            {
                LedTaskTim = 0;            // 计时器清零
                LedTaskId = 20;            // 切换到翻转状态
            }
        }
        break;

        case 20:    // 执行LED翻转状态
        {
            HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN);    // 翻转 LED1
            HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_GPIO_PIN);    // 翻转 LED2
            LedTaskId = 10;                           // 返回延时等待状态
        }
        break;

        default:
            break;
    }
}

