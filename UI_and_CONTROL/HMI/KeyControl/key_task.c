/* 包含头文件 ----------------------------------------------------------------*/
#include "key_task.h"
#include "key_drv.h"

volatile u16 KeyTaskId = 10;
volatile u16 KeyTaskTim = 0;

/**
 * 函数功能: 按键任务调度（周期性执行按键扫描）
 * 输入参数: 无（使用全局变量 KeyTaskId、KeyTaskTim）
 * 返回参数: 无
 * 说    明: 
 *         1. 该函数应在固定的时间中断中调用（每 100us 一次），
 *            通过状态机实现周期性执行 Key_Scan()，避免主循环中频繁调用。
 *         2. 状态流程：
 *            - 状态 10：延时等待阶段。每次调用 KeyTaskTim 加 1（需外部中断累加），
 *              当 KeyTaskTim >= 100 时（即达到设定延时），复位计时器并切换到状态 20。
 *            - 状态 20：执行一次 Key_Scan() 函数（读取按键并处理状态机），然后立即切回状态 10，开始下一轮等待。
 *         3. 时间计算：若中断周期为 T（例如 100us），则 KeyTaskTim 每 T 时间加 1，
 *            阈值 100 对应总延时 100 * T。通常 T=100us 时，100 次 = 10ms，即每 10ms 扫描一次按键。
 *         4. 注意：本代码中 KeyTaskTim 的累加未在此函数内实现，需确保外部（如定时器中断）调用本函数前已对 KeyTaskTim 递增。
 */
void Key_Task(void)
{
    switch (KeyTaskId)
    {
        case 10:    // 延时等待状态
        {
            if (KeyTaskTim >= 100)        // 计时达到 100 个周期（10ms）
            {
                KeyTaskTim = 0;           // 计时器清零
                KeyTaskId = 20;           // 切换到扫描状态
            }
        }
        break;

        case 20:    // 执行按键扫描状态
        {
            Key_Scan();                   // 调用按键扫描函数（读取电平并更新键值）
            KeyTaskId = 10;               // 返回延时等待状态
        }
        break;

        default:
            break;
    }
}
