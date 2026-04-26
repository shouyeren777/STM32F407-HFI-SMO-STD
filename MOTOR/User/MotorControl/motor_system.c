/**
  ******************************************************************************
  * 文件名程: 
  * 作    者: 浩盛科技
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/

#include "motor_system.h"

float spend_time = 0;
/**
  * 函数功能:电机系统初始化 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Motor_System_Init(void)
{
    Motor_Struct_Init();                       //结构体参数初始化
}

/**
 * 函数功能: 系统运行主函数（状态机调度）
 * 输入参数: 无（所有参数通过全局结构体 MC、MC.Sample、MC.Identify、MC.SMO、MC.EAngle 等传递）
 * 返回参数: 无
 * 说    明: 
 *         1. 该函数是电机控制系统的顶层调度函数，根据当前运行状态（RunState）执行相应操作。
 *         2. 首先进行 ADC 校准后的安全检查：计算母线电压和三相电流，并进行过压/欠压/过流判断。
 *         3. 然后根据 RunState 进入不同控制模式：
 *            - ADC_CALIB：ADC 偏移量校准
 *            - MOTOR_IDENTIFY：电机参数辨识（电阻、电感）
 *            - MOTOR_SENSORLESS：无感控制（基于观测器）
 *            - MOTOR_ERROR：故障处理（封锁 PWM、关闭驱动）
 *            - MOTOR_STOP：停机状态（输出零占空比）
 *         4. 状态之间可按条件自动切换，例如 ADC 校准完成后进入参数辨识，辨识完成后进入无感控制。
 *         5. 故障状态下会锁定错误码并禁止输出，保护功率器件和电机。
 *         6. 注意：无感控制段中有测量执行时间的代码（TIM6 计数），用于性能分析。
 */
void Motor_System_Run(void)
{
    /* ========== 1. ADC 校准完成后进行电压/电流采样与保护判断 ========== */
    if (MC.Sample.CalibEndFlag == 1)                     // ADC 偏移量已校准（通常在上电初始化时完成）
    {
        // 计算实际母线电压（单位：伏特）
        Calculate_Bus_Voltage(&MC.Sample);
        // 计算三相实际电流（单位：安培）
        Calculate_Phase_Current(&MC.Sample);

        // 设置 d、q 轴电流环 PID 输出限幅（防止输出电压超出母线电压能力）
        MC.IdPid.OutMax =  MC.Sample.BusReal * INV_SQRT3;
        MC.IdPid.OutMin = -MC.Sample.BusReal * INV_SQRT3;
        MC.IqPid.OutMax =  MC.Sample.BusReal * INV_SQRT3;
        MC.IqPid.OutMin = -MC.Sample.BusReal * INV_SQRT3;

        // 母线电压异常检测：正常范围 BUS_VOLTAGE_MIN~BUS_VOLTAGE_MAX（根据实际应用设定）
        if (MC.Sample.BusReal <= BUS_VOLTAGE_MIN || MC.Sample.BusReal >= BUS_VOLTAGE_MAX)
        {
            MC.Motor.RunState = MOTOR_ERROR;        // 切换到故障状态
            MC.Motor.ErrorCode = POWER_VOLT_ERR;    // 电源电压错误代码
        }

        // 过流检测：任一相电流绝对值超过 OVER_CURRENT 即触发过流保护
        if (MC.Sample.IuReal > OVER_CURRENT || MC.Sample.IuReal < -OVER_CURRENT ||
            MC.Sample.IvReal > OVER_CURRENT || MC.Sample.IvReal < -OVER_CURRENT ||
            MC.Sample.IwReal > OVER_CURRENT || MC.Sample.IwReal < -OVER_CURRENT)
        {
            MC.Motor.RunState = MOTOR_ERROR;
            MC.Motor.ErrorCode = OVER_CURRENT_ERR;   // 过流错误代码
        }
    }

    /* ========== 2. 根据系统运行状态执行对应控制 ========== */
    switch (MC.Motor.RunState)
    {
        /* ----- 状态1：ADC 偏移量校准（获取零电流/零电压基准） ----- */
        case ADC_CALIB:
        {
            Calculate_Adc_Offset(&MC.Sample);        // 累加 1024 次采样求平均
            if (MC.Sample.CalibEndFlag == 1)         // 校准完成
            {
                MC.Motor.RunState = MOTOR_IDENTIFY;  // 进入电机参数辨识阶段
            }
        }
        break;

        /* ----- 状态2：电机参数辨识（电阻、电感） ----- */
        case MOTOR_IDENTIFY:
        {
            Motor_Identify();                        // 执行参数辨识状态机
            if (MC.Identify.EndFlag == 1)            // 辨识完成
            {
                // 将辨识结果赋值给滑模观测器（用于无感控制）
                MC.SMO.Rs = MC.Identify.Rs;          // 定子电阻
                MC.SMO.Ld = MC.Identify.Ld;          // 直轴电感（假设 Ld = Lq）
                // 若无故障，则切换至无感控制模式
                if (MC.Motor.RunState != MOTOR_ERROR)
                {
                    MC.Motor.RunState = MOTOR_SENSORLESS;
                }
            }
        }
        break;

        /* ----- 状态4：无感控制（基于高频注入或滑模观测器+锁相环） ----- */
        case MOTOR_SENSORLESS:
        {
            Sensorless_Control();                   // 执行无感 FOC 控制
        }
        break;

        /* ----- 状态5：故障处理（封锁 PWM 输出，关闭驱动） ----- */
        case MOTOR_ERROR:
        {
            // 将三相占空比清零（
            MC.Foc.DutyCycleA = 0;
            MC.Foc.DutyCycleB = 0;
            MC.Foc.DutyCycleC = 0;
            // 根据硬件设计，拉低使能引脚（PB0、PA1、PA2 为驱动使能信号）
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        }
        break;

        /* ----- 状态6：停机状态（正常停止，无故障） ----- */
        case MOTOR_STOP:
        {
            // 输出零占空比，电机停机
            MC.Foc.DutyCycleA = 0;
            MC.Foc.DutyCycleB = 0;
            MC.Foc.DutyCycleC = 0;
        }
        break;

        default:
        break;
    }
}

