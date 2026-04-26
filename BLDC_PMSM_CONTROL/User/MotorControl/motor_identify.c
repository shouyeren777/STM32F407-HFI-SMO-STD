/* 包含头文件 ----------------------------------------------------------------*/

#include "motor_identify.h"

/**
 * 函数功能: 电机参数辨识（电阻、电感）
 * 输入参数: 无（所有参数通过全局结构体 MC.Identify、MC.Foc、MC.Sample、MC.Encoder 传递）
 * 返回参数: 无（结果存入 MC.Identify.Rs、MC.Identify.Ls 等，最终设置 MC.Identify.EndFlag）
 * 说    明: 
 *         1. 该函数通过两个子状态自动完成电机参数辨识：
 *            - RESISTANCE_IDENTIFICATION：直流伏安法测量定子电阻 Rs
 *            - INDUCTANCE_IDENTIFICATION：直流阶跃法测量定子电感 Ls（假设 Ld = Lq）
 *         2. 该函数应在 PWM 中断中周期性调用，每次执行一步状态机，直到 EndFlag 置1。
 *         3. 辨识过程中电机轴会轻微转动，需确保负载脱离。
 *         4. 电阻辨识原理：向 d 轴注入直流电流（电角度=0），测量不同电压下的稳定电流，
 *            根据欧姆定律 Rs = ΔU / ΔI。
 *         5. 电感辨识原理：施加阶跃电压，测量电流从0上升到稳定值95%的时间，
 *            对于一阶 RL 电路，L = R * t / ln(20)。
 *         6. 无编码器场景下，电感辨识完成即结束辨识流程。
 */
void Motor_Identify(void)
{
    /* 根据当前辨识阶段执行不同操作 */
    switch (MC.Identify.State)
    {
        /* ================= 阶段1：定子电阻辨识 ================= */
        case RESISTANCE_IDENTIFICATION:
        {
            /* ----- 子状态 Flag = 0：清空参数，准备第一次电流设定 ----- */
            if (MC.Identify.Flag == 0)
            {
                MC.Foc.Uq = 0;                 // 交轴电压清零（只施加直轴电压）
                MC.Foc.Ud = 0;                 // 直轴电压从0开始递增
                MC.Identify.Count = 0;          // 计数清零（本阶段未使用）
                MC.Identify.WaitTim = 0;        // 等待计时器清零
                MC.Identify.Flag = 1;           // 进入下一个子状态
            }

            /* ----- 子状态 Flag = 1：逐步增加 Ud 直到电流达到第一个目标值（0.6倍最大电流）----- */
            if (MC.Identify.Flag == 1)
            {
                // 计算当前实际电流值：Iu * Ud * 1.5 / Ubus
                // 推导：Ud施加于d轴，实际d轴电流 Id ≈ Iu（因为电角度=0，Iu与Id同相）
                // 1.5 为三相电流标幺化系数（取决于Clark变换实现）
                float current = (MC.Sample.IuReal * MC.Foc.Ud * 1.5f) / MC.Sample.BusReal;
                if (current >= 0.6f * MC.Identify.CurMax)   // 达到目标电流的60%
                {
                    MC.Identify.Flag = 2;       // 进入等待稳定和记录阶段
                }
                else
                {
                    MC.Foc.Ud += 0.0001f;       // 每次调用增加0.0001V（约0.1mV），逐步升压
                    MC.Identify.VoltageSet[0] = MC.Foc.Ud;   // 记录第一次电压点
                }
            }

            /* ----- 子状态 Flag = 2：等待电流稳定，采样100次求平均（第一组电流）----- */
            if (MC.Identify.Flag == 2)
            {
                MC.Identify.WaitTim++;          // 计数器累加（每次函数调用约50~100us，需根据实际周期调整）
                if (MC.Identify.WaitTim > 4000) // 等待约0.2秒（假设周期50us，4000次=0.2s）
                {
                    MC.Identify.CurSum += MC.Sample.IuReal;   // 累加相电流
                }
                if (MC.Identify.WaitTim >= 4100) // 再采集100次（4100-4000=100次）
                {
                    MC.Identify.CurAverage[0] = MC.Identify.CurSum * 0.01f; // 平均电流 = 累加和 / 100
                    MC.Identify.WaitTim = 0;     // 计时器复位
                    MC.Identify.CurSum = 0;       // 累加和清零
                    MC.Identify.Flag = 3;         // 进入第二组电压/电流测量
                }
            }

            /* ----- 子状态 Flag = 3：继续升压，直到电流达到最大电流 CurMax（第二组）----- */
            if (MC.Identify.Flag == 3)
            {
                float current = (MC.Sample.IuReal * MC.Foc.Ud * 1.5f) / MC.Sample.BusReal;
                if (current >= MC.Identify.CurMax)   // 达到最大电流
                {
                    MC.Identify.Flag = 4;       // 进入等待稳定和记录阶段
                }
                else
                {
                    MC.Foc.Ud += 0.0001f;       // 继续升压
                    MC.Identify.VoltageSet[1] = MC.Foc.Ud;   // 记录第二次电压点
                }
            }

            /* ----- 子状态 Flag = 4：等待稳定，采样100次求平均（第二组电流）----- */
            if (MC.Identify.Flag == 4)
            {
                MC.Identify.WaitTim++;
                if (MC.Identify.WaitTim > 4000)
                {
                    MC.Identify.CurSum += MC.Sample.IuReal;
                }
                if (MC.Identify.WaitTim >= 4100)
                {
                    MC.Identify.CurAverage[1] = MC.Identify.CurSum * 0.01f; // 平均电流
                    MC.Identify.WaitTim = 0;
                    MC.Identify.CurSum = 0;
                    MC.Identify.Flag = 5;       // 进入计算电阻阶段
                }
            }

            /* ----- 子状态 Flag = 5：计算定子电阻 Rs = ΔU / ΔI ----- */
            if (MC.Identify.Flag == 5)
            {
                // 电阻 = (第二组电压 - 第一组电压) / (第二组电流 - 第一组电流)
                MC.Identify.Rs = (MC.Identify.VoltageSet[1] - MC.Identify.VoltageSet[0])
                                 / (MC.Identify.CurAverage[1] - MC.Identify.CurAverage[0]);
                MC.Foc.Ud = 0;                  // 关断电压
                MC.Identify.Flag = 0;           // 重置标志
                MC.Identify.State = INDUCTANCE_IDENTIFICATION;  // 切换到电感辨识阶段
            }

            /* 电阻辨识过程中，始终保持电角度为0（d轴对齐） */
            MC.Foc.SinVal = 0;                  // sin(0) = 0
            MC.Foc.CosVal = 1;                  // cos(0) = 1
            IPark_Transform(&MC.Foc);           // 反Park变换，将Ud/Uq转换为Ualpha/Ubeta
        }
        break;

        /* ================= 阶段2：定子电感辨识 ================= */
        case INDUCTANCE_IDENTIFICATION:
        {
            /* ----- 子状态 Flag = 0：等待电流归零，确保初始条件 ----- */
            if (MC.Identify.Flag == 0)
            {
                MC.Foc.Uq = 0;
                MC.Foc.Ud = 0;
                // 检查相电流是否接近0（±0.05A以内）
                if (MC.Sample.IuReal >= -0.05f && MC.Sample.IuReal <= 0.05f)
                {
                    MC.Identify.Flag = 1;       // 电流归零，开始测试
                }
            }

            /* ----- 子状态 Flag = 1：施加固定电压 Ud，测量电流上升时间，计算电感 ----- */
            if (MC.Identify.Flag == 1)
            {
                MC.Foc.Ud = MC.Identify.VoltageSet[1];   // 使用电阻辨识时第二次的电压值
                MC.Identify.WaitTim++;                    // 计时器递增（单位：函数调用周期）

                // 当电流上升到平均电流的95%时，记录时间
                if (MC.Sample.IuReal >= MC.Identify.CurAverage[1] * 0.95f)
                {
                    // 电感计算公式：L = R * t / ln(1/(1-0.95)) = R * t / ln(20)
                    // ln(20) ≈ 2.9957，系数 0.334 = 1/2.9957 ≈ 0.334
                    // 0.00005 为函数调用周期（假设周期50us），WaitTim为周期数，故 t = WaitTim * 0.00005
                    MC.Identify.LsSum += MC.Identify.Rs * 0.334f * 0.00005f * MC.Identify.WaitTim;
                    MC.Identify.WaitTim = 0;              // 计时器复位
                    MC.Identify.Count++;                  // 完成一次测量，计数器+1
                    MC.Identify.Flag = 0;                 // 返回等待电流归零状态，进行下一次测量
                    MC.Foc.Ud = 0;                        // 关断电压，让电流下降
                    if (MC.Identify.Count >= 100)         // 重复测量100次，取平均值
                    {
                        MC.Identify.Flag = 2;             // 进入计算最终电感
                    }
                }
            }

            /* ----- 子状态 Flag = 2：计算平均电感，结束辨识流程 ----- */
            if (MC.Identify.Flag == 2)
            {
                MC.Identify.Ls = MC.Identify.LsSum * 0.01f;   // 平均值 = 累加和 / 100
                MC.Identify.Ld = MC.Identify.Ls;             // 假设表贴式电机，Ld = Lq = Ls
                MC.Identify.Lq = MC.Identify.Ls;
                MC.Identify.Flag = 0;
                MC.Identify.LsSum = 0;
                MC.Identify.WaitTim = 0;
                MC.Foc.Ud = 0;
                MC.Foc.Uq = 0;
                MC.Identify.EndFlag = 1;                    // 无编码器场景下直接完成辨识
            }

            /* 电感辨识过程中，同样保持电角度为0 */
            MC.Foc.SinVal = 0;
            MC.Foc.CosVal = 1;
            IPark_Transform(&MC.Foc);
        }
        break;

        default:
        break;
    }

    /* 所有阶段共用的SVPWM输出（根据当前Foc结构体中的Ualpha/Ubeta和Ubus生成PWM） */
    MC.Foc.Ubus = MC.Sample.BusReal;                // 更新实时母线电压
    Calculate_SVPWM(&MC.Foc);                       // SVPWM调制输出
}




