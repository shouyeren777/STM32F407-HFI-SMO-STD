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

#include "position_drv.h"                                

/**
 * 函数功能: 计算位置（基于编码器电角度累加，支持过零处理）
 * 输入参数: p - POSITION_STRUCT结构体指针，包含当前电角度ElectricalPosThis、
 *           上次电角度ElectricalPosLast、电角度最大值ElectricalValMax、
 *           位置累加和ElectricalPosSum等
 * 返回参数: 无（结果存入p->ElectricalPosSum，即累加总电角度）
 * 说    明: 
 *         1. 该函数通过对编码器电角度的变化量进行累加，得到转子转过的总电角度。
 *            可用于绝对位置控制（如伺服定位）或检测多圈旋转角度。
 *         2. 算法步骤：
 *            (1) 计算本次与上次电角度的差值：ElectricalPosChange = This - Last。
 *            (2) 更新上次电角度为本次值，供下次调用使用。
 *            (3) 处理编码器零点绕圈：由于电角度范围在[0, ElectricalValMax)内，
 *                当转子正向越过零点时，本次角度很小（接近0），上次角度很大（接近最大值），
 *                差值为负且绝对值大于半量程，此时应加上一个周期（ElectricalValMax）进行补偿。
 *                反向越过零点时类似，减去一个周期。
 *            (4) 将修正后的差值累加到总位置ElectricalPosSum中，得到累计电角度。
 *         3. 关键参数：
 *            - ElectricalValMax：电角度最大值，对于编码器通常是编码器单圈计数值（如4096），
 *              对于弧度制则是2π（约6.28318）。
 *            - 半量程判断：使用0.5f * ElectricalValMax作为阈值，可正确处理±180°范围内的跳变。
 *         4. 注意事项：
 *            - 调用前需确保p->ElectricalPosLast已初始化（通常为第一次采样值）。
 *            - 该函数累加的ElectricalPosSum会随转子旋转不断增长或减小，
 *              可用于计算多圈绝对位置，但需注意变量类型（如float或s32）防止溢出。
 *            - 配合极对数PolePairs，可转换为机械总位置：MechanicalPosSum = ElectricalPosSum / PolePairs。
 */
void Calculate_Position(POSITION_STRUCT *p)
{
    // 计算本次电角度变化量（本次 - 上次）
    p->ElectricalPosChange = p->ElectricalPosThis - p->ElectricalPosLast;
    // 更新上次电角度为本次值
    p->ElectricalPosLast = p->ElectricalPosThis;

    // 正向越过零点处理：若变化量 ≥ 半量程，实际应为负变化，减去一个周期
    if (p->ElectricalPosChange >= (p->ElectricalValMax * 0.5f))
    {
        p->ElectricalPosChange = p->ElectricalPosChange - p->ElectricalValMax;
    }
    // 反向越过零点处理：若变化量 ≤ -半量程，实际应为正变化，加上一个周期
    if (p->ElectricalPosChange <= (-p->ElectricalValMax * 0.5f))
    {
        p->ElectricalPosChange = p->ElectricalPosChange + p->ElectricalValMax;
    }

    // 累加得到总电角度位置
    p->ElectricalPosSum = p->ElectricalPosSum + p->ElectricalPosChange;
}









