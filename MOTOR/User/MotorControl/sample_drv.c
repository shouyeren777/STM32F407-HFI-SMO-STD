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

#include "sample_drv.h"                                                             

/**
 * 函数功能: 获取相电流和母线电压的基准值（偏移量校准）
 * 输入参数: p - SAMPLE_STRUCT结构体指针，包含ADC原始值、累加和、偏移量、校准标志等
 * 返回参数: 无（结果存入p->IuOffset, p->IwOffset, p->BusOffset, p->BusCalibReal, p->EndFlag）
 * 说    明: 
 *         1. 由于电流传感器和电压分压电路存在零点漂移，需要测量无电流/电压时的ADC平均值作为偏移量。
 *         2. 采样1024次原始值并累加，最后右移10位（除以1024）得到平均值。
 *         3. 该函数应在电机启动前、逆变器下桥臂导通（无电流输出）时调用，确保采样准确。
 *         4. 执行完成后p->EndFlag置1，表示偏移量已准备好，可供后续电流电压计算使用。
 *         5. 偏移量一旦计算完成，通常不再重复校准，除非温度变化较大需要重新校准。
 */
void Calculate_Adc_Offset(SAMPLE_STRUCT *p)
{
    // 若计数器为0，表示开始新的校准过程，初始化所有累加变量和标志
    if (p->OffsetCnt == 0)
    {
        p->CalibEndFlag = 0;          // 清除校准完成标志
        p->IuOffset = 0;         // U相电流偏移累加和清零
        p->IwOffset = 0;         // W相电流偏移累加和清零
        p->BusOffset = 0;        // 母线电压偏移累加和清零
        p->OffsetCnt = 0;        // 采样计数器清零
    }

    // 采样次数未达到1024次时，累加原始ADC值
    if (p->OffsetCnt < 1024)
    {
        p->IuOffset += p->IuRaw;      // 累加U相原始值
        p->IwOffset += p->IwRaw;      // 累加W相原始值
        p->BusOffset += p->BusRaw;    // 累加母线电压原始值
        p->OffsetCnt++;               // 计数器递增
    }
    else
    {
        // 已累加1024次，右移10位（除以1024）求平均值
        p->IuOffset = p->IuOffset >> 10;   // U相电流偏移量（ADC码值）
        p->IwOffset = p->IwOffset >> 10;   // W相电流偏移量（ADC码值）
        p->BusOffset = p->BusOffset >> 10; // 母线电压偏移量（ADC码值）

        // 将母线电压偏移量转换为实际电压值（乘以电压转换系数BusFactor）
        p->BusCalibReal = p->BusOffset * p->BusFactor;

        p->OffsetCnt = 0;          // 计数器归零，便于下次重新校准
        p->CalibEndFlag = 1;            // 设置校准完成标志
    }
}

/**
 * 函数功能: 计算三相电流实际值（以流入电机中性点方向为正）
 * 输入参数: p - SAMPLE_STRUCT结构体指针，包含ADC原始值、偏移量、方向标志、转换系数等
 * 返回参数: 无（结果存入p->IuReal, p->IvReal, p->IwReal）
 * 说    明: 
 *         1. 首先对U相和W相进行偏移补偿并乘以转换系数得到实际电流值。
 *         2. 根据基尔霍夫电流定律，三相电流之和为零：Iu + Iv + Iw = 0，因此 Iv = -Iu - Iw。
 *         3. 电流方向定义：以流入电机中性点为正方向。如果传感器方向与定义相反，可通过p->CurrentDir（±1）调整。
 *         4. 转换系数p->CurrentFactor将ADC原始值（减去偏移后）转换为安培，例如：CurrentFactor = 参考电压/(ADC分辨率*采样电阻*运放增益)。
 *         5. 只采样U相和W相，V相通过计算得到，以减少ADC通道和计算量。
 */
void Calculate_Phase_Current(SAMPLE_STRUCT *p)
{
    // 计算U相实际电流： (原始值 - 偏移量) * 方向 * 转换系数
    p->IuReal = p->CurrentDir * (p->IuRaw - p->IuOffset) * p->CurrentFactor;
    // 计算W相实际电流
    p->IwReal = p->CurrentDir * (p->IwRaw - p->IwOffset) * p->CurrentFactor;
    // 计算V相实际电流： Iu + Iv + Iw = 0  => Iv = -Iu - Iw
    p->IvReal = -p->IuReal - p->IwReal;
}

/**
 * 函数功能: 计算实时母线电压及电压变化量
 * 输入参数: p - SAMPLE_STRUCT结构体指针，包含母线电压原始值、转换系数、校准值等
 * 返回参数: 无（结果存入p->BusReal, p->BusChange）
 * 说    明: 
 *         1. 将母线电压ADC原始值乘以转换系数BusFactor，得到实际电压值BusReal（单位：伏特）。
 *         2. BusFactor的计算：BusFactor = (分压比 * 参考电压) / ADC分辨率，例如分压电阻分压后测量范围0~3.3V，ADC为12位，则BusFactor = 3.3/4095 * 分压比倒数。
 *         3. 计算电压变化量BusChange = 当前实时电压 - 校准电压（无负载时的基准电压），可用于判断母线电压波动或进行前馈补偿。
 *         4. 校准电压BusCalibReal由Calculate_Adc_Offset函数在系统启动时测量得到，代表无电流时的母线电压零点（通常为0或电源电压的一半，取决于电路）。
 */
void Calculate_Bus_Voltage(SAMPLE_STRUCT *p)
{
    // 实时母线电压 = 原始ADC值 * 转换系数
    p->BusReal = p->BusRaw * p->BusFactor;
    // 母线电压变化量 = 实时电压 - 校准基准电压
    p->BusChange = p->BusReal - p->BusCalibReal;
}






