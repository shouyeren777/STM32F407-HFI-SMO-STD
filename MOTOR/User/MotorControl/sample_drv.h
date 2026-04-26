#ifndef __SAMPLE_DRV_H
#define __SAMPLE_DRV_H

#include "main.h"

typedef struct
{    
  s8    CurrentDir;                 // 电流采样方向
    u8    CalibEndFlag;               // 校准完成标志    
    u16   OffsetCnt;                  // 基准值计算次数
    s32   BusOffset;                     // 母线电压基准值    
    s32   IuOffset;                   // U相电流偏置值
    s32   IvOffset;                   // V相电流偏置值        
    s32   IwOffset;                   // W相电流偏置值
    s32   BusRaw;                       // 母线电压原始值
    s32   IuRaw;                      // U相电流原始值
    s32   IvRaw;                      // V相电流原始值
    s32   IwRaw;                      // W相电流原始值    
    float IuReal;                     // U相电流真实值           
    float IvReal;                     // V相电流真实值  
    float IwReal;                     // W相电流真实值 
    float BusReal;                    // 母线电压真实值(动态)
    float BusCalibReal;               // 母线电压真实值(校准值，静态)
    float BusChange;                  // 母线电压变化值
    float BusFactor;                  // 母线电压计算系数
    float CurrentFactor;              // 相电流计算系数
    u16   AdcBuff[3];                 // 用于ADC规则通道接收数据    
}SAMPLE_STRUCT;


void Calculate_Adc_Offset(SAMPLE_STRUCT *p);
void Calculate_Phase_Current(SAMPLE_STRUCT *p);
void Calculate_Bus_Voltage(SAMPLE_STRUCT *p);





#endif 


