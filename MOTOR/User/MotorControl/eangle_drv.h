#ifndef __EANGLE_DRV_H
#define __EANGLE_DRV_H

#include "main.h"

typedef struct
{
    u8    Dir;                        // 编码器方向    
    u8    PolePairs;                  // 转子极对数
    s32   EncoderVal;                 // 编码器原始数据
    s32   EncoderValMax;              // 编码器最大原始值
    s32   EncoderValChange;           // 编码器值变化量
    u16   CalibFlag;                  // 校准完成标志
    s32   CalibOffset;                // 转子零位偏差    
    float Ts;
    float ElectricalAnglePU;          // 编码器电角度标幺值
    float ElectricalAngleSpdSet;      // 给定的电角速度        
    float ElectricalAngleSetPU;       // 给定电角度标幺值
}E_ANGLE_STRUCT; 

void Electrical_Angle_Generator(E_ANGLE_STRUCT *p);
void Calculate_Encoder_Data(E_ANGLE_STRUCT *p);


#endif 


