#ifndef __EANGLE_DRV_H
#define __EANGLE_DRV_H

#include "main.h"

typedef struct
{
    u8    PolePairs;                  // 转子极对数
    float Ts;
    float ElectricalAngleSpdSet;      // 给定的电角速度        
    float ElectricalAngleSetPU;       // 给定电角度标幺值
}E_ANGLE_STRUCT; 

void Electrical_Angle_Generator(E_ANGLE_STRUCT *p);


#endif 


