#ifndef __FOC_DRV_H
#define __FOC_DRV_H

#include "main.h"

typedef struct
{        
    float Iu;            // U相电流  
    float Iv;            // V相电流 
    float Iw;            // W相电流     
    float Ialpha;        // alpha轴电流 
    float Ibeta;           // beta轴电流 
    
    float SinVal;        // 正弦值
    float CosVal;        // 余弦值
    float Id;               // d轴电流 
    float Iq;               // q轴电流 
    
  float IdLPF;            // d轴电流滤波值
  float IqLPF;              // q轴电流滤波值
  float IdLPFFactor;      // d轴电流滤波系数
  float IqLPFFactor;      // q轴电流滤波系数    
    
    float Ud;            // d轴电压 
    float Uq;            // q轴电压     
    float Ualpha;        // alpha轴电压
    float Ubeta;         // beta轴电压        
    float Ubus;          // 母线电压    
    
  u16   PwmCycle;      // PWM周期
  u16   PwmLimit;         // 最大占空比
    u16   DutyCycleA;    // A相占空比
    u16   DutyCycleB;    // B相占空比
    u16   DutyCycleC;    // C相占空比    
}FOC_STRUCT;

void Clark_Transform(FOC_STRUCT *p);
void Park_Transform(FOC_STRUCT *p); 
void IPark_Transform(FOC_STRUCT *p);
void Calculate_SVPWM(FOC_STRUCT *p);


#endif 


