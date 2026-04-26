
/**
  ******************************************************************************
  * 文件名程: 
  * 作    者: 浩然
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/

#include "global_control.h"
#include "motor_system.h"
#include "led_task.h"
#include "lcd_task.h"
#include "lcd_mid.h"
#include "key_task.h"
#include "usart_task.h"

extern volatile u16 LedTaskTim;
extern volatile u16 LcdTaskTim;
extern volatile u16 KeyTaskTim;
extern volatile u16 UsartTaskTim;

/**
  * 函数功能: 全局初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Global_Init(void)
{
    HAL_Delay(100);                                         //延时等待电源稳定
    LCD_Display_Logo();                                     //LCD显示浩盛LOGO
    Motor_System_Init();                                    //电机系统初始化
    
    HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);   //ADC校准
    HAL_Delay(10);                                          //等待ADC校准完成
    HAL_ADC_Start_DMA(&hadc2,(u32 *)MC.Sample.AdcBuff,3);   //启动ADC开启DMA搬运
    
//    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);         //启动编码器接口 

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);          //设置初始占空比
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);          //设置初始占空比
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);          //设置初始占空比

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);               //开启对应通道PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);               //开启对应通道PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);               //开启对应通道PWM输出

    HAL_TIM_Base_Start_IT(&htim1);                          //开启定时器中断
    HAL_TIM_Base_Start_IT(&htim2);                          //开启定时器中断
    HAL_TIM_Base_Start(&htim6);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);     //使能SD1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);     //使能SD2
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);     //使能SD3
}

/**
  * 函数功能: 主循环
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Global_Loop(void)
{
    Lcd_Task();
}

/**
  * 函数功能: 定时器中断回调函数
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim1.Instance)                        //20KHZ   50US
    {
        HAL_ADCEx_InjectedStart_IT(&hadc2);                     //开启ADC注入通道中断
    }
    
    if(htim->Instance == htim2.Instance)                        //10KHZ   100US
    {
        LedTaskTim++;                                           //LED任务计时
        LcdTaskTim++;                                           //LCD任务计时
        KeyTaskTim++;                                           //按键扫描任务计时
        UsartTaskTim++;                                         //串口任务计时
        
        Led_Task();
        Key_Task();
        Usart_Task();
    }
}

/**
  * 函数功能: 设置目标值（无电位器）
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void Target_Set(void)
{
    const float default_speed_rpm = 1000.0f;

    switch(MC.Motor.RunMode)
    {
        case STRONG_DRAG_CURRENT_OPEN:
        case STRONG_DRAG_CURRENT_CLOSE:
        case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:
        case HFI_SPEED_CURRENT_CLOSE:
        case HFI_SMO_SPEED_CURRENT_CLOSE:
        {
            MC.Speed.MechanicalSpeedSet  =  Speed_Set_Dir * default_speed_rpm; // 无电位器时使用固定目标转速
        }break;                

        case HFI_CURRENT_CLOSE:
        {       
            MC.IqPid.Ref = 0; // 纯电流闭环测试模式默认给零转矩
        }break;    

        default:
        break;
    }
}

/**
  * 函数功能: ADC注入中断回调函数
  * 输入参数:
  * 返回参数:
  * 说    明: 20KHZ频率即50US执行一次
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{        
    MC.Sample.IuRaw = ADC2->JDR1;              //获取相电流
    MC.Sample.IwRaw = ADC2->JDR2;              //获取相电流
    MC.Sample.BusRaw = MC.Sample.AdcBuff[0];   //获取母线电压
    Target_Set();                              // 无电位器场景下的固定目标值
    Motor_System_Run();                        //电机系统运行

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,MC.Foc.DutyCycleA);     //更新PWM比较值
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,MC.Foc.DutyCycleB);     //更新PWM比较值
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,MC.Foc.DutyCycleC);     //更新PWM比较值
}





