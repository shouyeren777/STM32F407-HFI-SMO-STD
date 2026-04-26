/* 包含头文件 ----------------------------------------------------------------*/
#include "usart_task.h"
#include "motor_system.h"

TXDATA TxData;
volatile u16 UsartTaskId = 5;
volatile u16 UsartTaskTim = 0;

/**
  * 函数功能: 定时执行串口任务
  * 输入参数:
  * 返 回 值: 
  * 说    明:
  */
void Usart_Task(void)
{
    switch(UsartTaskId)
    {
        case 5: 
        {
            TxData.tail[0] = 0x00;
            TxData.tail[1] = 0x00;
            TxData.tail[2] = 0x80;
            TxData.tail[3] = 0x7f;
            UsartTaskId = 10;
        }
        break;        
        
        case 10:
        {
            if(UsartTaskTim >= 1)        //0.2ms
            {
                UsartTaskTim = 0;
                UsartTaskId = 20;
            }
        }
        break;
        
        case 20:
        { 
            if (__HAL_DMA_GET_COUNTER(&hdma_usart1_tx) == 0) 
            {              
                TxData.fdata[0] = MC.Sample.IuReal;     //打印U相电流值
                TxData.fdata[1] = MC.Sample.IvReal;            //打印V相电流值
                TxData.fdata[2] = MC.Sample.IwReal;            //打印W相电流值
                

                
//                TxData.fdata[0] = MC.IqPid.Ref;    //打印Q轴目标电流值
//              TxData.fdata[1] = MC.IqPid.Fbk;    //打印Q轴实际电流值
                
//                TxData.fdata[0] = MC.SpdPid.Ref;   //打印目标速度值
//                TxData.fdata[1] = MC.SpdPid.Fbk;     //打印实际速度值
                
//                TxData.fdata[0] = MC.PosPid.Ref;   //打印目标位置值
//                TxData.fdata[1] = MC.PosPid.Fbk;     //打印实际位置值
                

                
                __HAL_DMA_DISABLE(&hdma_usart1_tx);
                HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&TxData, sizeof(TxData));//发送结构体(不用对float做处理了)
                __HAL_DMA_ENABLE(&hdma_usart1_tx);
            }                
            UsartTaskId = 10;        
        }
        break;
        
    default:
      break;            
    }
}

