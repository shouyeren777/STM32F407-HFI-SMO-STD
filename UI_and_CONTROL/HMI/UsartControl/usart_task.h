#ifndef __USART_TASK_H
#define __USART_TASK_H


#include "main.h"

#define CH_COUNT 3               //定义要发送的浮点数个数
typedef struct
{                                                   
    float fdata[CH_COUNT];         //要发送的浮点数
    u8    tail[4];                 //VOFA的JUSTFLOAT格式的帧尾
}TXDATA;


void Usart_Task(void);







#endif



