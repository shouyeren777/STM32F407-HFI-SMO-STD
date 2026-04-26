#ifndef F407_HMI_KEY_DRV_H
#define F407_HMI_KEY_DRV_H

#include "main.h"


/************************************宏定义*************************************/

#define KEY0      HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)
#define KEY1      HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)
#define KEY2      HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)

/**********************************结构体定义***********************************/

typedef struct button
{
    uint8_t level;
    uint8_t status;
    uint16_t scan_cnt;
}button_t;

                              
/***********************************函数声明************************************/

void Key_Scan(void);

#endif /* F407_HMI_KEY_DRV_H */

/*********************************END OF FILE***********************************/






