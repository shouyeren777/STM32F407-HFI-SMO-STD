#include "lcd_mid.h"
#include "lcd_drv.h"
#include "motor_system.h"

s8 YCursor = 0;
s8 YCursorLast = 100;
u8 InitPage1 = 0;
u8 InitPage2 = 0;

extern const unsigned char gImage_hs_logo[8052];

void LCD_Clear()               
{
    LCD_Fill(0,0,LCD_W,LCD_H,BLACK);            //清屏
}

void LCD_Display_Logo()                       //显示LOGO 只在运行开始时显示
{
    LCD_Init();                                 //屏幕初始化
    LCD_ShowPicture(47,7,61,66,gImage_hs_logo); //显示浩盛LOGO
    HAL_Delay(1500);                            //延时显示浩盛LOGO    
    LCD_Fill(0,0,LCD_W,LCD_H,BLACK);            //清屏
}

void LCD_Display_Page1()                      
{
    InitPage1++;
    if(InitPage1 == 1)                          //静态内容显示                   
    {
        LCD_ShowChinese(64,0,"电压",WHITE,RED,16,0);    
        LCD_ShowString(96,0,":",LIGHTGREEN,BLACK,16,0);
        LCD_ShowString(152,0,"V",LIGHTGREEN,BLACK,16,0);    
        
        LCD_ShowChinese(64,16,"电流",BLACK,YELLOW,16,0);    
        LCD_ShowString(96,16,":",LIGHTGREEN,BLACK,16,0);
        LCD_ShowString(152,16,"A",LIGHTGREEN,BLACK,16,0);    

        LCD_ShowChinese(64,32,"速度",WHITE,LIGHTBLUE,16,0);    
        LCD_ShowString(96,32,":",LIGHTGREEN,BLACK,16,0);
        LCD_ShowString(152,32,"R",LIGHTGREEN,BLACK,16,0);    

        LCD_ShowChinese(64,48,"位置",BLACK,GREEN,16,0);    
        LCD_ShowString(96,48,":",LIGHTGREEN,BLACK,16,0);
        LCD_ShowString(152,48,"R",LIGHTGREEN,BLACK,16,0);    

        LCD_DrawLine(0,0,60,0,WHITE);               //画线
        LCD_DrawLine(60,0,60,31,WHITE);             //画线
        LCD_DrawLine(0,0,0,30,WHITE);                 //画线
        LCD_DrawLine(0,30,60,30,WHITE);             //画线
        LCD_DrawLine(1,1,59,1,WHITE);               //画线
        LCD_DrawLine(59,1,59,30,WHITE);             //画线
        LCD_DrawLine(1,1,1,29,WHITE);                 //画线
        LCD_DrawLine(1,29,59,29,WHITE);             //画线
        
        LCD_ShowString(7,7,"HS FOC",WHITE,BLACK,16,0);            
        LCD_ShowString(4,32,"RS:",WHITE,BLACK,16,0);    
        LCD_ShowString(4,48,"LD:",WHITE,BLACK,16,0);    
        LCD_ShowString(4,64,"STATUS:",WHITE,BLACK,16,0);        
    }

    if(InitPage1 >= 2)                       //动态内容显示                   
    {
        InitPage1 = 2;    

        LCD_ShowIntNum(26,32,MC.Identify.Rs * 1000,4,ROSE_PINK,BLACK,16);    
        LCD_ShowIntNum(26,48,MC.Identify.Ls * 1000000,4,ROSE_PINK,BLACK,16);            
        LCD_ShowFloatNum1(112,0,MC.Sample.BusReal,4,ROSE_PINK,BLACK,16);    
        
        if(MC.Foc.Iq < 0)
        {
            LCD_ShowString(104,16,"-",ROSE_PINK,BLACK,16,0);
            LCD_ShowFloatNum1(112,16,-MC.Foc.Iq,4,ROSE_PINK,BLACK,16);        
        }
        else
        {
            LCD_ShowString(104,16," ",ROSE_PINK,BLACK,16,0);
            LCD_ShowFloatNum1(112,16,MC.Foc.Iq,4,ROSE_PINK,BLACK,16);        
        }    

        if(MC.Speed.MechanicalSpeed < 0)
        {
            LCD_ShowString(104,32,"-",ROSE_PINK,BLACK,16,0);
            LCD_ShowIntNum(112,32,-MC.Speed.MechanicalSpeed,5,ROSE_PINK,BLACK,16);        
        }
        else
        {
            LCD_ShowString(104,32," ",ROSE_PINK,BLACK,16,0);
            LCD_ShowIntNum(112,32,MC.Speed.MechanicalSpeed,5,ROSE_PINK,BLACK,16);        
        }        
        
        if(MC.Position.MechanicalPosRaw < 0)
        {
            LCD_ShowString(104,48,"-",ROSE_PINK,BLACK,16,0);
            LCD_ShowFloatNum1(112,48,-(float)MC.Position.MechanicalPosRaw/PUL_MAX,4,ROSE_PINK,BLACK,16);        
        }
        else
        {
            LCD_ShowString(104,48," ",ROSE_PINK,BLACK,16,0);
            LCD_ShowFloatNum1(112,48,(float)MC.Position.MechanicalPosRaw/PUL_MAX,4,ROSE_PINK,BLACK,16);        
        }    
        
        switch (MC.Motor.RunState)
        {        
            case ADC_CALIB:                          //ADC校准
            {
                LCD_ShowString(64,64,"ADC_CALIB",BLACK,WHITE,16,0);
            }break;    
            
            case MOTOR_IDENTIFY:                     //参数辨识
            {
                LCD_ShowChinese(64,64,"电机参数辨识",BLACK,WHITE,16,0);    
            }break;    
            
            case MOTOR_SENSORLESS:                                                 //无感控制
            {
                switch(MC.Motor.RunMode)
                {
                    case STRONG_DRAG_CURRENT_OPEN:
                    {
                        LCD_ShowChinese(64,64,"无感强拖开环",BLACK,WHITE,16,0);
                    }break;

                    case STRONG_DRAG_CURRENT_CLOSE:
                    {
                        LCD_ShowChinese(64,64,"无感强拖闭环",BLACK,WHITE,16,0);
                    }break;

                    case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:                                                
                    {    
                        LCD_ShowChinese(64,64,"无感强拖滑膜",BLACK,WHITE,16,0);                        
                    }break;                

                    case HFI_CURRENT_CLOSE:
                    {
                        LCD_ShowChinese(64,64,"高频注入电流",BLACK,WHITE,16,0);
                    }break;
                    
                    case HFI_SMO_SPEED_CURRENT_CLOSE:                                      
                    {       
                        LCD_ShowChinese(64,64,"高频注入滑膜",BLACK,WHITE,16,0);    
                    }break;    

                } 
            }break;                        
        }            
    }            
}

void LCD_Display_Page2()
{    
    InitPage2++;
    if(InitPage2 == 1)                         //静态内容显示                   
    {
        LCD_ShowChinese(6,8,  "模",WHITE,BLACK,16,0);    
        LCD_ShowChinese(6,24, "式",WHITE,BLACK,16,0);    
        LCD_ShowChinese(6,40, "切",WHITE,BLACK,16,0);    
        LCD_ShowChinese(6,56, "换",WHITE,BLACK,16,0);    
        
        LCD_ShowChinese(64,0, "无感强拖开环",ROSE_PINK,BLACK,16,0);
        LCD_ShowChinese(64,16,"无感强拖闭环",ROSE_PINK,BLACK,16,0);
        LCD_ShowChinese(64,32,"无感强拖滑膜",ROSE_PINK,BLACK,16,0);
        LCD_ShowChinese(64,48,"高频注入电流",ROSE_PINK,BLACK,16,0);
        LCD_ShowChinese(64,64,"高频注入滑膜",ROSE_PINK,BLACK,16,0);
    }

    if(InitPage1 >= 2)                       //动态内容显示                   
    {
        InitPage1 = 2;    
      LCD_ShowString(30,YCursor,"-->",WHITE,BLACK,16,0);    
        LCD_ShowString(30,YCursorLast,"   ",WHITE,BLACK,16,0);                
    }
}

  
