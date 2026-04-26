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
#include "lcd_drv.h"
#include "font_drv.h"

#define LCD_DMA_BUFFER_SIZE 256

static u8 lcd_dma_buffer[LCD_DMA_BUFFER_SIZE];
static u16 lcd_buffer_index = 0;


static void LCD_Flush_buffer(void) {
  if (lcd_buffer_index > 0) {
    LCD_CS_Clr();
    HAL_SPI_Transmit(&hspi3, lcd_dma_buffer, lcd_buffer_index, 10);
    LCD_CS_Set();
    lcd_buffer_index = 0;
  }
}


static void LCD_buffer_write(u8 dat) {
  if (lcd_buffer_index >= LCD_DMA_BUFFER_SIZE) {
    LCD_Flush_buffer();
  }
  lcd_dma_buffer[lcd_buffer_index++] = dat;
}

static void LCD_Send_Byte(u8 dat) {
  LCD_CS_Clr();
  HAL_SPI_Transmit(&hspi3, &dat, 1, 1);
  LCD_CS_Set();
}

/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
                                color       要填充的颜色
      返回值：  无
******************************************************************************/
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color) {
  u16 i, j;
  u8 color_h = color >> 8;
  u8 color_l = color & 0xFF;
  LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);
  
  for (i = ysta; i < yend; i++) {
    for (j = xsta; j < xend; j++) {
      LCD_buffer_write(color_h);
      LCD_buffer_write(color_l);
    }
  }
  LCD_Flush_buffer();
}

/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(u16 x, u16 y, u16 color) {
  LCD_Address_Set(x, y, x, y);
  LCD_Send_Byte(color >> 8);
  LCD_Send_Byte(color & 0xFF);
}


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color) {
  u16 t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, uRow, uCol;
  delta_x = x2 - x1;
  delta_y = y2 - y1;
  uRow = x1;
  uCol = y1;
  if (delta_x > 0)
    incx = 1;
  else if (delta_x == 0)
    incx = 0;
  else {
    incx = -1;
    delta_x = -delta_x;
  }
  if (delta_y > 0)
    incy = 1;
  else if (delta_y == 0)
    incy = 0;
  else {
    incy = -1;
    delta_y = -delta_y;
  }
  if (delta_x > delta_y)
    distance = delta_x;
  else
    distance = delta_y;
  for (t = 0; t < distance + 1; t++) {
    LCD_DrawPoint(uRow, uCol, color);
    xerr += delta_x;
    yerr += delta_y;
    if (xerr > distance) {
      xerr -= distance;
      uRow += incx;
    }
    if (yerr > distance) {
      yerr -= distance;
      uCol += incy;
    }
  }
}


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
    LCD_DrawLine(x1,y1,x2,y1,color);
    LCD_DrawLine(x1,y1,x1,y2,color);
    LCD_DrawLine(x1,y2,x2,y2,color);
    LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
    int a,b;
    a=0;b=r;      
    while(a<=b)
    {
        LCD_DrawPoint(x0-b,y0-a,color);             //3           
        LCD_DrawPoint(x0+b,y0-a,color);             //0           
        LCD_DrawPoint(x0-a,y0+b,color);             //1                
        LCD_DrawPoint(x0-a,y0-b,color);             //2             
        LCD_DrawPoint(x0+b,y0+a,color);             //4               
        LCD_DrawPoint(x0+a,y0-b,color);             //5
        LCD_DrawPoint(x0+a,y0+b,color);             //6 
        LCD_DrawPoint(x0-b,y0+a,color);             //7
        a++;
        if((a*a+b*b)>(r*r))//判断要画的点是否过远
        {
            b--;
        }
    }
}

/******************************************************************************
      函数说明：显示汉字串
      入口数据：x,y显示坐标
                *s 要显示的汉字串
                fc 字的颜色
                bc 字的背景色
                sizey 字号 可选 16 24 32
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
    while(*s!=0)
    {
        if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
        else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
        else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
        else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
        else return;
        s+=2;
        x+=sizey;
    }
}

/******************************************************************************
      函数说明：显示单个12x12汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese12x12(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode) {
  u8 i, j, m = 0;
  u16 k;
  u16 HZnum;
  u16 TypefaceNum;
  u16 x0 = x;
  TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;

  HZnum = sizeof(tfont12) / sizeof(typFNT_GB12);
  for (k = 0; k < HZnum; k++) {
    if ((tfont12[k].Index[0] == *(s)) && (tfont12[k].Index[1] == *(s + 1))) {
      LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
      for (i = 0; i < TypefaceNum; i++) {
        for (j = 0; j < 8; j++) {
          if (!mode) {
            if (tfont12[k].Msk[i] & (0x01 << j)) {
              LCD_buffer_write(fc >> 8);
              LCD_buffer_write(fc & 0xFF);
            } else {
              LCD_buffer_write(bc >> 8);
              LCD_buffer_write(bc & 0xFF);
            }
            m++;
            if (m % sizey == 0) {
              m = 0;
              break;
            }
          } else {
            if (tfont12[k].Msk[i] & (0x01 << j))
              LCD_DrawPoint(x, y, fc);
            x++;
            if ((x - x0) == sizey) {
              x = x0;
              y++;
              break;
            }
          }
        }
      }
      LCD_Flush_buffer();
    }
    continue;
  }
}

/******************************************************************************
      函数说明：显示单个16x16汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese16x16(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey,
                          u8 mode) {
  u8 i, j, m = 0;
  u16 k;
  u16 HZnum;
  u16 TypefaceNum;
  u16 x0 = x;
  TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
  HZnum = sizeof(tfont16) / sizeof(typFNT_GB16);
  for (k = 0; k < HZnum; k++) {
    if ((tfont16[k].Index[0] == *(s)) && (tfont16[k].Index[1] == *(s + 1))) {
      LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
      for (i = 0; i < TypefaceNum; i++) {
        for (j = 0; j < 8; j++) {
          if (!mode) {
            if (tfont16[k].Msk[i] & (0x01 << j)) {
              LCD_buffer_write(fc >> 8);
              LCD_buffer_write(fc & 0xFF);
            } else {
              LCD_buffer_write(bc >> 8);
              LCD_buffer_write(bc & 0xFF);
            }
            m++;
            if (m % sizey == 0) {
              m = 0;
              break;
            }
          } else {
            if (tfont16[k].Msk[i] & (0x01 << j))
              LCD_DrawPoint(x, y, fc);
            x++;
            if ((x - x0) == sizey) {
              x = x0;
              y++;
              break;
            }
          }
        }
      }
      LCD_Flush_buffer();
    }
    continue;
  }
}


/******************************************************************************
      函数说明：显示单个24x24汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese24x24(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey,
                          u8 mode) {
  u8 i, j, m = 0;
  u16 k;
  u16 HZnum;
  u16 TypefaceNum;
  u16 x0 = x;
  TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
  HZnum = sizeof(tfont24) / sizeof(typFNT_GB24);
  for (k = 0; k < HZnum; k++) {
    if ((tfont24[k].Index[0] == *(s)) && (tfont24[k].Index[1] == *(s + 1))) {
      LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
      for (i = 0; i < TypefaceNum; i++) {
        for (j = 0; j < 8; j++) {
          if (!mode) {
            if (tfont24[k].Msk[i] & (0x01 << j)) {
              LCD_buffer_write(fc >> 8);
              LCD_buffer_write(fc & 0xFF);
            } else {
              LCD_buffer_write(bc >> 8);
              LCD_buffer_write(bc & 0xFF);
            }
            m++;
            if (m % sizey == 0) {
              m = 0;
              break;
            }
          } else {
            if (tfont24[k].Msk[i] & (0x01 << j))
              LCD_DrawPoint(x, y, fc);
            x++;
            if ((x - x0) == sizey) {
              x = x0;
              y++;
              break;
            }
          }
        }
      }
      LCD_Flush_buffer();
    }
    continue;
  }
}

/******************************************************************************
      函数说明：显示单个32x32汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese32x32(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey,
                          u8 mode) {
  u8 i, j, m = 0;
  u16 k;
  u16 HZnum;
  u16 TypefaceNum;
  u16 x0 = x;
  TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
  HZnum = sizeof(tfont32) / sizeof(typFNT_GB32);
  for (k = 0; k < HZnum; k++) {
    if ((tfont32[k].Index[0] == *(s)) && (tfont32[k].Index[1] == *(s + 1))) {
      LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
      for (i = 0; i < TypefaceNum; i++) {
        for (j = 0; j < 8; j++) {
          if (!mode) {
            if (tfont32[k].Msk[i] & (0x01 << j)) {
              LCD_buffer_write(fc >> 8);
              LCD_buffer_write(fc & 0xFF);
            } else {
              LCD_buffer_write(bc >> 8);
              LCD_buffer_write(bc & 0xFF);
            }
            m++;
            if (m % sizey == 0) {
              m = 0;
              break;
            }
          } else {
            if (tfont32[k].Msk[i] & (0x01 << j))
              LCD_DrawPoint(x, y, fc);
            x++;
            if ((x - x0) == sizey) {
              x = x0;
              y++;
              break;
            }
          }
        }
      }
      LCD_Flush_buffer();
    }
    continue;
  }
}


/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode) {
  u8 temp, sizex, t, m = 0;
  u16 i, TypefaceNum;
  u16 x0 = x;
  sizex = sizey / 2;
  TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
  num = num - ' ';
  LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);
  for (i = 0; i < TypefaceNum; i++) {
    if (sizey == 12)
      temp = ascii_1206[num][i];
    else if (sizey == 16)
      temp = ascii_1608[num][i];
    else if (sizey == 24)
      temp = ascii_2412[num][i];
    else if (sizey == 32)
      temp = ascii_3216[num][i];
    else
      return;
    for (t = 0; t < 8; t++) {
      if (!mode) {
        if (temp & (0x01 << t)) {
          LCD_buffer_write(fc >> 8);
          LCD_buffer_write(fc & 0xFF);
        } else {
          LCD_buffer_write(bc >> 8);
          LCD_buffer_write(bc & 0xFF);
        }
        m++;
        if (m % sizex == 0) {
          m = 0;
          break;
        }
      } else {
        if (temp & (0x01 << t))
          LCD_DrawPoint(x, y, fc);
        x++;
        if ((x - x0) == sizex) {
          x = x0;
          y++;
          break;
        }
      }
    }
  }
  LCD_Flush_buffer();
}



/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey,
                    u8 mode) {
  while (*p != '\0') {
    LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
    x += sizey / 2;
    p++;
  }
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
u32 mypow(u8 m, u8 n) {
  u32 result = 1;
  while (n--)
    result *= m;
  return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey) {
  u8 t, temp;
  u8 enshow = 0;
  u8 sizex = sizey / 2;
  for (t = 0; t < len; t++) {
    temp = (num / mypow(10, len - t - 1)) % 10;
    if (enshow == 0 && t < (len - 1)) {
      if (temp == 0) {
        LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
        continue;
      } else
        enshow = 1;
    }
    LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
  }
} 


/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc,
                       u8 sizey) {
  u8 t, temp, sizex;
  u16 num1;
  sizex = sizey / 2;
  num1 = num * 100;
  for (t = 0; t < len; t++) {
    temp = (num1 / mypow(10, len - t - 1)) % 10;
    if (t == (len - 2)) {
      LCD_ShowChar(x + (len - 2) * sizex, y, '.', fc, bc, sizey, 0);
      t++;
      len += 1;
    }
    LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
  }
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
void LCD_ShowPicture(u16 x, u16 y, u16 length, u16 width, const u8 pic[]) {
  u16 i, j;
  u32 k = 0;
  LCD_Address_Set(x, y, x + length - 1, y + width - 1);
  for (i = 0; i < length; i++) {
    for (j = 0; j < width; j++) {
      LCD_buffer_write(pic[k * 2]);
      LCD_buffer_write(pic[k * 2 + 1]);
      k++;
    }
  }
  LCD_Flush_buffer();
}

/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(u8 dat) {
  LCD_Send_Byte(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat) {
  LCD_Send_Byte(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat) {
  LCD_Send_Byte(dat >> 8);
  LCD_Send_Byte(dat & 0xFF);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat) {
  LCD_DC_Clr();
  LCD_Send_Byte(dat);
  LCD_DC_Set();
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
    if(USE_HORIZONTAL==0)
    {
        LCD_WR_REG(0x2a);//列地址设置
        LCD_WR_DATA(x1+26);
        LCD_WR_DATA(x2+26);
        LCD_WR_REG(0x2b);//行地址设置
        LCD_WR_DATA(y1+1);
        LCD_WR_DATA(y2+1);
        LCD_WR_REG(0x2c);//储存器写
    }
    else if(USE_HORIZONTAL==1)
    {
        LCD_WR_REG(0x2a);//列地址设置
        LCD_WR_DATA(x1+26);
        LCD_WR_DATA(x2+26);
        LCD_WR_REG(0x2b);//行地址设置
        LCD_WR_DATA(y1+1);
        LCD_WR_DATA(y2+1);
        LCD_WR_REG(0x2c);//储存器写
    }
    else if(USE_HORIZONTAL==2)
    {
        LCD_WR_REG(0x2a);//列地址设置
        LCD_WR_DATA(x1+1);
        LCD_WR_DATA(x2+1);
        LCD_WR_REG(0x2b);//行地址设置
        LCD_WR_DATA(y1+26);
        LCD_WR_DATA(y2+26);
        LCD_WR_REG(0x2c);//储存器写
    }
    else
    {
        LCD_WR_REG(0x2a);//列地址设置
        LCD_WR_DATA(x1+0);  //7735为0 7735S为1
        LCD_WR_DATA(x2+0);  //7735为0 7735S为1
        LCD_WR_REG(0x2b);//行地址设置
        LCD_WR_DATA(y1+24); //7735为24 7735S为26
        LCD_WR_DATA(y2+24); //7735为24 7735S为26
        LCD_WR_REG(0x2c);//储存器写
    }
}

void LCD_Init(void)
{
    
    LCD_RES_Clr();//复位
    HAL_Delay(100);
    LCD_RES_Set();    
  HAL_Delay(200);
    
    LCD_WR_REG(0x11);     //Sleep out
    HAL_Delay(120);       //Delay 120ms
    LCD_WR_REG(0x20);     //7735为20 7735S为21
    LCD_WR_REG(0xB1);     //Normal mode
    LCD_WR_DATA8(0x05);   
    LCD_WR_DATA8(0x3C);   
    LCD_WR_DATA8(0x3C);   
    LCD_WR_REG(0xB2);     //Idle mode
    LCD_WR_DATA8(0x05);   
    LCD_WR_DATA8(0x3C);   
    LCD_WR_DATA8(0x3C);   
    LCD_WR_REG(0xB3);     //Partial mode
    LCD_WR_DATA8(0x05);   
    LCD_WR_DATA8(0x3C);   
    LCD_WR_DATA8(0x3C);   
    LCD_WR_DATA8(0x05);   
    LCD_WR_DATA8(0x3C);   
    LCD_WR_DATA8(0x3C);   
    LCD_WR_REG(0xB4);     //Dot inversion
    LCD_WR_DATA8(0x03);   
    LCD_WR_REG(0xC0);     //AVDD GVDD
    LCD_WR_DATA8(0xAB);   
    LCD_WR_DATA8(0x0B);   
    LCD_WR_DATA8(0x04);   
    LCD_WR_REG(0xC1);     //VGH VGL
    LCD_WR_DATA8(0xC5);   //C0
    LCD_WR_REG(0xC2);     //Normal Mode
    LCD_WR_DATA8(0x0D);   
    LCD_WR_DATA8(0x00);   
    LCD_WR_REG(0xC3);     //Idle
    LCD_WR_DATA8(0x8D);   
    LCD_WR_DATA8(0x6A);   
    LCD_WR_REG(0xC4);     //Partial+Full
    LCD_WR_DATA8(0x8D);   
    LCD_WR_DATA8(0xEE);   
    LCD_WR_REG(0xC5);     //VCOM
    LCD_WR_DATA8(0x0F);   
    LCD_WR_REG(0xE0);     //positive gamma
    LCD_WR_DATA8(0x07);   
    LCD_WR_DATA8(0x0E);   
    LCD_WR_DATA8(0x08);   
    LCD_WR_DATA8(0x07);   
    LCD_WR_DATA8(0x10);   
    LCD_WR_DATA8(0x07);   
    LCD_WR_DATA8(0x02);   
    LCD_WR_DATA8(0x07);   
    LCD_WR_DATA8(0x09);   
    LCD_WR_DATA8(0x0F);   
    LCD_WR_DATA8(0x25);   
    LCD_WR_DATA8(0x36);   
    LCD_WR_DATA8(0x00);   
    LCD_WR_DATA8(0x08);   
    LCD_WR_DATA8(0x04);   
    LCD_WR_DATA8(0x10);   
    LCD_WR_REG(0xE1);     //negative gamma
    LCD_WR_DATA8(0x0A);   
    LCD_WR_DATA8(0x0D);   
    LCD_WR_DATA8(0x08);   
    LCD_WR_DATA8(0x07);   
    LCD_WR_DATA8(0x0F);   
    LCD_WR_DATA8(0x07);   
    LCD_WR_DATA8(0x02);   
    LCD_WR_DATA8(0x07);   
    LCD_WR_DATA8(0x09);   
    LCD_WR_DATA8(0x0F);   
    LCD_WR_DATA8(0x25);   
    LCD_WR_DATA8(0x35);   
    LCD_WR_DATA8(0x00);   
    LCD_WR_DATA8(0x09);   
    LCD_WR_DATA8(0x04);   
    LCD_WR_DATA8(0x10);    
    LCD_WR_REG(0x3A);     
    LCD_WR_DATA8(0x05);   
    LCD_WR_REG(0x36);
    if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x08);
    else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC8);
    else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x78);
    else LCD_WR_DATA8(0xA8);   
    LCD_Fill(0,0,LCD_W,LCD_H,BLACK);    
    LCD_WR_REG(0x29);     //Display on    
}



