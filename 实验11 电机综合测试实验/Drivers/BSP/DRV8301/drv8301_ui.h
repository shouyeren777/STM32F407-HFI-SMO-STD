/**
 * @file        drv8301_ui.h
 * @brief       480x800 LCD 上的 DRV8301 面板（线框图布局），逻辑对齐 TI DRV8301.c + 非阻塞按键/触摸
 */
#ifndef __DRV8301_UI_H
#define __DRV8301_UI_H

#include "./BSP/DRV8301/drv8301.h"

typedef DRV8301_handl drv8301_ui_handle_t;

void drv8301_ui_handle_init(drv8301_ui_handle_t *h);
void drv8301_ui_draw_panel(drv8301_ui_handle_t *h, uint16_t body_top, uint16_t body_h, uint16_t panel_w);
/** 请求下一次 draw 时执行 SPI 读（写寄存器后或进入 SPI 设置前可调用） */
void drv8301_ui_request_spi_read(void);
void drv8301_ui_abort(drv8301_ui_handle_t *h);
uint8_t drv8301_ui_process_key(drv8301_ui_handle_t *h, uint8_t key);
void drv8301_ui_touch_select(drv8301_ui_handle_t *h, uint16_t tx, uint16_t ty, uint16_t body_top,
                             uint16_t body_h, uint16_t panel_w);
uint8_t drv8301_ui_is_active(const drv8301_ui_handle_t *h);

#endif
