/**
 * @file        drv8301_ui.c
 * @brief       TI DRV8301.c 中菜单/显示/参数编辑在 LCD 上的实现（非阻塞），SPI 经 drv8301.c 单次 16 位帧
 */
#include "./BSP/DRV8301/drv8301_ui.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* SPI 轮询周期：提高 0X00/0X01 状态响应速度 */
#ifndef DRV8301_UI_SPI_READ_INTERVAL_MS
#define DRV8301_UI_SPI_READ_INTERVAL_MS  120U
#endif

static uint32_t s_drv8301_last_read_ms;
static uint8_t s_drv8301_have_spi_cache;
static uint8_t s_drv8301_spi_read_force;
static uint8_t s_drv8301_panel_cache_valid;
static uint16_t s_drv8301_panel_reg0;
static uint16_t s_drv8301_panel_reg1;
static uint16_t s_drv8301_panel_reg2;
static uint16_t s_drv8301_panel_reg3;
static uint8_t s_drv8301_panel_set_flag;
static uint8_t s_drv8301_panel_editing;
static uint8_t s_drv8301_panel_edit_temp;

#define SPI_ERR_LOG_CAP 9U
typedef struct
{
    char text[20];
    uint16_t color;
    uint8_t valid;
} spi_err_item_t;
static spi_err_item_t s_spi_err_log[2][SPI_ERR_LOG_CAP];
static uint8_t s_spi_err_log_next[2];
static uint32_t s_spi_err_log_count[2];
static uint8_t s_spi_err_log_dirty_full[2];
static uint8_t s_spi_err_log_dirty_slot_valid[2];
static uint8_t s_spi_err_log_dirty_slot[2];
static uint8_t s_spi_err_prev_valid;
static uint16_t s_spi_err_prev_d0;
static uint16_t s_spi_err_prev_d1;
static uint8_t s_spi_err_prev_f0;
static uint8_t s_spi_err_prev_f1;
static uint8_t s_spi_err_prev_a0;
static uint8_t s_spi_err_prev_a1;

static void drv8301_ui_spi_read_now(DRV8301_handl *h)
{
    DRV8301_Read(h);
    s_drv8301_last_read_ms = HAL_GetTick();
    s_drv8301_have_spi_cache = 1U;
    s_drv8301_spi_read_force = 0U;
}

#define SPI_LBL_IDLE    DARKBLUE
#define SPI_VAL_IDLE    BLACK
#define SPI_HEX_COLOR   CYAN
#define SPI_SEL_COLOR   RED
#define SPI_EDIT_COLOR  MAGENTA

static uint16_t ui_spi_value_color(const drv8301_ui_handle_t *h, uint8_t field_idx)
{
    if (h->set_flag != field_idx)
        return SPI_VAL_IDLE;
    return h->editing ? SPI_EDIT_COLOR : SPI_SEL_COLOR;
}

static uint16_t ui_spi_label_color(const drv8301_ui_handle_t *h, uint8_t field_idx)
{
    if (h->set_flag != field_idx)
        return SPI_LBL_IDLE;
    return h->editing ? SPI_EDIT_COLOR : SPI_SEL_COLOR;
}

static void show_str_xy(uint16_t x, uint16_t y, char *s, uint16_t color)
{
    lcd_show_string(x, y, (uint16_t)(lcddev.width - x), 24, 16, s, color);
}

static uint8_t spi_row_weight(uint8_t row)
{
    if (row == 0U)
        return 9U; /* enlarge 0X00/0X01 status zone */
    return 3U;     /* compress lower SPI config rows */
}

static uint16_t spi_row_height(uint16_t body_h, uint8_t row)
{
    uint8_t i, s;
    uint8_t prefix = 0U;
    uint8_t total = 0U;
    uint8_t w = spi_row_weight(row);
    uint32_t y0;
    uint32_t y1;
    for (i = 0U; i < 5U; i++)
        total = (uint8_t)(total + spi_row_weight(i));
    for (i = 0U; i < row; i++)
        prefix = (uint8_t)(prefix + spi_row_weight(i));
    y0 = ((uint32_t)body_h * (uint32_t)prefix) / (uint32_t)total;
    y1 = ((uint32_t)body_h * (uint32_t)(prefix + w)) / (uint32_t)total;
    return (uint16_t)(y1 - y0);
}

static uint16_t spi_row_top(uint16_t body_top, uint16_t body_h, uint8_t row)
{
    uint8_t i;
    uint8_t prefix = 0U;
    uint8_t total = 0U;
    for (i = 0U; i < 5U; i++)
        total = (uint8_t)(total + spi_row_weight(i));
    for (i = 0U; i < row; i++)
        prefix = (uint8_t)(prefix + spi_row_weight(i));
    return (uint16_t)(body_top + (uint16_t)(((uint32_t)body_h * (uint32_t)prefix) / (uint32_t)total));
}

static uint16_t spi_row_text_y(uint16_t body_top, uint16_t body_h, uint8_t row)
{
    uint16_t y0 = spi_row_top(body_top, body_h, row);
    uint16_t ht = spi_row_height(body_h, row);
    if (ht > 18U)
        return (uint16_t)(y0 + (ht - 16U) / 2U);
    return (uint16_t)(y0 + 1U);
}

static uint8_t spi_row_from_set_flag(uint8_t f)
{
    if (f >= 1U && f <= 2U)
        return 1U;
    if (f >= 3U && f <= 4U)
        return 2U;
    if (f >= 5U && f <= 6U)
        return 3U;
    if (f >= 7U && f <= 8U)
        return 4U;
    return 0U;
}

static uint8_t spi_field_from_row_half(uint8_t row, uint8_t is_right)
{
    if (is_right == 0U)
    {
        if (row == 1U)
            return 1U;
        if (row == 2U)
            return 3U;
        if (row == 3U)
            return 6U;
        if (row == 4U)
            return 7U;
    }
    else
    {
        if (row == 1U)
            return 2U;
        if (row == 2U)
            return 4U;
        if (row == 3U)
            return 5U;
        if (row == 4U)
            return 8U;
    }
    return 0U;
}

static void spi_cell_lr(uint16_t panel_w, uint8_t is_right, uint16_t *cell_l, uint16_t *cell_r)
{
    uint16_t half = (uint16_t)(panel_w / 2U);
    *cell_l = is_right ? (uint16_t)(half + 1U) : 2U;
    *cell_r = is_right ? (uint16_t)(panel_w - 3U) : (uint16_t)(half - 1U);
}

static uint16_t spi_arrow_width_for_cell(uint16_t cell_w)
{
    uint16_t aw = (uint16_t)((cell_w > 96U) ? 96U : (cell_w - 8U));
    if (aw < 24U)
        aw = 24U;
    return aw;
}

static uint8_t spi_touch_arrow_hit(uint16_t tx, uint16_t body_top, uint16_t body_h, uint16_t panel_w,
                                   uint8_t row, uint8_t is_right, uint8_t *is_up)
{
    uint16_t y0 = spi_row_top(body_top, body_h, row);
    uint16_t rh = spi_row_height(body_h, row);
    uint16_t y1 = (uint16_t)(y0 + rh - 1U);
    uint16_t cell_l;
    uint16_t cell_r;
    uint16_t cell_w = (uint16_t)(cell_r - cell_l + 1U);
    uint16_t aw;
    uint16_t ah = (uint16_t)((rh > 30U) ? 30U : (rh - 2U));
    uint16_t ax;
    uint16_t ay = (uint16_t)(y0 + ((rh > ah) ? ((rh - ah) / 2U) : 0U));
    uint16_t mid;
    if ((row < 1U) || (row > 4U))
        return 0U;
    spi_cell_lr(panel_w, is_right, &cell_l, &cell_r);
    cell_w = (uint16_t)(cell_r - cell_l + 1U);
    aw = spi_arrow_width_for_cell(cell_w);
    if ((cell_w <= 10U) || (aw < 24U) || (ah < 12U))
        return 0U;
    ax = (uint16_t)(cell_r - aw);
    if ((tx < ax) || (tx > cell_r) || (ay > y1))
        return 0U;
    if ((ah > rh) || ((uint16_t)(ay + ah - 1U) > y1))
        ah = (uint16_t)(y1 - ay + 1U);
    if ((tx >= ax) && (tx <= cell_r))
    {
        mid = (uint16_t)(ax + aw / 2U);
        *is_up = (tx > mid) ? 1U : 0U;
        return 1U;
    }
    return 0U;
}

static void spi_draw_touch_arrows(uint16_t body_top, uint16_t body_h, uint16_t panel_w, uint8_t row, uint8_t is_right)
{
    uint16_t y0 = spi_row_top(body_top, body_h, row);
    uint16_t rh = spi_row_height(body_h, row);
    uint16_t y1 = (uint16_t)(y0 + rh - 1U);
    uint16_t cell_l;
    uint16_t cell_r;
    uint16_t cell_w = (uint16_t)(cell_r - cell_l + 1U);
    uint16_t aw;
    uint16_t ah = (uint16_t)((rh > 30U) ? 30U : (rh - 2U));
    uint16_t ax;
    uint16_t ay = (uint16_t)(y0 + ((rh > ah) ? ((rh - ah) / 2U) : 0U));
    uint16_t mid;
    if ((row < 1U) || (row > 4U))
        return;
    spi_cell_lr(panel_w, is_right, &cell_l, &cell_r);
    cell_w = (uint16_t)(cell_r - cell_l + 1U);
    aw = spi_arrow_width_for_cell(cell_w);
    if ((cell_w <= 10U) || (aw < 24U) || (ah < 12U) || ay > y1)
        return;
    ax = (uint16_t)(cell_r - aw);
    if (ax <= cell_l)
        return;
    if ((ah > rh) || ((uint16_t)(ay + ah - 1U) > y1))
        ah = (uint16_t)(y1 - ay + 1U);
    mid = (uint16_t)(ax + aw / 2U);
    lcd_draw_rectangle(ax, ay, cell_r, (uint16_t)(ay + ah - 1U), BLACK);
    lcd_draw_line(mid, ay, mid, (uint16_t)(ay + ah - 1U), BLACK);
    lcd_show_string((uint16_t)(ax + 4U), (uint16_t)(ay + 2U), (uint16_t)(mid - ax - 2U), (uint16_t)(ah - 2U), 16, "v", BLACK);
    lcd_show_string((uint16_t)(mid + 2U), (uint16_t)(ay + 2U), (uint16_t)(cell_r - mid - 1U), (uint16_t)(ah - 2U), 16, "^", BLACK);
}

static void spi_draw_inner_grid(uint16_t body_top, uint16_t body_h, uint16_t w)
{
    uint16_t half = (uint16_t)(w / 2U);
    uint16_t k;
    uint16_t y_line;
    uint16_t y_bot = (uint16_t)(body_top + body_h - 1U);

    for (k = 1U; k <= 4U; k++)
    {
        y_line = spi_row_top(body_top, body_h, k);
        lcd_draw_line(1, y_line, (uint16_t)(w - 2U), y_line, BLACK);
    }
    lcd_draw_line(half, body_top, half, y_bot, BLACK);
}

static const char *voc_label(uint8_t t)
{
    static const char *const tbl[32] = {
        "0.060V", "0.068V", "0.076V", "0.086A", "0.097A", "0.109A", "0.123V", "0.138A",
        "0.155A", "0.175A", "0.197A", "0.222A", "0.250A", "0.282A", "0.317A", "0.358V",
        "0.403V", "0.454V", "0.511V", "0.576V", "0.648V", "0.730V", "0.822V", "0.926V",
        "1.043V", "1.175V", "1.324V", "1.491V", "1.679V", "1.892V", "2.131V", "2.400V",
    };
    if (t < 32U)
        return tbl[t];
    return "????";
}

void drv8301_ui_handle_init(drv8301_ui_handle_t *h)
{
    uint8_t i;
	int s = 0;
    (void)h;
    s_drv8301_last_read_ms = 0U;
    s_drv8301_have_spi_cache = 0U;
    s_drv8301_spi_read_force = 1U;
    s_drv8301_panel_cache_valid = 0U;
    s_drv8301_panel_edit_temp = 0U;
    memset(s_spi_err_log, 0, sizeof(s_spi_err_log));
    for (s = 0U; s < 2U; s++)
    {
        for (i = 0U; i < SPI_ERR_LOG_CAP; i++)
        {
            strcpy(s_spi_err_log[s][i].text, "NO_ERR");
            s_spi_err_log[s][i].color = BLACK;
            s_spi_err_log[s][i].valid = 1U;
        }
        s_spi_err_log_next[s] = 0U;
        s_spi_err_log_count[s] = 0U;
        s_spi_err_log_dirty_full[s] = 1U;
        s_spi_err_log_dirty_slot_valid[s] = 0U;
        s_spi_err_log_dirty_slot[s] = 0U;
    }
    s_spi_err_prev_valid = 0U;
    s_spi_err_prev_d0 = 0U;
    s_spi_err_prev_d1 = 0U;
    s_spi_err_prev_f0 = 0U;
    s_spi_err_prev_f1 = 0U;
    s_spi_err_prev_a0 = 0xFFU;
    s_spi_err_prev_a1 = 0xFFU;
    DRV8301_Init();
}

void drv8301_ui_request_spi_read(void)
{
    s_drv8301_spi_read_force = 1U;
}

static void load_edit_temp(drv8301_ui_handle_t *h)
{
    switch (h->set_flag)
    {
    case 1:
        h->edit_temp = (uint8_t)(h->reg2 & 3U);
        break;
    case 2:
        h->edit_temp = (uint8_t)((h->reg2 >> 3) & 1U);
        break;
    case 3:
        h->edit_temp = (uint8_t)(((h->reg2 >> 4) & 3U) | (((h->reg3 >> 4) & 4U)));
        break;
    case 4:
        h->edit_temp = (uint8_t)((h->reg2 >> 6) & 31U);
        break;
    case 5:
        h->edit_temp = (uint8_t)(h->reg3 & 3U);
        break;
    case 6:
        h->edit_temp = (uint8_t)((h->reg3 >> 2) & 3U);
        break;
    case 7:
        h->edit_temp = (uint8_t)((h->reg3 >> 4) & 1U);
        break;
    case 8:
        h->edit_temp = (uint8_t)((h->reg3 >> 5) & 1U);
        break;
    default:
        h->edit_temp = 0;
        break;
    }
}

static void apply_edit_temp(drv8301_ui_handle_t *h)
{
    switch (h->set_flag)
    {
    case 1:
        h->reg2 = (uint16_t)((h->reg2 & 0xFFFcU) | (h->edit_temp & 3U));
        break;
    case 2:
        h->reg2 = (uint16_t)((h->reg2 & 0xFFF7U) | (((uint16_t)(h->edit_temp & 1U)) << 3));
        break;
    case 3:
        h->reg2 = (uint16_t)((h->reg2 & 0xFFCFU) | (((uint16_t)(h->edit_temp & 3U)) << 4));
        h->reg3 = (uint16_t)((h->reg3 & 0xFFBFU) | (((uint16_t)(h->edit_temp & 4U)) << 4));
        break;
    case 4:
        h->reg2 = (uint16_t)((h->reg2 & 0xF83FU) | (((uint16_t)(h->edit_temp & 31U)) << 6));
        break;
    case 5:
        h->reg3 = (uint16_t)((h->reg3 & 0xFFFcU) | (h->edit_temp & 3U));
        break;
    case 6:
        h->reg3 = (uint16_t)((h->reg3 & 0xFFF3U) | (((uint16_t)(h->edit_temp & 3U)) << 2));
        break;
    case 7:
        h->reg3 = (uint16_t)((h->reg3 & 0xFFEFU) | (((uint16_t)(h->edit_temp & 1U)) << 4));
        break;
    case 8:
        h->reg3 = (uint16_t)((h->reg3 & 0xFFDFU) | (((uint16_t)(h->edit_temp & 1U)) << 5));
        break;
    default:
        break;
    }
}

static void draw_igate(drv8301_ui_handle_t *h, uint16_t x, uint16_t y)
{
    char b[8];
    uint8_t t = (uint8_t)(h->reg2 & 3U);
    if (h->set_flag == 1 && h->editing)
        t = h->edit_temp;
    switch (t)
    {
    case 0:
        strcpy(b, "1.7A ");
        break;
    case 1:
        strcpy(b, "0.7A ");
        break;
    case 2:
        strcpy(b, "0.25A");
        break;
    default:
        strcpy(b, "ERR  ");
        break;
    }
    show_str_xy(x, y, b, ui_spi_value_color(h, 1));
}

static void draw_pwmm(drv8301_ui_handle_t *h, uint16_t x, uint16_t y)
{
    char b[4];
    uint8_t t = (uint8_t)((h->reg2 >> 3) & 1U);
    if (h->set_flag == 2 && h->editing)
        t = h->edit_temp & 1U;
    b[0] = (char)(t ? '3' : '6');
    b[1] = '\0';
    show_str_xy(x, y, b, ui_spi_value_color(h, 2));
}

static void draw_ocpm(drv8301_ui_handle_t *h, uint16_t x, uint16_t y)
{
    char b[8];
    uint8_t t = (uint8_t)(((h->reg2 >> 4) & 3U) | (((h->reg3 >> 4) & 4U)));
    if (h->set_flag == 3 && h->editing)
        t = h->edit_temp;
    switch (t)
    {
    case 0:
        strcpy(b, "CBC  ");
        break;
    case 1:
        strcpy(b, "latch");
        break;
    case 2:
        strcpy(b, "Repor");
        break;
    case 3:
        strcpy(b, "Disab");
        break;
    case 4:
        strcpy(b, "off  ");
        break;
    default:
        strcpy(b, "ERR  ");
        break;
    }
    show_str_xy(x, y, b, ui_spi_value_color(h, 3));
}

static void draw_voc(drv8301_ui_handle_t *h, uint16_t x, uint16_t y)
{
    uint8_t t = (uint8_t)((h->reg2 >> 6) & 31U);
    if (h->set_flag == 4 && h->editing)
        t = h->edit_temp;
    {
        char b[12];
        strncpy(b, voc_label(t), sizeof(b) - 1);
        b[sizeof(b) - 1] = '\0';
        show_str_xy(x, y, b, ui_spi_value_color(h, 4));
    }
}

static void draw_octw(drv8301_ui_handle_t *h, uint16_t x, uint16_t y)
{
    char b[8];
    uint8_t t = (uint8_t)(h->reg3 & 3U);
    if (h->set_flag == 5 && h->editing)
        t = h->edit_temp & 3U;
    switch (t)
    {
    case 0:
        strcpy(b, "O&C");
        break;
    case 1:
        strcpy(b, "OT ");
        break;
    case 2:
        strcpy(b, "OC ");
        break;
    default:
        strcpy(b, "ERR");
        break;
    }
    show_str_xy(x, y, b, ui_spi_value_color(h, 5));
}

static void draw_gain(drv8301_ui_handle_t *h, uint16_t x, uint16_t y)
{
    char b[8];
    uint8_t t = (uint8_t)((h->reg3 >> 2) & 3U);
    if (h->set_flag == 6 && h->editing)
        t = h->edit_temp & 3U;
    switch (t)
    {
    case 0:
        strcpy(b, "10V/V");
        break;
    case 1:
        strcpy(b, "20V/V");
        break;
    case 2:
        strcpy(b, "40V/V");
        break;
    case 3:
        strcpy(b, "80V/V");
        break;
    default:
        strcpy(b, "?????");
        break;
    }
    show_str_xy(x, y, b, ui_spi_value_color(h, 6));
}

static void draw_so1(drv8301_ui_handle_t *h, uint16_t x, uint16_t y)
{
    char b[8];
    uint8_t t = (uint8_t)((h->reg3 >> 4) & 1U);
    if (h->set_flag == 7 && h->editing)
        t = h->edit_temp & 1U;
    strcpy(b, t ? "Off" : "On ");
    show_str_xy(x, y, b, ui_spi_value_color(h, 7));
}

static void draw_so2(drv8301_ui_handle_t *h, uint16_t x, uint16_t y)
{
    char b[8];
    uint8_t t = (uint8_t)((h->reg3 >> 5) & 1U);
    if (h->set_flag == 8 && h->editing)
        t = h->edit_temp & 1U;
    strcpy(b, t ? "Off" : "On ");
    show_str_xy(x, y, b, ui_spi_value_color(h, 8));
}

typedef struct
{
    const char *name;
    uint8_t bit;
} spi_bit_desc_t;

static const spi_bit_desc_t s_drv8301_sr1_bits[] = {
    {"FAULT", 10U}, {"GVDD_UV", 9U}, {"PVDD_UV", 8U}, {"OTSD", 7U},
    {"OTW", 6U}, {"FETHA_OC", 5U}, {"FETLA_OC", 4U}, {"FETHB_OC", 3U},
    {"FETLB_OC", 2U}, {"FETHC_OC", 1U}, {"FETLC_OC", 0U},
};

static const spi_bit_desc_t s_drv8301_sr2_bits[] = {
    {"GVDD_OV", 10U}, {"DID3", 9U}, {"DID2", 8U}, {"DID1", 7U}, {"DID0", 6U},
};

static void spi_draw_bit_list(uint16_t x, uint16_t y, const spi_bit_desc_t *bits, uint8_t cnt,
                              uint16_t dword, uint8_t cols, uint8_t font, uint16_t col_step, uint16_t row_step, uint16_t cell_w)
{
    uint8_t i;
    for (i = 0U; i < cnt; i++)
    {
        uint8_t row = (uint8_t)(i / cols);
        uint8_t col = (uint8_t)(i % cols);
        uint16_t cx = (uint16_t)(x + (uint16_t)col * col_step);
        uint16_t cy = (uint16_t)(y + (uint16_t)row * row_step);
        uint8_t bitv = (uint8_t)((dword >> bits[i].bit) & 1U);
        uint16_t color = bitv ? RED : BLUE;
        lcd_show_string(cx, cy, cell_w, row_step, font, (char *)bits[i].name, color);
    }
}

static uint16_t spi_err_log_color_for_count(uint32_t count)
{
    return (((count / SPI_ERR_LOG_CAP) & 1U) == 0U) ? BLACK : MAGENTA;
}

static void spi_err_log_push(uint8_t side, const char *msg)
{
    uint8_t idx;
    if (side > 1U)
        return;
    idx = s_spi_err_log_next[side];
    strncpy(s_spi_err_log[side][idx].text, msg, sizeof(s_spi_err_log[side][idx].text) - 1U);
    s_spi_err_log[side][idx].text[sizeof(s_spi_err_log[side][idx].text) - 1U] = '\0';
    s_spi_err_log[side][idx].color = spi_err_log_color_for_count(s_spi_err_log_count[side]);
    s_spi_err_log[side][idx].valid = 1U;
    s_spi_err_log_next[side] = (uint8_t)((idx + 1U) % SPI_ERR_LOG_CAP);
    s_spi_err_log_count[side]++;
    s_spi_err_log_dirty_slot_valid[side] = 1U;
    s_spi_err_log_dirty_slot[side] = idx;
}

static void spi_err_log_push_fault(uint8_t addr, const char *name)
{
    char msg[20];
    sprintf(msg, "%02X:%s", (unsigned)addr, name);
    if (addr == 0x00U)
        spi_err_log_push(0U, msg);
    else if (addr == 0x01U)
        spi_err_log_push(1U, msg);
}

static void spi_update_fault_log(uint16_t rsp0, uint16_t rsp1)
{
    uint8_t a0 = (uint8_t)((rsp0 >> 11) & 0x0FU);
    uint8_t a1 = (uint8_t)((rsp1 >> 11) & 0x0FU);
    uint16_t d0 = (uint16_t)(rsp0 & 0x07FFU);
    uint16_t d1 = (uint16_t)(rsp1 & 0x07FFU);
    uint8_t f0 = (uint8_t)((rsp0 >> 15) & 1U);
    uint8_t f1 = (uint8_t)((rsp1 >> 15) & 1U);
    uint8_t i;

    if (s_spi_err_prev_valid == 0U)
    {
        s_spi_err_prev_valid = 1U;
        s_spi_err_prev_d0 = d0;
        s_spi_err_prev_d1 = d1;
        s_spi_err_prev_f0 = f0;
        s_spi_err_prev_f1 = f1;
        s_spi_err_prev_a0 = a0;
        s_spi_err_prev_a1 = a1;
        return;
    }

    if ((a0 == 0U) && (s_spi_err_prev_a0 == 0U))
    {
        for (i = 0U; i < (uint8_t)(sizeof(s_drv8301_sr1_bits) / sizeof(s_drv8301_sr1_bits[0])); i++)
        {
            uint16_t m = (uint16_t)(1U << s_drv8301_sr1_bits[i].bit);
            if (((s_spi_err_prev_d0 & m) == 0U) && ((d0 & m) != 0U))
                spi_err_log_push_fault(0x00U, s_drv8301_sr1_bits[i].name);
        }
        if ((s_spi_err_prev_f0 == 0U) && (f0 != 0U))
            spi_err_log_push_fault(0x00U, "FRAME");
    }

    if ((a1 == 1U) && (s_spi_err_prev_a1 == 1U))
    {
        if (((s_spi_err_prev_d1 & (1U << 10)) == 0U) && ((d1 & (1U << 10)) != 0U))
            spi_err_log_push_fault(0x01U, "GVDD_OV");
        if ((s_spi_err_prev_f1 == 0U) && (f1 != 0U))
            spi_err_log_push_fault(0x01U, "FRAME");
    }

    s_spi_err_prev_d0 = d0;
    s_spi_err_prev_d1 = d1;
    s_spi_err_prev_f0 = f0;
    s_spi_err_prev_f1 = f1;
    s_spi_err_prev_a0 = a0;
    s_spi_err_prev_a1 = a1;
}

static void spi_fault_log_slot_xy(uint16_t x0, uint16_t area_w, uint16_t y_start, uint8_t slot, uint16_t *x, uint16_t *y, uint16_t *cell_w)
{
    uint16_t cols_w = (uint16_t)((area_w - 8U) / 3U);
    uint8_t r = (uint8_t)(slot / 3U);
    uint8_t c = (uint8_t)(slot % 3U);
    *x = (uint16_t)(x0 + 4U + c * cols_w);
    *y = (uint16_t)(y_start + 14U + r * 14U);
    *cell_w = (uint16_t)(cols_w - 2U);
}

static void spi_draw_fault_log_area_full(uint16_t y0, uint16_t row_h, uint16_t w)
{
    uint16_t half = (uint16_t)(w / 2U);
    uint16_t y_start = (uint16_t)(y0 + 112U);
    uint16_t y_end = (uint16_t)(y0 + row_h - 2U);
    uint16_t xl = 2U;
    uint16_t xr = (uint16_t)(half + 1U);
    uint16_t aw = (uint16_t)(half - 3U);
    uint8_t slot;
    uint8_t side;
    uint16_t x;
    uint16_t y;
    uint16_t cell_w;
    if (y_end <= y_start)
        return;
    lcd_fill(2U, y_start, (uint16_t)(w - 3U), y_end, WHITE);
    lcd_show_string((uint16_t)(xl + 4U), y_start, (uint16_t)(aw - 8U), 14U, 12U, "ERR00", DARKBLUE);
    lcd_show_string((uint16_t)(xr + 4U), y_start, (uint16_t)(aw - 8U), 14U, 12U, "ERR01", DARKBLUE);
    for (side = 0U; side < 2U; side++)
    {
        uint16_t xbase = (side == 0U) ? xl : xr;
        for (slot = 0U; slot < SPI_ERR_LOG_CAP; slot++)
        {
            spi_fault_log_slot_xy(xbase, aw, y_start, slot, &x, &y, &cell_w);
            if (s_spi_err_log[side][slot].valid != 0U)
                lcd_show_string(x, y, cell_w, 14U, 12U, s_spi_err_log[side][slot].text, s_spi_err_log[side][slot].color);
            else
                lcd_show_string(x, y, cell_w, 14U, 12U, "--", DARKBLUE);
        }
    }
    s_spi_err_log_dirty_full[0] = 0U;
    s_spi_err_log_dirty_full[1] = 0U;
    s_spi_err_log_dirty_slot_valid[0] = 0U;
    s_spi_err_log_dirty_slot_valid[1] = 0U;
}

static void spi_draw_fault_log_area_slot(uint16_t y0, uint16_t row_h, uint16_t w, uint8_t side, uint8_t slot)
{
    uint16_t half = (uint16_t)(w / 2U);
    uint16_t y_start = (uint16_t)(y0 + 112U);
    uint16_t y_end = (uint16_t)(y0 + row_h - 2U);
    uint16_t xbase = (side == 0U) ? 2U : (uint16_t)(half + 1U);
    uint16_t aw = (uint16_t)(half - 3U);
    uint16_t x;
    uint16_t y;
    uint16_t cell_w;
    if ((side > 1U) || (slot >= SPI_ERR_LOG_CAP) || (y_end <= y_start))
        return;
    spi_fault_log_slot_xy(xbase, aw, y_start, slot, &x, &y, &cell_w);
    lcd_fill(x, y, (uint16_t)(x + cell_w - 1U), (uint16_t)(y + 13U), WHITE);
    if (s_spi_err_log[side][slot].valid != 0U)
        lcd_show_string(x, y, cell_w, 14U, 12U, s_spi_err_log[side][slot].text, s_spi_err_log[side][slot].color);
    else
        lcd_show_string(x, y, cell_w, 14U, 12U, "--", DARKBLUE);
    s_spi_err_log_dirty_slot_valid[side] = 0U;
}

#define SPI_SIDE_LEFT   0x01U
#define SPI_SIDE_RIGHT  0x02U
#define SPI_SIDE_BOTH   (SPI_SIDE_LEFT | SPI_SIDE_RIGHT)

static uint8_t spi_side_mask_from_set_flag(uint8_t set_flag)
{
    if ((set_flag == 1U) || (set_flag == 3U) || (set_flag == 6U) || (set_flag == 7U))
        return SPI_SIDE_LEFT;
    if ((set_flag == 2U) || (set_flag == 4U) || (set_flag == 5U) || (set_flag == 8U))
        return SPI_SIDE_RIGHT;
    return 0U;
}

static void spi_cell_clear_and_band(const drv8301_ui_handle_t *h, uint16_t body_top, uint16_t body_h, uint16_t w, uint8_t row, uint8_t side_mask)
{
    uint16_t half = (uint16_t)(w / 2U);
    uint16_t y_a = spi_row_top(body_top, body_h, row);
    uint16_t y_b = (uint16_t)(y_a + spi_row_height(body_h, row) - 1U);
    uint16_t x_l = 2U;
    uint16_t x_r = (uint16_t)(w - 3U);
    uint16_t x1;
    uint16_t x2;
    uint8_t sel_row = spi_row_from_set_flag(h->set_flag);
    uint8_t sel_side = spi_side_mask_from_set_flag(h->set_flag);
    if (y_b <= y_a)
        return;
    if ((side_mask & SPI_SIDE_LEFT) != 0U)
    {
        x1 = x_l;
        x2 = (uint16_t)(half - 1U);
        if (x2 > x1)
            lcd_fill(x1, y_a, x2, y_b, WHITE);
        if ((row == sel_row) && ((sel_side & SPI_SIDE_LEFT) != 0U))
        {
            uint16_t band_c = h->editing ? LGRAYBLUE : LIGHTBLUE;
            if (x2 > x1)
                lcd_fill(x1, y_a, x2, y_b, band_c);
        }
    }
    if ((side_mask & SPI_SIDE_RIGHT) != 0U)
    {
        x1 = (uint16_t)(half + 1U);
        x2 = x_r;
        if (x2 > x1)
            lcd_fill(x1, y_a, x2, y_b, WHITE);
        if ((row == sel_row) && ((sel_side & SPI_SIDE_RIGHT) != 0U))
        {
            uint16_t band_c = h->editing ? LGRAYBLUE : LIGHTBLUE;
            if (x2 > x1)
                lcd_fill(x1, y_a, x2, y_b, band_c);
        }
    }
}

static void spi_draw_row0(drv8301_ui_handle_t *h, uint16_t body_top, uint16_t body_h, uint16_t w, uint8_t side_mask)
{
    char line[28];
    uint16_t rsp0 = h->reg0;
    uint16_t rsp1 = h->reg1;
    uint8_t f0 = (uint8_t)((rsp0 >> 15) & 1U);
    uint8_t f1 = (uint8_t)((rsp1 >> 15) & 1U);
    uint8_t a0 = (uint8_t)((rsp0 >> 11) & 0x0FU);
    uint8_t a1 = (uint8_t)((rsp1 >> 11) & 0x0FU);
    uint16_t d0 = (uint16_t)(rsp0 & 0x07FFU);
    uint16_t d1 = (uint16_t)(rsp1 & 0x07FFU);
    uint16_t half = (uint16_t)(w / 2U);
    uint16_t y0 = spi_row_top(body_top, body_h, 0U);
    uint16_t row_h = spi_row_height(body_h, 0U);
    spi_cell_clear_and_band(h, body_top, body_h, w, 0U, side_mask);
    if ((side_mask & SPI_SIDE_LEFT) != 0U)
    {
        lcd_show_string(8U, (uint16_t)(y0 + 6U), (uint16_t)(half - 8U), 20U, 16U, "0X00", SPI_LBL_IDLE);
        if (a0 == 0U)
        {
            sprintf(line, "D:%03X F:%u", (unsigned)d0, (unsigned)f0);
            lcd_show_string(60U, (uint16_t)(y0 + 6U), (uint16_t)(half - 60U), 20U, 16U, line, SPI_HEX_COLOR);
            spi_draw_bit_list(8U, (uint16_t)(y0 + 30U), s_drv8301_sr1_bits,
                              (uint8_t)(sizeof(s_drv8301_sr1_bits) / sizeof(s_drv8301_sr1_bits[0])),
                              d0, 3U, 16U, 76U, 22U, 74U);
        }
        else
        {
            sprintf(line, "A:%X ERR", (unsigned)a0);
            lcd_show_string(60U, (uint16_t)(y0 + 6U), (uint16_t)(half - 60U), 20U, 16U, line, RED);
        }
    }
    if ((side_mask & SPI_SIDE_RIGHT) != 0U)
    {
        uint16_t xr = (uint16_t)(half + 6U);
        lcd_show_string(xr, (uint16_t)(y0 + 6U), (uint16_t)(half - 6U), 20U, 16U, "0X01", SPI_LBL_IDLE);
        if (a1 == 1U)
        {
            sprintf(line, "D:%03X F:%u", (unsigned)d1, (unsigned)f1);
            lcd_show_string((uint16_t)(xr + 52U), (uint16_t)(y0 + 6U), (uint16_t)(half - 58U), 20U, 16U, line, SPI_HEX_COLOR);
            spi_draw_bit_list(xr, (uint16_t)(y0 + 30U), s_drv8301_sr2_bits,
                              (uint8_t)(sizeof(s_drv8301_sr2_bits) / sizeof(s_drv8301_sr2_bits[0])),
                              d1, 2U, 16U, 112U, 24U, 110U);
        }
        else
        {
            sprintf(line, "A:%X ERR", (unsigned)a1);
            lcd_show_string((uint16_t)(xr + 52U), (uint16_t)(y0 + 6U), (uint16_t)(half - 58U), 20U, 16U, line, RED);
        }
    }
    if ((side_mask == SPI_SIDE_BOTH) || (s_spi_err_log_dirty_full[0] != 0U) || (s_spi_err_log_dirty_full[1] != 0U))
        spi_draw_fault_log_area_full(y0, row_h, w);
    else
    {
        if (s_spi_err_log_dirty_slot_valid[0] != 0U)
            spi_draw_fault_log_area_slot(y0, row_h, w, 0U, s_spi_err_log_dirty_slot[0]);
        if (s_spi_err_log_dirty_slot_valid[1] != 0U)
            spi_draw_fault_log_area_slot(y0, row_h, w, 1U, s_spi_err_log_dirty_slot[1]);
    }
}

static void spi_draw_row1(drv8301_ui_handle_t *h, uint16_t body_top, uint16_t body_h, uint16_t w, uint8_t side_mask)
{
    uint16_t y_r = spi_row_text_y(body_top, body_h, 1U);
    uint16_t cell_l;
    uint16_t cell_r;
    uint16_t cell_w;
    uint16_t ax;
    uint16_t text_r;
    spi_cell_clear_and_band(h, body_top, body_h, w, 1U, side_mask);
    if ((side_mask & SPI_SIDE_LEFT) != 0U)
    {
        spi_cell_lr(w, 0U, &cell_l, &cell_r);
        cell_w = (uint16_t)(cell_r - cell_l + 1U);
        ax = (uint16_t)(cell_r - spi_arrow_width_for_cell(cell_w));
        text_r = (ax > cell_l + 4U) ? (uint16_t)(ax - 4U) : cell_r;
        lcd_show_string((uint16_t)(cell_l + 4U), y_r, (uint16_t)(text_r - cell_l), 20, 16, "I_GATE:", ui_spi_label_color(h, 1));
        draw_igate(h, (uint16_t)(cell_l + 62U), y_r);
        spi_draw_touch_arrows(body_top, body_h, w, 1U, 0U);
    }
    if ((side_mask & SPI_SIDE_RIGHT) != 0U)
    {
        spi_cell_lr(w, 1U, &cell_l, &cell_r);
        cell_w = (uint16_t)(cell_r - cell_l + 1U);
        ax = (uint16_t)(cell_r - spi_arrow_width_for_cell(cell_w));
        text_r = (ax > cell_l + 4U) ? (uint16_t)(ax - 4U) : cell_r;
        lcd_show_string((uint16_t)(cell_l + 4U), y_r, (uint16_t)(text_r - cell_l), 20, 16, "PWM_MODE:", ui_spi_label_color(h, 2));
        draw_pwmm(h, (uint16_t)(cell_l + 78U), y_r);
        spi_draw_touch_arrows(body_top, body_h, w, 1U, 1U);
    }
}

static void spi_draw_row2(drv8301_ui_handle_t *h, uint16_t body_top, uint16_t body_h, uint16_t w, uint8_t side_mask)
{
    uint16_t y_r = spi_row_text_y(body_top, body_h, 2U);
    uint16_t cell_l;
    uint16_t cell_r;
    uint16_t cell_w;
    uint16_t ax;
    uint16_t text_r;
    spi_cell_clear_and_band(h, body_top, body_h, w, 2U, side_mask);
    if ((side_mask & SPI_SIDE_LEFT) != 0U)
    {
        spi_cell_lr(w, 0U, &cell_l, &cell_r);
        cell_w = (uint16_t)(cell_r - cell_l + 1U);
        ax = (uint16_t)(cell_r - spi_arrow_width_for_cell(cell_w));
        text_r = (ax > cell_l + 4U) ? (uint16_t)(ax - 4U) : cell_r;
        lcd_show_string((uint16_t)(cell_l + 4U), y_r, (uint16_t)(text_r - cell_l), 20, 16, "OCP_MODE:", ui_spi_label_color(h, 3));
        draw_ocpm(h, (uint16_t)(cell_l + 80U), y_r);
        spi_draw_touch_arrows(body_top, body_h, w, 2U, 0U);
    }
    if ((side_mask & SPI_SIDE_RIGHT) != 0U)
    {
        spi_cell_lr(w, 1U, &cell_l, &cell_r);
        cell_w = (uint16_t)(cell_r - cell_l + 1U);
        ax = (uint16_t)(cell_r - spi_arrow_width_for_cell(cell_w));
        text_r = (ax > cell_l + 4U) ? (uint16_t)(ax - 4U) : cell_r;
        lcd_show_string((uint16_t)(cell_l + 4U), y_r, (uint16_t)(text_r - cell_l), 20, 16, "V_OC:", ui_spi_label_color(h, 4));
        draw_voc(h, (uint16_t)(cell_l + 54U), y_r);
        spi_draw_touch_arrows(body_top, body_h, w, 2U, 1U);
    }
}

static void spi_draw_row3(drv8301_ui_handle_t *h, uint16_t body_top, uint16_t body_h, uint16_t w, uint8_t side_mask)
{
    uint16_t y_r = spi_row_text_y(body_top, body_h, 3U);
    uint16_t cell_l;
    uint16_t cell_r;
    uint16_t cell_w;
    uint16_t ax;
    uint16_t text_r;
    spi_cell_clear_and_band(h, body_top, body_h, w, 3U, side_mask);
    if ((side_mask & SPI_SIDE_LEFT) != 0U)
    {
        spi_cell_lr(w, 0U, &cell_l, &cell_r);
        cell_w = (uint16_t)(cell_r - cell_l + 1U);
        ax = (uint16_t)(cell_r - spi_arrow_width_for_cell(cell_w));
        text_r = (ax > cell_l + 4U) ? (uint16_t)(ax - 4U) : cell_r;
        lcd_show_string((uint16_t)(cell_l + 4U), y_r, (uint16_t)(text_r - cell_l), 20, 16, "GAIN:", ui_spi_label_color(h, 6));
        draw_gain(h, (uint16_t)(cell_l + 52U), y_r);
        spi_draw_touch_arrows(body_top, body_h, w, 3U, 0U);
    }
    if ((side_mask & SPI_SIDE_RIGHT) != 0U)
    {
        spi_cell_lr(w, 1U, &cell_l, &cell_r);
        cell_w = (uint16_t)(cell_r - cell_l + 1U);
        ax = (uint16_t)(cell_r - spi_arrow_width_for_cell(cell_w));
        text_r = (ax > cell_l + 4U) ? (uint16_t)(ax - 4U) : cell_r;
        lcd_show_string((uint16_t)(cell_l + 4U), y_r, (uint16_t)(text_r - cell_l), 20, 16, "OCTW_MODE:", ui_spi_label_color(h, 5));
        draw_octw(h, (uint16_t)(cell_l + 84U), y_r);
        spi_draw_touch_arrows(body_top, body_h, w, 3U, 1U);
    }
}

static void spi_draw_row4(drv8301_ui_handle_t *h, uint16_t body_top, uint16_t body_h, uint16_t w, uint8_t side_mask)
{
    uint16_t y_r = spi_row_text_y(body_top, body_h, 4U);
    uint16_t cell_l;
    uint16_t cell_r;
    uint16_t cell_w;
    uint16_t ax;
    uint16_t text_r;
    spi_cell_clear_and_band(h, body_top, body_h, w, 4U, side_mask);
    if ((side_mask & SPI_SIDE_LEFT) != 0U)
    {
        spi_cell_lr(w, 0U, &cell_l, &cell_r);
        cell_w = (uint16_t)(cell_r - cell_l + 1U);
        ax = (uint16_t)(cell_r - spi_arrow_width_for_cell(cell_w));
        text_r = (ax > cell_l + 4U) ? (uint16_t)(ax - 4U) : cell_r;
        lcd_show_string((uint16_t)(cell_l + 4U), y_r, (uint16_t)(text_r - cell_l), 20, 16, "S01:", ui_spi_label_color(h, 7));
        draw_so1(h, (uint16_t)(cell_l + 44U), y_r);
        spi_draw_touch_arrows(body_top, body_h, w, 4U, 0U);
    }
    if ((side_mask & SPI_SIDE_RIGHT) != 0U)
    {
        spi_cell_lr(w, 1U, &cell_l, &cell_r);
        cell_w = (uint16_t)(cell_r - cell_l + 1U);
        ax = (uint16_t)(cell_r - spi_arrow_width_for_cell(cell_w));
        text_r = (ax > cell_l + 4U) ? (uint16_t)(ax - 4U) : cell_r;
        lcd_show_string((uint16_t)(cell_l + 4U), y_r, (uint16_t)(text_r - cell_l), 20, 16, "S02:", ui_spi_label_color(h, 8));
        draw_so2(h, (uint16_t)(cell_l + 44U), y_r);
        spi_draw_touch_arrows(body_top, body_h, w, 4U, 1U);
    }
}

static void key_up(drv8301_ui_handle_t *h);
static void key_down(drv8301_ui_handle_t *h);

void drv8301_ui_abort(drv8301_ui_handle_t *h)
{
    h->set_flag = 0U;
    h->editing = 0U;
}

void drv8301_ui_draw_panel(drv8301_ui_handle_t *h, uint16_t body_top, uint16_t body_h, uint16_t panel_w)
{
    uint16_t y0 = body_top;
    uint16_t w = panel_w;
    uint16_t yb = (uint16_t)(body_top + body_h);
    uint8_t need_redraw;
    uint8_t row_side_dirty[5] = {0U, 0U, 0U, 0U, 0U};
    uint8_t old_sel_row;
    uint8_t new_sel_row;
    uint8_t old_sel_side;
    uint8_t new_sel_side;
    uint8_t did_read = 0U;
    uint16_t row0_top;
    uint16_t row0_h;

    if (yb > lcddev.height - 2U)
        yb = (uint16_t)(lcddev.height - 2U);

    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed = (uint32_t)(now - s_drv8301_last_read_ms);
        if ((s_drv8301_spi_read_force != 0U) || (s_drv8301_have_spi_cache == 0U) ||
            (elapsed >= (uint32_t)DRV8301_UI_SPI_READ_INTERVAL_MS))
        {
            drv8301_ui_spi_read_now(h);
            did_read = 1U;
        }
    }
    if (did_read != 0U)
        spi_update_fault_log(h->reg0, h->reg1);

    need_redraw = (uint8_t)(
        (s_drv8301_panel_cache_valid == 0U) ||
        (s_drv8301_panel_reg0 != h->reg0) ||
        (s_drv8301_panel_reg1 != h->reg1) ||
        (s_drv8301_panel_reg2 != h->reg2) ||
        (s_drv8301_panel_reg3 != h->reg3) ||
        (s_drv8301_panel_set_flag != h->set_flag) ||
        (s_drv8301_panel_editing != h->editing) ||
        (s_drv8301_panel_edit_temp != h->edit_temp) ||
        (s_spi_err_log_dirty_full[0] != 0U) || (s_spi_err_log_dirty_full[1] != 0U) ||
        (s_spi_err_log_dirty_slot_valid[0] != 0U) || (s_spi_err_log_dirty_slot_valid[1] != 0U));
    if (need_redraw == 0U)
        return;

    if (s_drv8301_panel_cache_valid == 0U)
    {
        if (yb > y0)
            lcd_fill(1U, y0, (uint16_t)(w - 2U), yb, WHITE);
        spi_draw_inner_grid(body_top, body_h, w);
        spi_draw_row0(h, body_top, body_h, w, SPI_SIDE_BOTH);
        spi_draw_row1(h, body_top, body_h, w, SPI_SIDE_BOTH);
        spi_draw_row2(h, body_top, body_h, w, SPI_SIDE_BOTH);
        spi_draw_row3(h, body_top, body_h, w, SPI_SIDE_BOTH);
        spi_draw_row4(h, body_top, body_h, w, SPI_SIDE_BOTH);
        lcd_show_string(8U, (uint16_t)(body_top + body_h - 14U), (uint16_t)(w - 16U), 14, 12, "KEYS=SPI Touch=motor", DARKBLUE);
    }
    else
    {
        old_sel_row = spi_row_from_set_flag(s_drv8301_panel_set_flag);
        new_sel_row = spi_row_from_set_flag(h->set_flag);
        old_sel_side = spi_side_mask_from_set_flag(s_drv8301_panel_set_flag);
        new_sel_side = spi_side_mask_from_set_flag(h->set_flag);

        if (s_drv8301_panel_reg0 != h->reg0)
            row_side_dirty[0] |= SPI_SIDE_LEFT;
        if (s_drv8301_panel_reg1 != h->reg1)
            row_side_dirty[0] |= SPI_SIDE_RIGHT;
        if (s_drv8301_panel_reg2 != h->reg2)
        {
            row_side_dirty[1] |= SPI_SIDE_BOTH;
            row_side_dirty[2] |= SPI_SIDE_BOTH;
        }
        if (s_drv8301_panel_reg3 != h->reg3)
        {
            row_side_dirty[2] |= SPI_SIDE_LEFT;
            row_side_dirty[3] |= SPI_SIDE_BOTH;
            row_side_dirty[4] |= SPI_SIDE_BOTH;
        }
        if ((old_sel_row >= 1U) && (old_sel_row <= 4U))
            row_side_dirty[old_sel_row] |= (old_sel_side != 0U) ? old_sel_side : SPI_SIDE_BOTH;
        if ((new_sel_row >= 1U) && (new_sel_row <= 4U))
            row_side_dirty[new_sel_row] |= (new_sel_side != 0U) ? new_sel_side : SPI_SIDE_BOTH;
        if ((h->editing != 0U) && (s_drv8301_panel_edit_temp != h->edit_temp) &&
            (new_sel_row >= 1U) && (new_sel_row <= 4U))
            row_side_dirty[new_sel_row] |= (new_sel_side != 0U) ? new_sel_side : SPI_SIDE_BOTH;
        if ((s_spi_err_log_dirty_full[0] != 0U) || (s_spi_err_log_dirty_full[1] != 0U))
            row_side_dirty[0] |= SPI_SIDE_BOTH;

        if (row_side_dirty[0] != 0U)
            spi_draw_row0(h, body_top, body_h, w, row_side_dirty[0]);
        if (row_side_dirty[1] != 0U)
            spi_draw_row1(h, body_top, body_h, w, row_side_dirty[1]);
        if (row_side_dirty[2] != 0U)
            spi_draw_row2(h, body_top, body_h, w, row_side_dirty[2]);
        if (row_side_dirty[3] != 0U)
            spi_draw_row3(h, body_top, body_h, w, row_side_dirty[3]);
        if (row_side_dirty[4] != 0U)
            spi_draw_row4(h, body_top, body_h, w, row_side_dirty[4]);

        if ((row_side_dirty[0] == 0U) && ((s_spi_err_log_dirty_slot_valid[0] != 0U) || (s_spi_err_log_dirty_slot_valid[1] != 0U)))
        {
            row0_top = spi_row_top(body_top, body_h, 0U);
            row0_h = spi_row_height(body_h, 0U);
            if (s_spi_err_log_dirty_slot_valid[0] != 0U)
                spi_draw_fault_log_area_slot(row0_top, row0_h, w, 0U, s_spi_err_log_dirty_slot[0]);
            if (s_spi_err_log_dirty_slot_valid[1] != 0U)
                spi_draw_fault_log_area_slot(row0_top, row0_h, w, 1U, s_spi_err_log_dirty_slot[1]);
        }
    }

    s_drv8301_panel_reg0 = h->reg0;
    s_drv8301_panel_reg1 = h->reg1;
    s_drv8301_panel_reg2 = h->reg2;
    s_drv8301_panel_reg3 = h->reg3;
    s_drv8301_panel_set_flag = h->set_flag;
    s_drv8301_panel_editing = h->editing;
    s_drv8301_panel_edit_temp = h->edit_temp;
    s_drv8301_panel_cache_valid = 1U;
}

uint8_t drv8301_ui_is_active(const drv8301_ui_handle_t *h)
{
    return (uint8_t)((h->set_flag != 0U) || (h->editing != 0U));
}

void drv8301_ui_touch_select(drv8301_ui_handle_t *h, uint16_t tx, uint16_t ty, uint16_t body_top,
                             uint16_t body_h, uint16_t panel_w)
{
    uint16_t rem;
    uint8_t row;
    uint16_t ht;
    uint8_t r;
    uint8_t is_right;
    uint8_t field;
    uint8_t is_up;
    uint8_t hit_arrow;

    if (ty < body_top)
        return;

    rem = (uint16_t)(ty - body_top);
    if (rem >= body_h)
        return;

    row = 0U;
    for (r = 0U; r < 5U; r++)
    {
        ht = spi_row_height(body_h, r);
        if (rem < ht)
        {
            row = r;
            break;
        }
        rem = (uint16_t)(rem - ht);
    }

    if (row == 0U)
        return;
    is_right = (tx >= (uint16_t)(panel_w / 2U)) ? 1U : 0U;
    field = spi_field_from_row_half(row, is_right);
    if (field == 0U)
        return;

    hit_arrow = spi_touch_arrow_hit(tx, body_top, body_h, panel_w, row, is_right, &is_up);
    if (hit_arrow != 0U)
    {
        if (h->set_flag != field)
        {
            h->set_flag = field;
            h->editing = 1U;
            load_edit_temp(h);
        }
        else if (h->editing == 0U)
        {
            h->editing = 1U;
            load_edit_temp(h);
        }

        if (is_up != 0U)
            key_up(h);
        else
            key_down(h);
    }
    else
    {
        if (h->set_flag != field)
        {
            h->set_flag = field;
            h->editing = 0U;
        }
        else if (h->editing != 0U)
        {
            apply_edit_temp(h);
            DRV8301_Senddata(h);
            h->editing = 0U;
            h->set_flag = 0U;
        }
    }

    drv8301_ui_request_spi_read();
}

static void key_up(drv8301_ui_handle_t *h)
{
    switch (h->set_flag)
    {
    case 1:
        if (h->edit_temp < 2U)
            h->edit_temp++;
        else
            h->edit_temp = 0U;
        break;
    case 2:
        h->edit_temp ^= 1U;
        break;
    case 3:
        if (h->edit_temp < 4U)
            h->edit_temp++;
        else
            h->edit_temp = 0U;
        break;
    case 4:
        if (h->edit_temp < 31U)
            h->edit_temp++;
        else
            h->edit_temp = 0U;
        break;
    case 5:
        if (h->edit_temp < 2U)
            h->edit_temp++;
        else
            h->edit_temp = 0U;
        break;
    case 6:
        /* 与 TI 原版一致：从最大值加一圈回到 2 而非 0 */
        if (h->edit_temp < 3U)
            h->edit_temp++;
        else
            h->edit_temp = 2U;
        break;
    case 7:
    case 8:
        h->edit_temp ^= 1U;
        break;
    default:
        break;
    }
}

static void key_down(drv8301_ui_handle_t *h)
{
    switch (h->set_flag)
    {
    case 1:
        if (h->edit_temp > 0U)
            h->edit_temp--;
        else
            h->edit_temp = 2U;
        break;
    case 2:
        h->edit_temp ^= 1U;
        break;
    case 3:
        if (h->edit_temp > 0U)
            h->edit_temp--;
        else
            h->edit_temp = 4U;
        break;
    case 4:
        if (h->edit_temp > 0U)
            h->edit_temp--;
        else
            h->edit_temp = 31U;
        break;
    case 5:
        if (h->edit_temp > 0U)
            h->edit_temp--;
        else
            h->edit_temp = 2U;
        break;
    case 6:
        if (h->edit_temp > 0U)
            h->edit_temp--;
        else
            h->edit_temp = 3U;
        break;
    case 7:
    case 8:
        h->edit_temp ^= 1U;
        break;
    default:
        break;
    }
}

uint8_t drv8301_ui_process_key(drv8301_ui_handle_t *h, uint8_t key)
{
    if (h->set_flag == 0U)
        return 0U;

    if (key == KEY0_PRES)
    {
        if (!h->editing)
        {
            h->editing = 1U;
            drv8301_ui_spi_read_now(h);
            load_edit_temp(h);
        }
        else
        {
            apply_edit_temp(h);
            DRV8301_Senddata(h);
            drv8301_ui_request_spi_read();
            h->editing = 0U;
            h->set_flag = 0U;
        }
        return 1U;
    }

    if (!h->editing)
        return 1U;

    if (key == KEY2_PRES)
    {
        key_up(h);
        return 1U;
    }
    if (key == KEY1_PRES)
    {
        key_down(h);
        return 1U;
    }

    return 1U;
}
