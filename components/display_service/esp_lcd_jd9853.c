// JD9853 面板驱动实现（2.01" 240x296 SPI TFT）
//
// 结构完全对齐 ESP-IDF 内置 panel 驱动（esp_lcd_panel_st7789.c）：
// 一个 jd9853_panel_t 内嵌 esp_lcd_panel_t 作为基类，通过 __containerof 反查自身，
// 再把成员函数指针填进基类的函数表，交给 esp_lcd_panel_ops 通用 API 调用。
#include <assert.h>
#include <stdlib.h>
#include <sys/cdefs.h>

#include "display_service/esp_lcd_jd9853.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "jd9853";

// 厂商初始化命令：cmd + 参数 + 参数长度 + 执行后延时
typedef struct {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t data_bytes;
    uint16_t delay_ms;
} jd9853_init_cmd_t;

// 2.01" 240x296 IPS 面板初始化序列。
// 关键点：
//   - 0xDF 0x98 0x53 连发两次解锁厂商寄存器页，0xDE 切换命令页（0/1/2）
//   - 0x3A = 0x05 指定 RGB565（0x06 为 RGB666，本工程用 16bpp）
//   - 0x2A/0x2B 只是设定初始窗口，每次刷新由 draw_bitmap 重新下发 CASET/RASET
static const jd9853_init_cmd_t kInitCmds[] = {
    {0x11, (const uint8_t[]){0x00}, 0, 120}, // SLPOUT
    {0xDF, (const uint8_t[]){0x98, 0x53}, 2, 0},
    {0xDF, (const uint8_t[]){0x98, 0x53}, 2, 0},
    {0xB2, (const uint8_t[]){0x23}, 1, 0},
    {0xB7, (const uint8_t[]){0x00, 0x47, 0x00, 0x6F}, 4, 0},
    {0xBB, (const uint8_t[]){0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0}, 6, 0},
    {0xC0, (const uint8_t[]){0x44, 0xA4}, 2, 0},
    {0xC1, (const uint8_t[]){0x16}, 1, 0},
    {0xC3, (const uint8_t[]){0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77}, 8, 0},
    {0xC4, (const uint8_t[]){0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82}, 12, 0},
    {0xC8, (const uint8_t[]){0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17,
                             0x12, 0x0D, 0x04, 0x00, 0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28,
                             0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00}, 32, 0}, // 正/负 Gamma
    {0xD0, (const uint8_t[]){0x04, 0x06, 0x6B, 0x0F, 0x00}, 5, 0},
    {0xD7, (const uint8_t[]){0x00, 0x30}, 2, 0},
    {0xE6, (const uint8_t[]){0x14}, 1, 0},
    {0xDE, (const uint8_t[]){0x01}, 1, 0},                                           // 切到命令页 1
    {0xB7, (const uint8_t[]){0x03, 0x13, 0xEF, 0x35, 0x35}, 5, 0},
    {0xC1, (const uint8_t[]){0x14, 0x15, 0xC0}, 3, 0},
    {0xC2, (const uint8_t[]){0x06, 0x3A}, 2, 0},
    {0xC4, (const uint8_t[]){0x72, 0x12}, 2, 0},
    {0xBE, (const uint8_t[]){0x00}, 1, 0},
    {0xDE, (const uint8_t[]){0x02}, 1, 0}, // 切到命令页 2
    {0xE5, (const uint8_t[]){0x00, 0x02, 0x00}, 3, 0},
    {0xE5, (const uint8_t[]){0x01, 0x02, 0x00}, 3, 0},
    {0xDE, (const uint8_t[]){0x00}, 1, 0}, // 回到命令页 0
    {0x35, (const uint8_t[]){0x00}, 1, 0}, // 撕裂效应输出（TE）开启
    {0x3A, (const uint8_t[]){0x05}, 1, 0}, // COLMOD: RGB565
    {0x2A, (const uint8_t[]){0x00, 0x00, 0x00, 0xEF}, 4, 0}, // CASET: 0 ~ 239
    {0x2B, (const uint8_t[]){0x00, 0x00, 0x01, 0x3F}, 4, 0}, // RASET: 0 ~ 319
    {0xDE, (const uint8_t[]){0x02}, 1, 0},
    {0xE5, (const uint8_t[]){0x00, 0x02, 0x00}, 3, 0},
    {0xDE, (const uint8_t[]){0x00}, 1, 0},
    {0x29, (const uint8_t[]){0x00}, 0, 0}, // DISPON
};

// 面板私有数据：base 必须为首成员，才能用 __containerof 从基类指针反查回自身
typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // MADCTL 当前值（RGB 顺序 / 镜像 / 行列交换）
    uint8_t colmod_val; // COLMOD 当前值（像素格式）
} jd9853_panel_t;

static esp_err_t panel_jd9853_del(esp_lcd_panel_t *panel);
static esp_err_t panel_jd9853_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_jd9853_init(esp_lcd_panel_t *panel);
static esp_err_t panel_jd9853_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start,
                                          int x_end, int y_end, const void *color_data);
static esp_err_t panel_jd9853_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_jd9853_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_jd9853_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_jd9853_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_jd9853_disp_on_off(esp_lcd_panel_t *panel, bool on_off);
static esp_err_t panel_jd9853_disp_sleep(esp_lcd_panel_t *panel, bool sleep);

esp_err_t esp_lcd_new_panel_jd9853(const esp_lcd_panel_io_handle_t io,
                                   const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    jd9853_panel_t *jd9853 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG,
                      err, TAG, "invalid argument");
    jd9853 = (jd9853_panel_t *)calloc(1, sizeof(jd9853_panel_t));
    ESP_GOTO_ON_FALSE(jd9853, ESP_ERR_NO_MEM, err, TAG, "no mem for jd9853 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_ele_order) {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        jd9853->madctl_val = 0;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        jd9853->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported rgb element order");
        break;
    }

    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        jd9853->colmod_val = 0x55;
        jd9853->fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666：每个分量占 1 字节高 6 位，共 3 字节
        jd9853->colmod_val = 0x66;
        jd9853->fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    jd9853->io = io;
    jd9853->reset_gpio_num = panel_dev_config->reset_gpio_num;
    jd9853->reset_level = panel_dev_config->flags.reset_active_high;

    jd9853->base.del = panel_jd9853_del;
    jd9853->base.reset = panel_jd9853_reset;
    jd9853->base.init = panel_jd9853_init;
    jd9853->base.draw_bitmap = panel_jd9853_draw_bitmap;
    jd9853->base.invert_color = panel_jd9853_invert_color;
    jd9853->base.set_gap = panel_jd9853_set_gap;
    jd9853->base.mirror = panel_jd9853_mirror;
    jd9853->base.swap_xy = panel_jd9853_swap_xy;
    jd9853->base.disp_on_off = panel_jd9853_disp_on_off;
    jd9853->base.disp_sleep = panel_jd9853_disp_sleep;

    *ret_panel = &(jd9853->base);
    ESP_LOGD(TAG, "new jd9853 panel @%p", jd9853);
    return ESP_OK;

err:
    if (jd9853) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(jd9853);
    }
    return ret;
}

static esp_err_t panel_jd9853_del(esp_lcd_panel_t *panel)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    if (jd9853->reset_gpio_num >= 0) {
        gpio_reset_pin(jd9853->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del jd9853 panel @%p", jd9853);
    free(jd9853);
    return ESP_OK;
}

static esp_err_t panel_jd9853_reset(esp_lcd_panel_t *panel)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    if (jd9853->reset_gpio_num >= 0) {
        gpio_set_level(jd9853->reset_gpio_num, jd9853->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(jd9853->reset_gpio_num, !jd9853->reset_level);
        vTaskDelay(pdMS_TO_TICKS(120));
    } else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(jd9853->io, LCD_CMD_SWRESET, NULL, 0),
                            TAG, "send SWRESET failed");
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    return ESP_OK;
}

static esp_err_t panel_jd9853_init(esp_lcd_panel_t *panel)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    esp_lcd_panel_io_handle_t io = jd9853->io;

    // 上电复位后控制器处于睡眠模式，先退出睡眠；再下发由 panel_dev_config 推导的
    // MADCTL/COLMOD（初始化表中的 0x3A 会再次覆盖 COLMOD，值一致）
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0),
                        TAG, "send SLPOUT failed");
    vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t madctl = jd9853->madctl_val;
    uint8_t colmod = jd9853->colmod_val;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, &madctl, 1),
                        TAG, "send MADCTL failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, &colmod, 1),
                        TAG, "send COLMOD failed");

    const size_t cmds_num = sizeof(kInitCmds) / sizeof(kInitCmds[0]);
    for (size_t i = 0; i < cmds_num; i++) {
        const jd9853_init_cmd_t *cmd = &kInitCmds[i];
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, cmd->cmd, cmd->data, cmd->data_bytes),
                            TAG, "send command %02Xh failed", cmd->cmd);
        if (cmd->delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(cmd->delay_ms));
        }
    }
    ESP_LOGD(TAG, "send init commands success");
    return ESP_OK;
}

static esp_err_t panel_jd9853_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start,
                                          int x_end, int y_end, const void *color_data)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");

    // 可视区在 GRAM 中的偏移由此补偿（本模组 320-296=24 行）
    x_start += jd9853->x_gap;
    x_end += jd9853->x_gap;
    y_start += jd9853->y_gap;
    y_end += jd9853->y_gap;

    const uint8_t caset[] = {
        (uint8_t)(x_start >> 8), (uint8_t)(x_start & 0xFF),
        (uint8_t)((x_end - 1) >> 8), (uint8_t)((x_end - 1) & 0xFF),
    };
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(jd9853->io, LCD_CMD_CASET, caset, sizeof(caset)),
                        TAG, "send CASET failed");

    const uint8_t raset[] = {
        (uint8_t)(y_start >> 8), (uint8_t)(y_start & 0xFF),
        (uint8_t)((y_end - 1) >> 8), (uint8_t)((y_end - 1) & 0xFF),
    };
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(jd9853->io, LCD_CMD_RASET, raset, sizeof(raset)),
                        TAG, "send RASET failed");

    const size_t len = (size_t)(x_end - x_start) * (y_end - y_start) * jd9853->fb_bits_per_pixel / 8;
    return esp_lcd_panel_io_tx_color(jd9853->io, LCD_CMD_RAMWR, color_data, len);
}

static esp_err_t panel_jd9853_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    const int command = invert_color_data ? LCD_CMD_INVON : LCD_CMD_INVOFF;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(jd9853->io, command, NULL, 0),
                        TAG, "send invert color command failed");
    return ESP_OK;
}

static esp_err_t panel_jd9853_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    if (mirror_x) {
        jd9853->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        jd9853->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        jd9853->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        jd9853->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(jd9853->io, LCD_CMD_MADCTL,
                                                  &jd9853->madctl_val, 1),
                        TAG, "send MADCTL failed");
    return ESP_OK;
}

static esp_err_t panel_jd9853_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    if (swap_axes) {
        jd9853->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        jd9853->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(jd9853->io, LCD_CMD_MADCTL,
                                                  &jd9853->madctl_val, 1),
                        TAG, "send MADCTL failed");
    return ESP_OK;
}

static esp_err_t panel_jd9853_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    jd9853->x_gap = x_gap;
    jd9853->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_jd9853_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    const int command = on_off ? LCD_CMD_DISPON : LCD_CMD_DISPOFF;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(jd9853->io, command, NULL, 0),
                        TAG, "send display on/off command failed");
    return ESP_OK;
}

static esp_err_t panel_jd9853_disp_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    jd9853_panel_t *jd9853 = __containerof(panel, jd9853_panel_t, base);
    const int command = sleep ? LCD_CMD_SLPIN : LCD_CMD_SLPOUT;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(jd9853->io, command, NULL, 0),
                        TAG, "send sleep command failed");
    vTaskDelay(pdMS_TO_TICKS(120)); // 进出睡眠需等待振荡器稳定
    return ESP_OK;
}
