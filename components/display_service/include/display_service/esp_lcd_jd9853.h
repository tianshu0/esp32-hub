// JD9853 SPI 面板驱动（esp_lcd 自定义 panel）
//
// 目标模组：2.01" TFT / SPI / 240x296 / driver IC: JD9853（Jadard）
//   - 控制器 GRAM 原生 240 列 x 320 行，本模组只用到其中 296 行，
//     剩下的 24 行由 esp_lcd_panel_set_gap() 偏移补偿（见 DisplayService::Init）。
//
// 实现 esp_lcd_panel_t 接口，用法与 ESP-IDF 内置的 ST7789/SSD1306 驱动完全一致：
//   esp_lcd_new_panel_io_spi() -> esp_lcd_new_panel_jd9853() -> esp_lcd_panel_ops 系列 API
#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建 JD9853 面板实例
 *
 * @param[in]  io                 SPI 面板 IO 句柄（esp_lcd_new_panel_io_spi 创建）
 * @param[in]  panel_dev_config   面板通用配置（复位脚 / RGB 顺序 / 位深）
 * @param[out] ret_panel          返回的面板句柄，交给 esp_lcd_panel_ops 使用
 * @return
 *      - ESP_OK               成功
 *      - ESP_ERR_INVALID_ARG  入参为空
 *      - ESP_ERR_NO_MEM       内存不足
 */
esp_err_t esp_lcd_new_panel_jd9853(const esp_lcd_panel_io_handle_t io,
                                   const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel);

#ifdef __cplusplus
}
#endif
