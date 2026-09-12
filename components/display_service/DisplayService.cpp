// DisplayService 实现：SPI 总线 + JD9853 面板 + LVGL(esp_lvgl_port) 显示 + 双屏 UI
//
// 面板硬件初始化沿用 esp_lcd（JD9853 驱动见同目录 esp_lcd_jd9853.c）；
// LVGL 任务、绘制缓冲、flush 与 DMA 完成同步全部由 esp_lvgl_port 官方移植层处理
// （内部注册 on_color_trans_done）。
//
// 双屏 UI：
//   - 状态页：中继 ID、Wi-Fi 在线/IP、MQTT 状态、已配对节点数、运行时长
//   - 节点列表页：每个节点一行（ID + 传感器类型 + 距上次数据时长）
// 第一版 UI 用英文 + ASCII（Montserrat 16/24 即可），后续若加中文再加中文字体。
#include "display_service/DisplayService.hpp"
#include "display_service/esp_lcd_jd9853.h"
#include "wifi_provider/WifiManager.hpp"
#include "ble_central/BleCentral.hpp"
#include "mqtt_reporter/MqttReporter.hpp"
#include "node_registry/NodeRegistry.hpp"
#include "app_config/AppConfig.hpp"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lvgl_port.h"
#include "esp_timer.h"
#include "app_config/AppConfig.hpp"
#include <cstdio>
#include <cstring>

namespace esp32hub {

static const char* TAG = "display";

// ---- 配色（0xRRGGBB）----
static constexpr uint32_t kColorBg       = 0x0B1E3A; // 屏幕底色：深海军蓝
static constexpr uint32_t kColorCardBg   = 0x102A4C; // 卡片底色
static constexpr uint32_t kColorCardBd   = 0x23527E; // 卡片/分隔线描边
static constexpr uint32_t kColorCyan     = 0x2FD4F5; // 强调青（图标/标题）
static constexpr uint32_t kColorMuted    = 0x8FB4D8; // 次要文字（蓝灰）
static constexpr uint32_t kColorWhite    = 0xFFFFFF;
static constexpr uint32_t kColorGreen    = 0x2FE06F;
static constexpr uint32_t kColorGreenBg  = 0x0C3A24;
static constexpr uint32_t kColorRed      = 0xFF5C5C;
static constexpr uint32_t kColorRedBg    = 0x3A1218;
static constexpr uint32_t kColorAmber    = 0xFFC83D;
static constexpr uint32_t kColorAmberBg  = 0x3A2E0C;

// ================ 屏幕硬件初始化 ================

esp_err_t DisplayService::Init(const Pins& pins)
{
    // 1. 初始化 SPI 总线（SPI2_HOST）。LCD 只写不读，MISO 必须显式设为 -1，
    //    零初始化会被当成 GPIO0 一并配置，引发总线异常。
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = pins.mosi;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = pins.sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = kWidth * 40 * sizeof(uint16_t); // LVGL 分区缓冲大小

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    // 2. 创建 SPI 面板 IO（CS/DC 引脚）
    //    lcd_cmd_bits/lcd_param_bits 必须为 8（JD9853 命令/参数均 8 位），
    //    漏设会导致命令相位宽度为 0，出现 "tx_buffer should be NULL... skip MOSI phase" 错误。
    //    on_color_trans_done 留空：esp_lvgl_port 内部会自行注册 flush 同步回调。
    //    pclk 40MHz：MOSI/SCLK 走 C3 的 FSPI IOMUX 脚，实测余量充足；
    //    若换长排线后出现花屏，降到 20MHz 即可。
    esp_lcd_panel_io_spi_config_t io_cfg = {};
    io_cfg.cs_gpio_num = pins.cs;
    io_cfg.dc_gpio_num = pins.dc;
    io_cfg.spi_mode = 0;
    io_cfg.pclk_hz = 40 * 1000 * 1000;
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;
    io_cfg.trans_queue_depth = 10;
    io_cfg.on_color_trans_done = nullptr;
    io_cfg.user_ctx = nullptr;

    esp_lcd_panel_io_handle_t io_handle = nullptr;
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_io_spi failed: %s", esp_err_to_name(err));
        return err;
    }

    // 3. 创建 JD9853 面板（自定义驱动，见 esp_lcd_jd9853.c）
    //    rgb_ele_order 必须为 RGB：LVGL 输出标准 RGB565；
    //    R/B 通道问题由下方 swap_bytes 解决（字节序修正后不能再置 BGR 位，
    //    否则 R/B 会被双重交换——实测深蓝背景会显示成亮青、卡片显示成品红）。
    esp_lcd_panel_dev_config_t panel_cfg = {};
    panel_cfg.reset_gpio_num = pins.rst;
    panel_cfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_cfg.bits_per_pixel = 16;
    panel_cfg.vendor_config = nullptr;

    err = esp_lcd_new_panel_jd9853(io_handle, &panel_cfg, &panel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_jd9853 failed: %s", esp_err_to_name(err));
        return err;
    }

    // 4. 复位 + 初始化 + 方向校正
    esp_lcd_panel_reset(panel_);
    esp_lcd_panel_init(panel_);
    // JD9853 的 GRAM 是 240x320，本模组可视区只有 240x296：
    // 沿 Y 轴偏移 24 行，让写入的 240x296 落在可视区内。
    // 若换模组后发现画面上下偏移 24 行，把下面的 mirror_y 改成 true 即可把偏移换到另一端。
    esp_lcd_panel_set_gap(panel_, 0, 24);
    // JD9853 IPS 模组需要反色。
    // 竖屏（240 宽 x 296 高）对应 swap_xy=false / mirror_x=false / mirror_y=false；
    // 若换屏后方向不对，再调整 mirror_x/mirror_y/swap_xy 即可。
    esp_lcd_panel_invert_color(panel_, true);
    esp_lcd_panel_swap_xy(panel_, false);
    esp_lcd_panel_mirror(panel_, false, false);
    esp_lcd_panel_disp_on_off(panel_, true);

    // 5. 背光（若接了 BLK 引脚）
    if (pins.bl >= 0) {
        gpio_config_t bl_conf = {};
        bl_conf.pin_bit_mask = 1ULL << pins.bl;
        bl_conf.mode = GPIO_MODE_OUTPUT;
        bl_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        bl_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        bl_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&bl_conf);
        gpio_set_level(static_cast<gpio_num_t>(pins.bl), 1);
    }

    // 6. LVGL 移植层初始化（内部创建 lvgl 任务 + tick 定时器）
    lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    err = lvgl_port_init(&lvgl_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lvgl_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // 7. 向 LVGL 注册显示设备：缓冲按 40 行分区（buffer_size 单位为像素），
    //    移植层负责 flush 与 DMA 完成同步。
    //    color_format=RGB565 与面板的 16bpp 一致。
    //    swap_bytes 必须开启：LVGL 的 RGB565 按小端存放，JD9853 要求高字节先传，
    //    不交换时深蓝背景会显示成黄绿、文字抗锯齿像素错位成彩色花边（发虚模糊）。
    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = io_handle;
    disp_cfg.panel_handle = panel_;
    disp_cfg.buffer_size = kWidth * 40; // 像素数（非字节）
    disp_cfg.double_buffer = true;
    disp_cfg.hres = kWidth;
    disp_cfg.vres = kHeight;
    disp_cfg.color_format = LV_COLOR_FORMAT_RGB565;
    disp_cfg.flags.buff_dma = true;     // 缓冲从 DMA 可访问内存分配
    disp_cfg.flags.swap_bytes = true;   // RGB565 高低字节交换（JD9853 大端接收）
    // rotation 需与上方 esp_lcd_panel_swap_xy/mirror 的硬件状态保持一致
    disp_cfg.rotation.swap_xy = false;
    disp_cfg.rotation.mirror_x = false;
    disp_cfg.rotation.mirror_y = false;

    lvgl_disp_ = lvgl_port_add_disp(&disp_cfg);
    if (lvgl_disp_ == nullptr) {
        ESP_LOGE(TAG, "lvgl_port_add_disp failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "JD9853 + LVGL init ok (%dx%d)", kWidth, kHeight);
    return ESP_OK;
}

// ================ UI 绑定 ================

void DisplayService::Bind(WifiManager* wifi, BleCentral* ble,
                          MqttReporter* reporter, NodeRegistry* registry)
{
    wifi_ = wifi;
    ble_ = ble;
    reporter_ = reporter;
    registry_ = registry;
}

void DisplayService::Start()
{
    // LVGL 锁内构建两块屏幕，避免渲染任务启动后并发竞争
    lvgl_port_lock(0);
    BuildScreens();
    lvgl_port_unlock();

    xTaskCreate(&DisplayService::TaskMain, "display", 6144, this, 5, &task_);
}

void DisplayService::TaskMain(void* arg)
{
    DisplayService* self = static_cast<DisplayService*>(arg);
    self->Run();
    vTaskDelete(nullptr);
}

void DisplayService::Run()
{
    ESP_LOGI(TAG, "display render task started");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        lvgl_port_lock(0);
        Render();
        lvgl_port_unlock();
    }
}

// ================ LVGL UI 构建 ================

namespace {

// 带圆角/描边的卡片底板
lv_obj_t* MakeCard(lv_obj_t* parent, int x, int y, int w, int h)
{
    lv_obj_t* card = lv_obj_create(parent);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_size(card, w, h);
    lv_obj_set_style_radius(card, 10, 0);
    lv_obj_set_style_bg_color(card, lv_color_hex(kColorCardBg), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(kColorCardBd), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    return card;
}

// 普通文本标签
lv_obj_t* MakeLabel(lv_obj_t* parent, int x, int y, const char* text,
                    uint32_t color, const lv_font_t* font)
{
    lv_obj_t* lbl = lv_label_create(parent);
    lv_obj_set_pos(lbl, x, y);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, lv_color_hex(color), 0);
    lv_obj_set_style_text_font(lbl, font, 0);
    return lbl;
}

// 屏幕底色与公共属性
void SetupScreenBg(lv_obj_t* scr)
{
    lv_obj_set_style_bg_color(scr, lv_color_hex(kColorBg), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
}

} // namespace

void DisplayService::BuildScreens()
{
    scr_status_ = lv_obj_create(nullptr);
    SetupScreenBg(scr_status_);
    BuildStatusScreen(scr_status_);

    scr_nodes_ = lv_obj_create(nullptr);
    SetupScreenBg(scr_nodes_);
    BuildNodesScreen(scr_nodes_);

    lv_screen_load(scr_status_);
    showing_nodes_ = false;
}

void DisplayService::BuildStatusScreen(lv_obj_t* scr)
{
    // 标题栏
    MakeLabel(scr, 8, 6, "esp32-hub", kColorCyan, &lv_font_montserrat_24);

    // 在线徽章
    badge_ = lv_obj_create(scr);
    lv_obj_set_pos(badge_, 168, 10);
    lv_obj_set_size(badge_, 64, 22);
    lv_obj_set_style_radius(badge_, 11, 0);
    lv_obj_set_style_bg_color(badge_, lv_color_hex(kColorAmberBg), 0);
    lv_obj_set_style_bg_opa(badge_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(badge_, 0, 0);
    lv_obj_set_style_pad_all(badge_, 0, 0);
    lv_obj_remove_flag(badge_, LV_OBJ_FLAG_SCROLLABLE);
    badge_label_ = lv_label_create(badge_);
    lv_obj_set_style_text_font(badge_label_, &lv_font_montserrat_16, 0);
    lv_obj_center(badge_label_);
    lv_label_set_text(badge_label_, "BOOT");

    // 分隔线
    lv_obj_t* line = lv_obj_create(scr);
    lv_obj_set_pos(line, 0, 40);
    lv_obj_set_size(line, kWidth, 2);
    lv_obj_set_style_bg_color(line, lv_color_hex(kColorCardBd), 0);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_set_style_pad_all(line, 0, 0);
    lv_obj_remove_flag(line, LV_OBJ_FLAG_SCROLLABLE);

    // 中继 ID 卡片
    MakeLabel(scr, 12, 56, "RELAY ID", kColorMuted, &lv_font_montserrat_16);
    relay_id_lbl_ = MakeLabel(scr, 12, 78, "—", kColorWhite, &lv_font_montserrat_24);

    // Wi-Fi IP 卡片
    MakeLabel(scr, 130, 56, "WIFI", kColorMuted, &lv_font_montserrat_16);
    ip_value_ = MakeLabel(scr, 130, 78, "—", kColorWhite, &lv_font_montserrat_24);

    // MQTT 状态卡片
    MakeLabel(scr, 12, 132, "MQTT", kColorMuted, &lv_font_montserrat_16);
    mqtt_state_ = MakeLabel(scr, 12, 154, "—", kColorWhite, &lv_font_montserrat_24);

    // 节点数卡片
    MakeLabel(scr, 130, 132, "NODES", kColorMuted, &lv_font_montserrat_16);
    node_count_ = MakeLabel(scr, 130, 154, "—", kColorWhite, &lv_font_montserrat_24);

    // 运行时长（页脚，296 高屏底部预留 16px 内边距）
    uptime_label_ = MakeLabel(scr, 8, 266, "uptime 00:00:00", kColorMuted, &lv_font_montserrat_16);
}

void DisplayService::BuildNodesScreen(lv_obj_t* scr)
{
    MakeLabel(scr, 8, 6, "Paired Nodes", kColorCyan, &lv_font_montserrat_24);

    // 4 个节点行：每行 56 像素高、间隔 62（296 高屏放得下 4 行）
    for (int i = 0; i < kMaxNodesShown; ++i) {
        int y = 42 + i * 62;
        lv_obj_t* row = MakeCard(scr, 8, y, kWidth - 16, 56);
        node_rows_[i] = row;
        node_ids_[i]   = MakeLabel(row, 10, 6,  "—", kColorWhite, &lv_font_montserrat_16);
        node_types_[i] = MakeLabel(row, 10, 26, "—", kColorMuted,  &lv_font_montserrat_16);
        node_ages_[i]  = MakeLabel(row, 10, 42, "—", kColorMuted,  &lv_font_montserrat_16);
    }
}

// ================ 渲染：切屏 + 更新动态数据 ================

void DisplayService::Render()
{
    // 切屏：状态页 ↔ 节点列表页（每秒切换，便于观察两侧状态）
    // 没有节点时只显示状态页，避免空白节点页无意义切换
    bool has_nodes = registry_ && registry_->Count() > 0;
    if (has_nodes) {
        if (showing_nodes_) {
            lv_screen_load(scr_status_);
            showing_nodes_ = false;
        } else {
            lv_screen_load(scr_nodes_);
            showing_nodes_ = true;
        }
    } else if (showing_nodes_) {
        lv_screen_load(scr_status_);
        showing_nodes_ = false;
    }

    // ---- 状态页：填充数据 ----
    bool wifi_online  = wifi_ && wifi_->IsOnline();
    bool provision    = wifi_ && wifi_->IsProvisioning();
    bool mqtt_online  = reporter_ && reporter_->IsConnected();

    const char* badge_text = "BOOT";
    uint32_t badge_bg = kColorAmberBg;
    uint32_t badge_fg = kColorAmber;
    if (wifi_online) {
        badge_text = "ONLINE";
        badge_bg = kColorGreenBg;
        badge_fg = kColorGreen;
    } else if (provision) {
        badge_text = "PROV";
        badge_bg = kColorAmberBg;
        badge_fg = kColorAmber;
    } else {
        badge_text = "DOWN";
        badge_bg = kColorRedBg;
        badge_fg = kColorRed;
    }
    lv_obj_set_style_bg_color(badge_, lv_color_hex(badge_bg), 0);
    lv_label_set_text(badge_label_, badge_text);
    lv_obj_set_style_text_color(badge_label_, lv_color_hex(badge_fg), 0);

    // 中继 ID（首次拿到 MAC 后由 app_config 生成并保存，这里只读取）
    if (relay_id_lbl_ && registry_) {
        const char* rid = registry_->RelayId();
        if (rid && rid[0]) {
            lv_label_set_text(relay_id_lbl_, rid);
        }
    }

    // Wi-Fi IP
    std::string ip = wifi_ ? wifi_->CurrentIp() : std::string();
    if (ip.empty()) ip = wifi_ ? wifi_->ApSsid() : std::string("—");
    if (ip_value_) lv_label_set_text(ip_value_, ip.c_str());

    // MQTT 状态
    if (mqtt_state_) {
        lv_label_set_text(mqtt_state_, mqtt_online ? "ONLINE" : (wifi_online ? "OFF" : "—"));
        lv_obj_set_style_text_color(mqtt_state_,
                                    lv_color_hex(mqtt_online ? kColorGreen : kColorRed), 0);
    }

    // 节点数
    if (node_count_ && registry_) {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%d", registry_->Count());
        lv_label_set_text(node_count_, buf);
    }

    // uptime（秒）
    if (uptime_label_) {
        static int64_t boot_us = 0;
        if (boot_us == 0) boot_us = esp_timer_get_time();
        int64_t uptime_s = (esp_timer_get_time() - boot_us) / 1000000;
        int h = static_cast<int>(uptime_s / 3600);
        int m = static_cast<int>((uptime_s % 3600) / 60);
        int s = static_cast<int>(uptime_s % 60);
        char buf[24];
        std::snprintf(buf, sizeof(buf), "uptime %02d:%02d:%02d", h, m, s);
        lv_label_set_text(uptime_label_, buf);
    }

    // ---- 节点列表页：填充数据 ----
    if (has_nodes) {
        for (int i = 0; i < kMaxNodesShown; ++i) {
            if (i < registry_->Count()) {
                auto info = registry_->GetByIndex(i);
                if (node_ids_[i])   lv_label_set_text(node_ids_[i],   info.node_id.c_str());
                if (node_types_[i]) lv_label_set_text(node_types_[i], info.sensor_summary.c_str());
                if (node_ages_[i]) {
                    int64_t age_s = (esp_timer_get_time() - info.last_seen_us) / 1000000;
                    char buf[24];
                    std::snprintf(buf, sizeof(buf), "%llds ago",
                                  static_cast<long long>(age_s));
                    lv_label_set_text(node_ages_[i], buf);
                }
            } else {
                if (node_ids_[i])   lv_label_set_text(node_ids_[i],   "—");
                if (node_types_[i]) lv_label_set_text(node_types_[i], "");
                if (node_ages_[i])  lv_label_set_text(node_ages_[i],  "");
            }
        }
    }
}

} // namespace esp32hub
