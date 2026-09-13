// DisplayService 实现：SPI 总线 + JD9853 面板 + LVGL(esp_lvgl_port) 显示 + 单屏 UI
//
// 面板硬件初始化沿用 esp_lcd（JD9853 驱动见同目录 esp_lcd_jd9853.c）；
// LVGL 任务、绘制缓冲、flush 与 DMA 完成同步全部由 esp_lvgl_port 官方移植层处理
// （内部注册 on_color_trans_done）。
//
// 当前只有一个状态页：中继 ID、Wi-Fi 在线/IP、MQTT 状态、已配对节点数、运行时长。
// 节点详情/选择页暂不实现，后续接入物理按钮后由按钮事件手动切换。
// UI 用英文 + ASCII（Montserrat 16/24 即可），后续若加中文再加中文字体。
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
#include "esp_system.h"
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
    buscfg.max_transfer_sz = kWidth * 20 * sizeof(uint16_t); // 横屏一次刷 20 行：296*20*2=11840 B

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
    // JD9853 的 GRAM 是 240x320，本模组可视区 240x296：
    // 实测可视区从 GRAM 第 0 行开始（320-296=24 行死区在底部），gap 必须为 (0,0)。
    // 若误设 y_gap=24，画面会整体下移 24 行：顶部可视区读到从未写入的 GRAM，
    // 表现为屏幕顶部一条约 24 行高、随圆角弯曲的雪花噪点带，底部 uptime 被裁掉。
    // 横屏（MV=1）后逻辑 X 轴映射到 GRAM 行：逻辑 x 取 0~295 正好落在可见行 0~295，
    // 24 行死区仍在末端、不写入，所以 gap 维持 (0,0) 即可。
    esp_lcd_panel_set_gap(panel_, 0, 0);
    // JD9853 IPS 模组需要反色。
    // 横屏 296x240 = MADCTL 置 MV(行列交换) + MX(水平镜像)，即顺时针 90°。
    // 这是唯一的旋转手段：esp_lvgl_port 保持 ROTATION_0，LVGL 逻辑分辨率直接给
    // 296x240，flush 坐标不做任何二次变换（与官方 esp_lcd st7789 驱动的用法一致：
    // MV=1 后 CASET 寻址长轴 0~319、RASET 寻址短轴 0~239，驱动里无需交换窗口）。
    // 若实物装壳方向相反、画面上下颠倒：把下面改成 swap_xy(true) + mirror(false,true)。
    esp_lcd_panel_invert_color(panel_, true);
    esp_lcd_panel_swap_xy(panel_, true);
    esp_lcd_panel_mirror(panel_, true, false);
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

    // 7. 向 LVGL 注册显示设备：缓冲按 20 行分区（buffer_size 单位为像素），
    //    移植层负责 flush 与 DMA 完成同步。
    //    横屏双缓冲 2 × 296×20×2 = 23680 B；C3 无 PSRAM，堆紧张，
    //    不能再用整屏缓冲，20 行分区实测流畅度足够。
    //    color_format=RGB565 与面板的 16bpp 一致。
    //    swap_bytes 必须开启：LVGL 的 RGB565 按小端存放，JD9853 要求高字节先传，
    //    不交换时深蓝背景会显示成黄绿、文字抗锯齿像素错位成彩色花边（发虚模糊）。
    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = io_handle;
    disp_cfg.panel_handle = panel_;
    disp_cfg.buffer_size = kWidth * 20; // 像素数（非字节）
    disp_cfg.double_buffer = true;
    disp_cfg.hres = kWidth;             // 296（横屏逻辑宽）
    disp_cfg.vres = kHeight;            // 240（横屏逻辑高）
    disp_cfg.color_format = LV_COLOR_FORMAT_RGB565;
    disp_cfg.flags.buff_dma = true;     // 缓冲从 DMA 可访问内存分配
    disp_cfg.flags.swap_bytes = true;   // RGB565 高低字节交换（JD9853 大端接收）
    // 硬件旋转 90°（CW）：必须与上方 esp_lcd_panel_swap_xy/mirror 的设置一致；
    // esp_lvgl_port 在 ROTATION_0 下只会把这两个值原样下发给面板驱动。
    disp_cfg.rotation.swap_xy = true;
    disp_cfg.rotation.mirror_x = true;
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
    // LVGL 锁内构建状态页，避免渲染任务启动后并发竞争
    lvgl_port_lock(0);
    BuildScreens();
    lvgl_port_unlock();

    // 任务栈不足时 xTaskCreate 只返回错误、不打印日志，必须显式检查
    if (xTaskCreate(&DisplayService::TaskMain, "display", 6144, this, 5, &task_) != pdPASS) {
        ESP_LOGE(TAG, "create display task failed, free heap %u B",
                 (unsigned)esp_get_free_heap_size());
    }
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

    lv_screen_load(scr_status_);
}

void DisplayService::BuildStatusScreen(lv_obj_t* scr)
{
    // 圆角安全区：四角圆角半径约 26px，贴边元素统一内缩 30px 起步；
    // y 方向内容只使用 40~214 这段（240 高），页脚水平居中落在底边平直段内。

    // 标题栏（内缩 30px 避开左上圆角弧）
    MakeLabel(scr, 30, 8, "esp32-hub", kColorCyan, &lv_font_montserrat_24);

    // 在线徽章（右上角同样内缩；66 宽可容下 "ONLINE"）
    badge_ = lv_obj_create(scr);
    lv_obj_set_pos(badge_, 200, 10);
    lv_obj_set_size(badge_, 66, 22);
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

    // 分隔线（y=40 已在圆角弧之外，可整宽绘制）
    lv_obj_t* line = lv_obj_create(scr);
    lv_obj_set_pos(line, 0, 40);
    lv_obj_set_size(line, kWidth, 2);
    lv_obj_set_style_bg_color(line, lv_color_hex(kColorCardBd), 0);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_set_style_pad_all(line, 0, 0);
    lv_obj_remove_flag(line, LV_OBJ_FLAG_SCROLLABLE);

    // 左卡片：RELAY ID + WIFI + MQTT（x=8 起，宽 140；y=48 高 158，止于 206）
    lv_obj_t* left = MakeCard(scr, 8, 48, 140, 158);
    MakeLabel(left, 12, 6,   "RELAY ID", kColorMuted, &lv_font_montserrat_16);
    relay_id_lbl_ = MakeLabel(left, 12, 24, "-", kColorWhite, &lv_font_montserrat_24);

    MakeLabel(left, 12, 62, "WIFI", kColorMuted, &lv_font_montserrat_16);
    ip_value_ = MakeLabel(left, 12, 82, "-", kColorWhite, &lv_font_montserrat_16);
    lv_obj_set_width(ip_value_, 116);
    lv_label_set_long_mode(ip_value_, LV_LABEL_LONG_CLIP);

    MakeLabel(left, 12, 108, "MQTT", kColorMuted, &lv_font_montserrat_16);
    mqtt_state_ = MakeLabel(left, 12, 126, "-", kColorWhite, &lv_font_montserrat_24);

    // 右卡片：NODES（x=156 起，宽 132）
    lv_obj_t* right = MakeCard(scr, 156, 48, 132, 158);
    MakeLabel(right, 12, 40, "NODES", kColorMuted, &lv_font_montserrat_16);
    node_count_ = MakeLabel(right, 12, 60, "-", kColorWhite, &lv_font_montserrat_24);
    MakeLabel(right, 12, 100, "paired", kColorMuted, &lv_font_montserrat_16);

    // 运行时长：相对屏幕底边水平居中（文字只占底部平直段，不进左右圆角弧）
    uptime_label_ = MakeLabel(scr, 0, 0, "uptime 00:00:00", kColorMuted, &lv_font_montserrat_16);
    lv_obj_align(uptime_label_, LV_ALIGN_BOTTOM_MID, 0, -6);
}

// ================ 渲染：更新动态数据 ================

void DisplayService::Render()
{
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
    if (ip.empty()) ip = wifi_ ? wifi_->ApSsid() : std::string("-");
    if (ip_value_) lv_label_set_text(ip_value_, ip.c_str());

    // MQTT 状态
    if (mqtt_state_) {
        lv_label_set_text(mqtt_state_, mqtt_online ? "ONLINE" : (wifi_online ? "OFF" : "-"));
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
}

} // namespace esp32hub
