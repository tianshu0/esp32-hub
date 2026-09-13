// display_service 组件：JD9853 SPI 屏驱动（2.01" 240x296）+ LVGL UI（状态页）
//
// 与 esp32-broker 的差异：
//   - 把 Display（硬件）+ StatusUi（UI）合并为单一组件，避免文件爆炸
//   - 不再注入 Broker 指针，改为注入 MqttReporter（hub 是 MQTT 客户端）
//
// 数据流：
//   DisplayService 通过 getter 读取 WifiManager/BleCentral/MqttReporter/NodeRegistry 状态，
//   每 1 秒刷新一次文本/颜色。BLE/MQTT 数据变化由各自组件直接写 registry，这里只读取。
// 节点详情/选择页暂不实现：后续加物理按钮后，由按钮事件手动切换页面再补 BuildNodesScreen。
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esp32hub {

class WifiManager;
class BleCentral;
class MqttReporter;
class NodeRegistry;

class DisplayService {
public:
    // 2.01" TFT（JD9853）SPI 接线：VCC GND SCL SDA RES DC CS BLK
    //
    // 引脚按 ESP32-C3 模组可用 GPIO 分配，ESP32-C3 Super Mini 等板子可直接照此接线：
    //   - 6/7/10 是 C3 的 FSPI IOMUX 默认脚（CLK/MOSI/CS0），走硬件映射才能跑高时钟
    //   - 避开 strapping 脚(2/8/9)、USB Serial/JTAG(18/19)、UART0 控制台(20/21)、
    //     片内 Flash(11~17)
    struct Pins {
        int mosi = 7;  // SDA
        int sclk = 6;  // SCL
        int cs   = 10;
        int dc   = 4;
        int rst  = 5;
        // BLK 背光：高电平点亮。若硬件直接把 BLK 接 3.3V 常亮，可设为 -1 不软件控制。
        int bl   = 3;
    };

    // 初始化 SPI 总线 + JD9853 面板 + LVGL（esp_lvgl_port）
    esp_err_t Init(const Pins& pins);

    // 绑定状态来源（必须在 Start 前调用，使 Render 能读到实时状态）
    void Bind(WifiManager* wifi, BleCentral* ble, MqttReporter* reporter, NodeRegistry* registry);

    // 构建状态页 LVGL 屏幕并启动渲染任务（内部 xTaskCreate）
    void Start();

    // 逻辑分辨率（横屏）：面板原生 GRAM 为 240x320、可视区 240x296，
    // 通过 JD9853 的 MADCTL MV 位做 90° 硬件旋转，LVGL 侧直接按 296x240 布局。
    // 横屏理由：模组四角为大圆角（半径约 26px），竖屏时四角元素（标题/徽章/页脚）
    // 会被圆角遮罩裁掉；横屏后宽 296、高仅 240，内容可沿长边内缩避开圆角。
    static constexpr int kWidth  = 296;
    static constexpr int kHeight = 240;

private:
    static void TaskMain(void* arg);
    void Run();
    void BuildScreens();
    void BuildStatusScreen(lv_obj_t* scr);    // 中继状态：Wi-Fi/MQTT/节点数
    void Render();                            // LVGL 锁内：更新状态页动态数据

    esp_lcd_panel_handle_t panel_ = nullptr;
    lv_display_t* lvgl_disp_      = nullptr;

    WifiManager* wifi_     = nullptr;
    BleCentral*  ble_      = nullptr;
    MqttReporter* reporter_ = nullptr;
    NodeRegistry* registry_ = nullptr;
    TaskHandle_t task_     = nullptr;

    lv_obj_t* scr_status_ = nullptr;

    // 状态页动态控件
    lv_obj_t* badge_         = nullptr;
    lv_obj_t* badge_label_   = nullptr;
    lv_obj_t* relay_id_lbl_  = nullptr;
    lv_obj_t* ip_value_      = nullptr;
    lv_obj_t* mqtt_state_    = nullptr;
    lv_obj_t* node_count_    = nullptr;
    lv_obj_t* uptime_label_  = nullptr;
};

} // namespace esp32hub
