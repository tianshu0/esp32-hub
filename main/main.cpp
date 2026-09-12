// esp32-hub 装配入口：只做系统初始化 + 组件组合，业务状态机在各组件 Init 内部
//
// 职责链：
//   wifi_provider  -> 拿到 IP 后触发事件
//     mqtt_reporter 订阅 IP 事件，连 broker
//     ble_central   订阅 IP 事件，开始扫描
//   ble_central    -> GATT 连接 + 握手 -> 写 node_registry
//   ble_central    -> GATT Notify 数据 -> sensor_pipeline 解析
//   sensor_pipeline-> 投递传感器数据事件
//   mqtt_reporter  -> 订阅数据事件，按主题上报
//   display_service-> 查询 wifi/ble/mqtt/registry 状态，刷新屏幕
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"

#include "app_config/AppConfig.hpp"
#include "wifi_provider/WifiManager.hpp"
#include "display_service/DisplayService.hpp"
#include "ble_central/BleCentral.hpp"
#include "node_registry/NodeRegistry.hpp"
#include "sensor_pipeline/SensorPipeline.hpp"
#include "mqtt_reporter/MqttReporter.hpp"

// 各组件类均定义于 esp32hub 命名空间，装配入口统一引入
using namespace esp32hub;

static const char* TAG = "esp32-hub";

extern "C" void app_main(void)
{
    // 注意：NVS 初始化（含损坏擦除自愈）由 AppConfig::Init() 统一完成，
    // 这里不再裸调 nvs_flash_init()，否则分区表变动后旧数据会直接 abort。
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // NimBLE 协议栈初始化（host 必须先于 ble_central 启动）
    ESP_ERROR_CHECK(nimble_port_init());

    // 以下组件均以 static 存在：内部创建了任务/注册了事件回调，
    // 若作为栈对象在 app_main 返回时析构会造成悬挂指针，故常驻到系统重启。
    static AppConfig config;        // 配置存储（NVS 读写，内部初始化 NVS）
    ESP_ERROR_CHECK(config.Init());

    static NodeRegistry registry;    // 已配对节点表（持久化 + RelayId 生成）
    ESP_ERROR_CHECK(registry.Init(&config));

    static WifiManager wifi;        // Wi-Fi 连接管理 + 配网 Portal（自动回退 AP 模式）
    wifi.Init(&config);

    static DisplayService display;  // JD9853 SPI 屏(2.01" 240x296) + LVGL UI（状态页/节点列表页）
    display.Init(DisplayService::Pins{});

    // HTTP 常驻：配网态服务配置页
    ESP_ERROR_CHECK(wifi.StartPortal());

    // WiFi 任务必须在 BLE/MQTT 订阅事件前启动，
    // 否则它们读取到的 wifi 状态仍是默认 kIdle，首屏渲染错位。
    wifi.Start();

    // sensor_pipeline 先于 mqtt_reporter / ble_central 初始化：
    // 前者用于创建事件基，后者要向该事件基 post/订阅事件。
    static SensorPipeline pipeline; // 蓝牙数据解析：GATT 数据 -> 解析 -> 投递传感器数据事件
    ESP_ERROR_CHECK(pipeline.Init());

    static MqttReporter reporter;   // MQTT 客户端：连 esp32-broker，按主题上报中继/传感器数据
    ESP_ERROR_CHECK(reporter.Init(&config, &registry));

    static BleCentral ble;           // BLE Central：扫描/连接/握手/Notify 接收
    ESP_ERROR_CHECK(ble.Init(&config, &registry, &pipeline));

    // DisplayService 读取 wifi/ble/mqtt/registry 状态刷新屏幕
    display.Bind(&wifi, &ble, &reporter, &registry);
    display.Start();

    ESP_LOGI(TAG, "esp32-hub started");
}
