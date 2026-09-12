// ble_central 组件：BLE Central 角色（扫描/连接/服务发现/握手/Notify 接收）
//
// 协议设计（自定义 GATT 服务）：
//   - 服务 UUID：        0000ff00-0000-1000-8000-00805f9b34fb  (esp32-node 专属服务)
//   - 特征值 WRITE：      0000ff01-0000-1000-8000-00805f9b34fb  hub→node 握手请求
//   - 特征值 NOTIFY/READ：0000ff02-0000-1000-8000-00805f9b34fb  node→hub 响应/数据
//   - CCC 描述符：        00002902-0000-1000-8000-00805f9b34fb  订阅 NOTIFY
//
// 握手协议（4 步，JSON over GATT）：
//   1. hub → node (WRITE ff01)：{"act":"hello","relay_id":"hub-1A2B","ver":1}
//   2. node → hub (NOTIFY ff02)：{"act":"hello_ack","node_id":"node-xxx","types":["temp_hum"]}
//   3. hub → node (WRITE ff01)：{"act":"accept","interval_ms":5000}
//   4. node → hub (NOTIFY ff02)：{"act":"ack_done"}
//
// 数据上报（握手完成后，node 周期性 NOTIFY）：
//   {"type":"temp_hum","ts":1234567890,"values":{"temp":23.5,"humidity":65.2}}
//
// 设计要点：
//   - 单任务运行扫描/连接状态机（首版不做多连接并发，简化状态管理）
//   - 与 esp32-broker 一致：Init 内 xTaskCreate，static 生命周期
//   - 不依赖 mqtt_reporter：收到数据交给 SensorPipeline.Submit，
//     pipeline 内部 esp_event_post 给订阅者（含 mqtt_reporter）
//   - 不依赖 display_service：display_service 主动查询 NodeRegistry
#pragma once

#include <cstdint>
#include <string>
#include "esp_err.h"
#include "esp_event.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

namespace esp32hub {

class AppConfig;
class NodeRegistry;
class SensorPipeline;

class BleCentral {
public:
    BleCentral() = default;
    ~BleCentral();

    // 初始化：注册 Wi-Fi 事件 + 启动 NimBLE host 与状态机任务（均在本函数内 xTaskCreate）
    esp_err_t Init(AppConfig* config, NodeRegistry* registry, SensorPipeline* pipeline);

    // 是否正在扫描 / 是否已连接（供 display_service 查询）
    bool IsScanning() const { return scanning_; }
    bool IsConnected() const { return conn_handle_ != BLE_HS_CONN_HANDLE_NONE; }

private:
    static void TaskMain(void* arg);
    static void HostTask(void* arg);   // NimBLE host 事件循环任务入口
    static void WifiEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data);

    // GAP 事件
    static int GapEventCb(ble_gap_event* event, void* arg);
    // GATT 服务/特征值发现回调
    static int SvcDiscCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                         const struct ble_gatt_svc* service, void* arg);
    static int ChrDiscCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                         const struct ble_gatt_chr* chr, void* arg);

    void Run();
    void StartScan();
    void StopScan();
    void AttemptConnect(const ble_addr_t& addr);
    void DiscoverService(uint16_t conn_handle);
    void SubscribeAndHandshake(uint16_t conn_handle);
    void HandleNotify(const std::string& json);

    AppConfig* config_ = nullptr;
    NodeRegistry* registry_ = nullptr;
    SensorPipeline* pipeline_ = nullptr;

    TaskHandle_t task_ = nullptr;
    EventGroupHandle_t events_ = nullptr;
    static constexpr EventBits_t kWifiOnline  = 1 << 0;
    static constexpr EventBits_t kWifiOffline = 1 << 1;
    static constexpr EventBits_t kNodeLost    = 1 << 2;

    bool scanning_ = false;
    uint16_t conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
    bool paired_ = false;
    std::string node_id_;             // 当前连接节点的 ID（握手时从 hello_ack 提取）

    // 服务发现中间状态
    uint16_t svc_start_ = 0;
    uint16_t svc_end_   = 0;
    uint16_t h_write_   = 0;   // ff01：hub→node 握手请求
    uint16_t h_notify_  = 0;   // ff02：node→hub 响应/数据
};

} // namespace esp32hub
