// mqtt_reporter 组件：MQTT 客户端（连 esp32-broker，上报中继状态与传感器数据）
//
// 职责：
//   - 订阅 Wi-Fi 事件：拿到 IP 才启动 MQTT，断开立即停
//   - 订阅 SensorPipeline 的 hub_sensor_data 事件：
//       kEventSensorData -> 上报 hub/{relay_id}/node/{node_id}/data
//       kEventNodeOnline -> 上报 hub/{relay_id}/node/{node_id}/register
//   - 上报中继自身状态：hub/{relay_id}/status（含 LWT 离线遗嘱）、hub/{relay_id}/info
//
// 主题设计（与 pc-monitor-dashboard 订阅对应）：
//   hub/{relay_id}/status                  在线状态（online/offline，retained + LWT）
//   hub/{relay_id}/info                    中继信息（版本/节点数）
//   hub/{relay_id}/node/{node_id}/register 节点注册（能力清单）
//   hub/{relay_id}/node/{node_id}/data     节点传感器数据
//
// 设计要点：
//   - 与 esp32-broker 一致：Init 内 xTaskCreate，static 生命周期
//   - 组件自包含：不暴露事件给外部，display_service 通过 IsConnected() 查询状态
//   - MQTT 客户端使用 ESP-IDF 官方 esp-mqtt（自动重连）
#pragma once

#include <cstdint>
#include <string>
#include "esp_err.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sensor_pipeline/SensorPipeline.hpp"

namespace esp32hub {

class AppConfig;
class NodeRegistry;

class MqttReporter {
public:
    MqttReporter() = default;
    ~MqttReporter();

    // 初始化：订阅 Wi-Fi 与 sensor_pipeline 事件；不立即连 broker（等 Wi-Fi 在线）
    esp_err_t Init(AppConfig* config, NodeRegistry* registry);

    // MQTT 是否已连接（供 display_service 查询）
    bool IsConnected() const { return connected_; }

private:
    static void TaskMain(void* arg);
    static void WifiEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data);
    static void SensorEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data);
    static void MqttEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data);

    void Run();
    void StartClient();
    void StopClient();
    // 启动前的域名解析预检（mDNS 的 .local 在刚关联 AP 时常常首轮无应答，
    // 直接让 esp-mqtt 去连会刷一条 7 秒超时报错；这里先带重试解析到 IP 再启动）
    bool WaitForBrokerResolved(const std::string& host);
    void OnMqttConnected();
    void PublishStatus(const char* state);
    void PublishInfo();
    void PublishSensorData(const SensorPipeline::SensorData& data);
    void PublishNodeRegister(const SensorPipeline::NodeRegistered& reg);
    // 按原始 node_id + 能力清单发布 register（MQTT 重连后补发持久化节点时使用）
    void PublishNodeRegisterRaw(const char* node_id, const char* capability_json);

    AppConfig* config_ = nullptr;
    NodeRegistry* registry_ = nullptr;

    TaskHandle_t task_ = nullptr;
    EventGroupHandle_t events_ = nullptr;
    esp_mqtt_client_handle_t client_ = nullptr;

    static constexpr EventBits_t kWifiOnline  = 1 << 0;
    static constexpr EventBits_t kWifiOffline = 1 << 1;
    static constexpr EventBits_t kMqttStarted = 1 << 2;

    bool connected_ = false;
    bool registered_handlers_ = false;
};

} // namespace esp32hub
