// MqttReporter 实现：esp-mqtt 客户端 + 事件驱动
//
// 状态机：
//   WiFi 在线 -> 启动 mqtt 客户端（自动重连）-> 连接成功 -> 发布 status/info
//              -> 传感器数据/注册事件到达 -> 按主题发布
//   WiFi 断开 -> 停止 mqtt 客户端
#include "mqtt_reporter/MqttReporter.hpp"
#include "app_config/AppConfig.hpp"
#include "node_registry/NodeRegistry.hpp"

#include <cstdio>
#include <cstring>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"

namespace esp32hub {

static const char* TAG = "mqtt_reporter";

MqttReporter::~MqttReporter()
{
    if (events_ != nullptr) {
        vEventGroupDelete(events_);
    }
}

esp_err_t MqttReporter::Init(AppConfig* config, NodeRegistry* registry)
{
    config_ = config;
    registry_ = registry;

    events_ = xEventGroupCreate();
    if (events_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // 订阅 Wi-Fi 事件：拿到 IP 才启动 MQTT
    esp_err_t err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &MqttReporter::WifiEventHandler, this);
    if (err != ESP_OK) return err;
    err = esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                     &MqttReporter::WifiEventHandler, this);
    if (err != ESP_OK) return err;

    // 订阅 SensorPipeline 事件（数据 + 节点注册）
    err = esp_event_handler_register(kSensorDataEventBase, ESP_EVENT_ANY_ID,
                                     &MqttReporter::SensorEventHandler, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register sensor event handler failed: %s", esp_err_to_name(err));
        return err;
    }
    registered_handlers_ = true;

    // 启动自身状态机任务（内部 xTaskCreate）
    xTaskCreate(&MqttReporter::TaskMain, "mqtt_reporter", 6144, this, 5, &task_);
    ESP_LOGI(TAG, "mqtt reporter initialized");
    return ESP_OK;
}

void MqttReporter::TaskMain(void* arg)
{
    MqttReporter* self = static_cast<MqttReporter*>(arg);
    self->Run();
    vTaskDelete(nullptr);
}

void MqttReporter::WifiEventHandler(void* arg, esp_event_base_t base, int32_t id, void* /*data*/)
{
    MqttReporter* self = static_cast<MqttReporter*>(arg);
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(self->events_, kWifiOnline);
        xEventGroupClearBits(self->events_, kWifiOffline);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupSetBits(self->events_, kWifiOffline);
        xEventGroupClearBits(self->events_, kWifiOnline);
    }
}

void MqttReporter::Run()
{
    ESP_LOGI(TAG, "mqtt reporter task started, waiting for wifi online");
    for (;;) {
        // 等 Wi-Fi 在线
        xEventGroupWaitBits(events_, kWifiOnline, pdTRUE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "wifi online, starting mqtt client");
        StartClient();

        // 等 Wi-Fi 断开
        xEventGroupWaitBits(events_, kWifiOffline, pdTRUE, pdFALSE, portMAX_DELAY);
        ESP_LOGW(TAG, "wifi offline, stopping mqtt client");
        StopClient();
    }
}

// ================ MQTT 客户端生命周期 ================

void MqttReporter::StartClient()
{
    if (client_ != nullptr) {
        return;
    }

    const std::string& host = config_->BrokerHost();
    uint16_t port = config_->BrokerPort();
    char uri[96];
    std::snprintf(uri, sizeof(uri), "mqtt://%s:%u", host.c_str(), port);

    // 遗嘱状态主题（broker 检测到连接异常断开时由它代为发布 offline）
    char will_topic[64];
    std::snprintf(will_topic, sizeof(will_topic), "hub/%s/status",
                  registry_->RelayId());
    // note: will 结构与 uri 需在 esp_mqtt_client_init 调用期间保持有效，
    //       故使用静态缓冲（单实例场景安全）。
    static char s_uri[96];
    static char s_will_topic[64];
    std::strncpy(s_uri, uri, sizeof(s_uri) - 1);
    std::strncpy(s_will_topic, will_topic, sizeof(s_will_topic) - 1);

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = s_uri;
    cfg.session.keepalive = 30;
    cfg.session.last_will.topic = s_will_topic;
    cfg.session.last_will.msg = "offline";
    cfg.session.last_will.msg_len = strlen("offline");
    cfg.session.last_will.qos = 0;
    cfg.session.last_will.retain = true;
    cfg.session.disable_clean_session = false;

    client_ = esp_mqtt_client_init(&cfg);
    if (client_ == nullptr) {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        return;
    }
    esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY,
                                   &MqttReporter::MqttEventHandler, this);
    esp_err_t err = esp_mqtt_client_start(client_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_mqtt_client_start failed: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(client_);
        client_ = nullptr;
        return;
    }
    ESP_LOGI(TAG, "mqtt client started -> %s", s_uri);
}

void MqttReporter::StopClient()
{
    if (client_ == nullptr) {
        return;
    }
    esp_mqtt_client_stop(client_);
    esp_mqtt_client_destroy(client_);
    client_ = nullptr;
    connected_ = false;
}

// ================ MQTT 事件回调 ================

void MqttReporter::MqttEventHandler(void* arg, esp_event_base_t /*base*/, int32_t id, void* data)
{
    MqttReporter* self = static_cast<MqttReporter*>(arg);
    auto* event = static_cast<esp_mqtt_event_handle_t>(data);

    switch (id) {
    case MQTT_EVENT_CONNECTED:
        self->connected_ = true;
        ESP_LOGI(TAG, "mqtt connected");
        self->OnMqttConnected();
        break;
    case MQTT_EVENT_DISCONNECTED:
        self->connected_ = false;
        ESP_LOGW(TAG, "mqtt disconnected (will auto-reconnect)");
        break;
    case MQTT_EVENT_ERROR:
        self->connected_ = false;
        ESP_LOGW(TAG, "mqtt error, type=%d", event ? (int)event->error_handle->error_type : -1);
        break;
    default:
        break;
    }
}

void MqttReporter::OnMqttConnected()
{
    // 连接成功后立即发布在线状态与中继信息
    PublishStatus("online");
    PublishInfo();
}

// ================ 发布接口 ================

void MqttReporter::PublishStatus(const char* state)
{
    if (client_ == nullptr) return;
    char topic[64];
    std::snprintf(topic, sizeof(topic), "hub/%s/status", registry_->RelayId());
    // retained：PC 端后连接也能立刻看到当前状态
    esp_mqtt_client_publish(client_, topic, state, 0, 0, 1);
    ESP_LOGI(TAG, "published %s = %s", topic, state);
}

void MqttReporter::PublishInfo()
{
    if (client_ == nullptr) return;
    char topic[64];
    char payload[128];
    std::snprintf(topic, sizeof(topic), "hub/%s/info", registry_->RelayId());
    std::snprintf(payload, sizeof(payload),
                  "{\"relay_id\":\"%s\",\"fw\":\"esp32-hub/1.0\",\"nodes\":%d}",
                  registry_->RelayId(), registry_->Count());
    esp_mqtt_client_publish(client_, topic, payload, 0, 0, 1);
    ESP_LOGI(TAG, "published %s = %s", topic, payload);
}

void MqttReporter::PublishSensorData(const SensorPipeline::SensorData& data)
{
    if (client_ == nullptr || !connected_) {
        ESP_LOGW(TAG, "mqtt not connected, drop sensor data from %s", data.node_id);
        return;
    }
    char topic[128];
    char payload[384];
    std::snprintf(topic, sizeof(topic), "hub/%s/node/%s/data",
                  registry_->RelayId(), data.node_id);
    std::snprintf(payload, sizeof(payload),
                  "{\"relay_id\":\"%s\",\"node_id\":\"%s\",\"type\":\"%s\","
                  "\"ts\":%lld,\"values\":%s}",
                  registry_->RelayId(), data.node_id, data.type,
                  static_cast<long long>(data.ts),
                  data.values_json[0] ? data.values_json : "{}");
    esp_mqtt_client_publish(client_, topic, payload, 0, 0, 0);
    ESP_LOGD(TAG, "published %s = %s", topic, payload);
}

void MqttReporter::PublishNodeRegister(const SensorPipeline::NodeRegistered& reg)
{
    if (client_ == nullptr || !connected_) {
        ESP_LOGW(TAG, "mqtt not connected, drop register for %s", reg.node_id);
        return;
    }
    char topic[128];
    char payload[384];
    std::snprintf(topic, sizeof(topic), "hub/%s/node/%s/register",
                  registry_->RelayId(), reg.node_id);
    std::snprintf(payload, sizeof(payload),
                  "{\"relay_id\":\"%s\",\"node_id\":\"%s\",\"capability\":%s}",
                  registry_->RelayId(), reg.node_id,
                  reg.capability_json[0] ? reg.capability_json : "{}");
    // register 用 retained：PC 端后连接也能拿到节点能力
    esp_mqtt_client_publish(client_, topic, payload, 0, 0, 1);
    ESP_LOGI(TAG, "published %s = %s", topic, payload);
}

// ================ 传感器事件订阅 ================

void MqttReporter::SensorEventHandler(void* arg, esp_event_base_t /*base*/, int32_t id, void* data)
{
    MqttReporter* self = static_cast<MqttReporter*>(arg);
    switch (id) {
    case SensorPipeline::kEventSensorData:
        if (data != nullptr) {
            self->PublishSensorData(*static_cast<SensorPipeline::SensorData*>(data));
        }
        break;
    case SensorPipeline::kEventNodeOnline:
        if (data != nullptr) {
            self->PublishNodeRegister(*static_cast<SensorPipeline::NodeRegistered*>(data));
        }
        break;
    default:
        break;
    }
}

} // namespace esp32hub
