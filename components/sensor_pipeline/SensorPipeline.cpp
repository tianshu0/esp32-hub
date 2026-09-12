// SensorPipeline 实现：GATT 数据解析 + esp_event 投递
//
// 解析策略：
//   1. cJSON_Parse 整段 JSON
//   2. 取 type / ts / values 字段（缺失字段丢弃并告警）
//   3. 构造 SensorData 结构（含 node_id），esp_event_post 到 mqtt_reporter
//   4. 失败的 JSON 仅日志，不投递（避免污染下游）
#include "sensor_pipeline/SensorPipeline.hpp"

#include <cstring>
#include "esp_log.h"
#include "cJSON.h"

namespace esp32hub {

static const char* TAG = "sensor_pipeline";
const char* kSensorDataEventBase = "hub_sensor_data";

SensorPipeline::~SensorPipeline()
{
    Deinit();
}

esp_err_t SensorPipeline::Init()
{
    if (inited_) return ESP_OK;
    // 事件基 kSensorDataEventBase 挂在默认事件循环上（main 已 create_default），
    // 故这里无需再创建事件循环，仅做就绪标记。
    inited_ = true;
    ESP_LOGI(TAG, "sensor pipeline ready (event base: %s)", kSensorDataEventBase);
    return ESP_OK;
}

void SensorPipeline::Deinit()
{
    inited_ = false;
}

void SensorPipeline::Submit(uint16_t conn_handle, uint16_t /*attr_handle*/,
                            const std::string& json, const std::string& node_id)
{
    if (!inited_ || json.empty()) return;

    cJSON* root = cJSON_ParseWithLength(json.data(), json.size());
    if (root == nullptr) {
        ESP_LOGW(TAG, "json parse failed: %s", json.c_str());
        return;
    }

    cJSON* type = cJSON_GetObjectItem(root, "type");
    cJSON* ts   = cJSON_GetObjectItem(root, "ts");
    cJSON* vals = cJSON_GetObjectItem(root, "values");

    if (!cJSON_IsString(type) || !type->valuestring[0]) {
        ESP_LOGW(TAG, "missing type field: %s", json.c_str());
        cJSON_Delete(root);
        return;
    }

    SensorData data{};
    data.conn_handle = conn_handle;
    std::strncpy(data.node_id, node_id.c_str(), sizeof(data.node_id) - 1);
    std::strncpy(data.type, type->valuestring, sizeof(data.type) - 1);
    data.ts = cJSON_IsNumber(ts) ? (int64_t)ts->valuedouble : 0;

    // values 字段保留原文：mqtt_reporter 可直接转发，不需要重复构造
    if (cJSON_IsObject(vals) || cJSON_IsArray(vals)) {
        char* s = cJSON_PrintUnformatted(vals);
        if (s != nullptr) {
            std::strncpy(data.values_json, s, sizeof(data.values_json) - 1);
            cJSON_free(s);
        }
    }
    cJSON_Delete(root);

    esp_err_t err = esp_event_post(kSensorDataEventBase, kEventSensorData,
                                   &data, sizeof(data),
                                   0 /* 不指定任务，由系统分配 */);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_event_post failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGD(TAG, "sensor data posted: node=%s type=%s ts=%lld",
             data.node_id, data.type, (long long)data.ts);
}

void SensorPipeline::SubmitRegistration(const std::string& node_id,
                                        const std::string& capability_json)
{
    if (!inited_ || node_id.empty()) return;

    NodeRegistered reg{};
    std::strncpy(reg.node_id, node_id.c_str(), sizeof(reg.node_id) - 1);
    std::strncpy(reg.capability_json, capability_json.c_str(), sizeof(reg.capability_json) - 1);

    esp_err_t err = esp_event_post(kSensorDataEventBase, kEventNodeOnline,
                                   &reg, sizeof(reg), 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "post node registered event failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "node registered event posted: %s", reg.node_id);
}

} // namespace esp32hub
