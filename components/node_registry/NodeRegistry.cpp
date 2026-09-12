// NodeRegistry 实现：内存表 + SPIFFS 持久化 + RelayId 生成
//
// 与 esp32-broker 的区别：
//   - broker 的 app_config 单条 NVS 即可承载 Wi-Fi 凭据（固定字段）
//   - hub 的配对表字段不固定（能力清单），用 SPIFFS 存 JSON 文件更合适
//   - NVS 仅保存 relay_id（单字段，与配对表分离）
#include "node_registry/NodeRegistry.hpp"
#include "app_config/AppConfig.hpp"

#include <cstring>
#include <cstdio>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_spiffs.h"
#include "cJSON.h"

namespace esp32hub {

static const char* TAG = "node_registry";
static constexpr const char* kStoragePartition = "storage";
static constexpr const char* kRegistryFile = "/storage/registry.json";

NodeRegistry::~NodeRegistry()
{
    // SPIFFS 不主动卸载（系统生命周期内常驻）
}

esp_err_t NodeRegistry::Init(AppConfig* config)
{
    config_ = config;

    // 挂载 SPIFFS（用于配对表持久化）
    esp_vfs_spiffs_conf_t spiffs_cfg = {};
    spiffs_cfg.base_path = "/storage";
    spiffs_cfg.partition_label = kStoragePartition;
    spiffs_cfg.format_if_mount_failed = true; // 损坏自动格式化
    spiffs_cfg.max_files = 4;
    esp_err_t err = esp_vfs_spiffs_register(&spiffs_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spiffs mount failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "spiffs mounted at /storage");

    // 加载持久化配对表（文件不存在则视为空表）
    LoadFromFile();

    // 确保 relay_id 存在（首次启动生成并保存到 AppConfig）
    EnsureRelayId();
    ESP_LOGI(TAG, "relay_id = %s, paired nodes = %d", relay_id_.c_str(), count_);
    return ESP_OK;
}

esp_err_t NodeRegistry::LoadFromFile()
{
    FILE* f = fopen(kRegistryFile, "rb");
    if (f == nullptr) {
        ESP_LOGI(TAG, "registry file not found, starting empty");
        return ESP_OK; // 不视为错误
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 8192) {
        fclose(f);
        ESP_LOGW(TAG, "registry file size invalid: %ld", sz);
        return ESP_ERR_INVALID_SIZE;
    }
    std::string buf(static_cast<size_t>(sz), '\0');
    fread(&buf[0], 1, buf.size(), f);
    fclose(f);

    cJSON* root = cJSON_ParseWithLength(buf.data(), buf.size());
    if (root == nullptr) {
        ESP_LOGW(TAG, "registry json parse failed, starting empty");
        return ESP_ERR_INVALID_STATE;
    }
    cJSON* arr = cJSON_GetObjectItem(root, "nodes");
    if (cJSON_IsArray(arr)) {
        int n = cJSON_GetArraySize(arr);
        if (n > kMaxNodes) n = kMaxNodes;
        for (int i = 0; i < n; ++i) {
            cJSON* item = cJSON_GetArrayItem(arr, i);
            if (!cJSON_IsObject(item)) continue;
            cJSON* id = cJSON_GetObjectItem(item, "node_id");
            cJSON* sm  = cJSON_GetObjectItem(item, "sensor_summary");
            if (cJSON_IsString(id) && id->valuestring[0]) {
                nodes_[count_].node_id = id->valuestring;
                if (cJSON_IsString(sm)) {
                    nodes_[count_].sensor_summary = sm->valuestring;
                }
                nodes_[count_].online = false; // 启动时所有节点离线
                nodes_[count_].last_seen_us = 0;
                ++count_;
            }
        }
    }
    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t NodeRegistry::SaveToFile() const
{
    cJSON* root = cJSON_CreateObject();
    cJSON* arr = cJSON_AddArrayToObject(root, "nodes");
    for (int i = 0; i < count_; ++i) {
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "node_id", nodes_[i].node_id.c_str());
        cJSON_AddStringToObject(item, "sensor_summary", nodes_[i].sensor_summary.c_str());
        cJSON_AddItemToArray(arr, item);
    }
    char* str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (str == nullptr) return ESP_ERR_NO_MEM;

    FILE* f = fopen(kRegistryFile, "wb");
    if (f == nullptr) {
        cJSON_free(str);
        ESP_LOGE(TAG, "open %s for write failed", kRegistryFile);
        return ESP_FAIL;
    }
    fwrite(str, 1, std::strlen(str), f);
    fclose(f);
    cJSON_free(str);
    return ESP_OK;
}

void NodeRegistry::EnsureRelayId()
{
    if (!relay_id_.empty()) return;

    // 优先从 AppConfig 读取（已配网过会存在 NVS）
    if (config_ && !config_->RelayId().empty()) {
        relay_id_ = config_->RelayId();
        return;
    }

    // 首次启动：基于 WiFi MAC 生成，如 "hub-1A2B"
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "hub-%02X%02X", mac[4], mac[5]);
    relay_id_ = buf;

    // 保存到 AppConfig（NVS），下次直接读取
    if (config_) {
        config_->SetRelayId(relay_id_);
    }
    ESP_LOGI(TAG, "generated relay_id: %s", relay_id_.c_str());
}

esp_err_t NodeRegistry::RegisterNode(const std::string& node_id, const std::string& capability_json)
{
    if (node_id.empty()) return ESP_ERR_INVALID_ARG;

    // 解析能力清单 JSON（{"types":["temp_hum","pressure"]}），生成 sensor_summary
    std::string summary;
    cJSON* cap = cJSON_ParseWithLength(capability_json.data(), capability_json.size());
    if (cap != nullptr) {
        cJSON* types = cJSON_GetObjectItem(cap, "types");
        if (cJSON_IsArray(types)) {
            int n = cJSON_GetArraySize(types);
            for (int i = 0; i < n; ++i) {
                cJSON* t = cJSON_GetArrayItem(types, i);
                if (cJSON_IsString(t)) {
                    if (!summary.empty()) summary += ",";
                    summary += t->valuestring;
                }
            }
        }
        cJSON_Delete(cap);
    }

    // 已存在则更新，否则新增
    for (int i = 0; i < count_; ++i) {
        if (nodes_[i].node_id == node_id) {
            nodes_[i].sensor_summary = summary;
            nodes_[i].online = true;
            nodes_[i].last_seen_us = esp_timer_get_time();
            SaveToFile();
            ESP_LOGI(TAG, "node updated: %s [%s]", node_id.c_str(), summary.c_str());
            return ESP_OK;
        }
    }
    if (count_ >= kMaxNodes) {
        ESP_LOGW(TAG, "node table full, ignoring new node %s", node_id.c_str());
        return ESP_ERR_NO_MEM;
    }
    nodes_[count_].node_id = node_id;
    nodes_[count_].sensor_summary = summary;
    nodes_[count_].online = true;
    nodes_[count_].last_seen_us = esp_timer_get_time();
    ++count_;
    SaveToFile();
    ESP_LOGI(TAG, "node registered: %s [%s]", node_id.c_str(), summary.c_str());
    return ESP_OK;
}

void NodeRegistry::UpdateDataTime(const std::string& node_id)
{
    for (int i = 0; i < count_; ++i) {
        if (nodes_[i].node_id == node_id) {
            nodes_[i].last_seen_us = esp_timer_get_time();
            nodes_[i].online = true;
            return;
        }
    }
}

void NodeRegistry::MarkOffline(const std::string& node_id)
{
    for (int i = 0; i < count_; ++i) {
        if (nodes_[i].node_id == node_id) {
            nodes_[i].online = false;
            return;
        }
    }
}

NodeRegistry::NodeInfo NodeRegistry::GetByIndex(int index) const
{
    if (index < 0 || index >= count_) {
        return NodeInfo{};
    }
    return nodes_[index];
}

} // namespace esp32hub
