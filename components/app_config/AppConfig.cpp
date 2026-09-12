#include "app_config/AppConfig.hpp"

#include <cstring>
#include <vector>
#include "esp_log.h"
#include "nvs_flash.h"

namespace esp32hub {

static const char* TAG = "app_config";
static constexpr const char* kNamespace = "esp32hub";

// NVS 键名（长度必须 <= 15）
static constexpr const char* kKeySsid       = "wifi_ssid";
static constexpr const char* kKeyPassword  = "wifi_password";
static constexpr const char* kKeyRelayId   = "relay_id";
static constexpr const char* kKeyBrokerHost = "broker_host";
static constexpr const char* kKeyBrokerPort = "broker_port";

AppConfig::~AppConfig()
{
    if (open_) {
        nvs_close(handle_);
    }
}

esp_err_t AppConfig::Init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS 分区损坏/版本升级：擦除后重试
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_open(kNamespace, NVS_READWRITE, &handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }
    open_ = true;
    return Load();
}

esp_err_t AppConfig::Load()
{
    if (!open_) {
        return ESP_ERR_INVALID_STATE;
    }
    wifi_ssid_      = ReadString(handle_, kKeySsid, "");
    wifi_password_ = ReadString(handle_, kKeyPassword, "");
    relay_id_      = ReadString(handle_, kKeyRelayId, "");
    broker_host_   = ReadString(handle_, kKeyBrokerHost, "esp32-broker.local");

    uint16_t port = 1883;
    if (nvs_get_u16(handle_, kKeyBrokerPort, &port) != ESP_OK) {
        port = 1883;
    }
    broker_port_ = port;
    return ESP_OK;
}

std::string AppConfig::ReadString(nvs_handle_t handle, const char* key, const std::string& fallback) const
{
    // 第一次调用返回长度（含结尾 '\0'）
    size_t len = 0;
    if (nvs_get_str(handle, key, nullptr, &len) != ESP_OK) {
        return fallback;
    }
    if (len <= 1) {
        return fallback; // 空串或无数据
    }
    std::string value(len - 1, '\0');
    len = value.size() + 1; // 传入缓冲容量（含结尾 '\0'）
    if (nvs_get_str(handle, key, &value[0], &len) == ESP_OK) {
        return value; // 不含结尾 '\0'
    }
    return fallback;
}

esp_err_t AppConfig::WriteString(const char* key, const std::string& value)
{
    if (!open_) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = nvs_set_str(handle_, key, value.c_str());
    if (err == ESP_OK) {
        err = nvs_commit(handle_);
    }
    return err;
}

esp_err_t AppConfig::SetWifiCreds(const std::string& ssid, const std::string& password)
{
    if (ssid.empty() || ssid.size() > 32 || password.size() > 64) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = WriteString(kKeySsid, ssid);
    if (err == ESP_OK) {
        err = WriteString(kKeyPassword, password);
    }
    if (err == ESP_OK) {
        wifi_ssid_ = ssid;
        wifi_password_ = password;
    }
    return err;
}

esp_err_t AppConfig::ClearWifiCreds()
{
    esp_err_t err = nvs_erase_key(handle_, kKeySsid);
    esp_err_t err2 = nvs_erase_key(handle_, kKeyPassword);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err2 == ESP_ERR_NVS_NOT_FOUND) {
        err2 = ESP_OK;
    }
    if (err == ESP_OK && err2 == ESP_OK) {
        err = nvs_commit(handle_);
    }
    if (err == ESP_OK) {
        wifi_ssid_.clear();
        wifi_password_.clear();
    }
    return err;
}

esp_err_t AppConfig::SetRelayId(const std::string& id)
{
    if (id.empty() || id.size() > 31) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = WriteString(kKeyRelayId, id);
    if (err == ESP_OK) {
        relay_id_ = id;
    }
    return err;
}

esp_err_t AppConfig::SetBrokerHost(const std::string& host)
{
    if (host.empty() || host.size() > 63) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = WriteString(kKeyBrokerHost, host);
    if (err == ESP_OK) {
        broker_host_ = host;
    }
    return err;
}

esp_err_t AppConfig::SetBrokerPort(uint16_t port)
{
    if (!open_) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = nvs_set_u16(handle_, kKeyBrokerPort, port);
    if (err == ESP_OK) {
        err = nvs_commit(handle_);
    }
    if (err == ESP_OK) {
        broker_port_ = port;
    }
    return err;
}

} // namespace esp32hub
