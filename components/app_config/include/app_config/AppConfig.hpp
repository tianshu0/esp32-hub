// app_config 组件：基于 NVS 的持久化配置存取
// 本组件提供 Wi-Fi 凭据、中继 ID、Broker 地址、屏幕参数等配置项的读写。
//
// 与 esp32-broker 的区别：
//   - 多了 relay_id（中继 ID，基于 MAC 生成后固定）
//   - 多了 broker_host / broker_port（hub 作为 MQTT 客户端连远程 broker）
//   - 不再有 mqtt_port（broker 监听端口），改由 hub 作为客户端去连
#pragma once

#include <string>
#include <cstdint>
#include "esp_err.h"
#include "nvs.h"

namespace esp32hub {

class AppConfig {
public:
    AppConfig() = default;
    ~AppConfig();

    // 初始化 NVS（内部调用 nvs_flash_init，并对首启/损坏做 erase 重试）
    esp_err_t Init();

    // ---- Wi-Fi STA 凭据 ----
    bool HasWifiCreds() const { return !wifi_ssid_.empty(); }
    const std::string& WifiSsid() const { return wifi_ssid_; }
    const std::string& WifiPassword() const { return wifi_password_; }
    esp_err_t SetWifiCreds(const std::string& ssid, const std::string& password);
    esp_err_t ClearWifiCreds();

    // ---- 中继 ID（持久化首次生成值，作为 MQTT 主题与配对标识）----
    const std::string& RelayId() const { return relay_id_; }
    esp_err_t SetRelayId(const std::string& id);

    // ---- MQTT Broker 地址（主机名或 IP，如 esp32-broker.local / 192.168.1.10）----
    const std::string& BrokerHost() const { return broker_host_; }
    esp_err_t SetBrokerHost(const std::string& host);
    uint16_t BrokerPort() const { return broker_port_; }
    esp_err_t SetBrokerPort(uint16_t port);

    // 重新从 NVS 加载全部配置
    esp_err_t Load();

private:
    std::string ReadString(nvs_handle_t handle, const char* key, const std::string& fallback) const;
    esp_err_t WriteString(const char* key, const std::string& value);

    nvs_handle_t handle_ = 0;
    bool open_ = false;

    std::string wifi_ssid_;
    std::string wifi_password_;
    std::string relay_id_;             // 形如 "hub-001A2B"，首次启动基于 MAC 生成
    std::string broker_host_ = "esp32-broker.local";
    uint16_t broker_port_ = 1883;
};

} // namespace esp32hub
