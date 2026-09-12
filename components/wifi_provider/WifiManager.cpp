// WifiManager 实现：状态机 + 事件驱动
//
// 与 esp32-broker 的差异：StartPortal 不再注入 Broker*，仅注册 credentials 回调。
#include "wifi_provider/WifiManager.hpp"

#include <cstring>
#include <cstdio>
#include "esp_log.h"
#include "esp_mac.h"

namespace esp32hub {

static const char* TAG = "wifi_manager";

WifiManager::~WifiManager()
{
    if (events_ != nullptr) {
        vEventGroupDelete(events_);
    }
}

esp_err_t WifiManager::Init(AppConfig* config)
{
    config_ = config;

    // 创建默认 netif：STA 与 AP（供配网态使用）
    sta_netif_ = esp_netif_create_default_wifi_sta();
    ap_netif_ = esp_netif_create_default_wifi_ap();
    if (sta_netif_ == nullptr || ap_netif_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiManager::WifiEventHandler, this);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &WifiManager::WifiEventHandler, this);
    if (err != ESP_OK) {
        return err;
    }

    events_ = xEventGroupCreate();
    return ESP_OK;
}

void WifiManager::Start()
{
    xTaskCreate(&WifiManager::TaskMain, "wifi_mgr", 4096, this, 5, &task_);
}

esp_err_t WifiManager::StartPortal()
{
    // HTTP 常驻启动，仅注册 credentials 回调
    return portal_.Start(
        [this](const std::string& ssid, const std::string& password) {
            esp_err_t err = config_->SetWifiCreds(ssid, password);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "wifi creds saved: %s", ssid.c_str());
            } else {
                ESP_LOGE(TAG, "save creds failed: %s", esp_err_to_name(err));
            }
            xEventGroupSetBits(events_, kSaved);
        });
}

void WifiManager::RequestProvisioning()
{
    provision_requested_ = true;
    if (events_ != nullptr) {
        xEventGroupSetBits(events_, kDisconnect);
    }
}

void WifiManager::TaskMain(void* arg)
{
    WifiManager* self = static_cast<WifiManager*>(arg);
    self->Run();
    vTaskDelete(nullptr);
}

void WifiManager::WifiEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    WifiManager* self = static_cast<WifiManager*>(arg);
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        xEventGroupSetBits(self->events_, kStaStarted);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "STA got ip: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(self->events_, kGotIp);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        auto* event = static_cast<wifi_event_sta_disconnected_t*>(data);
        ESP_LOGW(TAG, "STA disconnected, reason: %d", event->reason);
        xEventGroupSetBits(self->events_, kDisconnect);
    }
}

void WifiManager::Run()
{
    ESP_LOGI(TAG, "wifi manager task started");

    for (;;) {
        if (provision_requested_ || !config_->HasWifiCreds()) {
            // 无凭据 / 被请求重新配网 -> 进 AP 配网态
            RunProvisioning();
            continue;
        }

        // 有凭据 -> 尝试 STA 连接
        if (TryConnectStation()) {
            mode_ = Mode::kOnline;
            connect_failures_ = 0;
            ESP_LOGI(TAG, "STA connected, broker/mqtt will run");

            // 在线态：等待断开或用户主动请求重配
            xEventGroupClearBits(events_, kDisconnect);
            for (;;) {
                EventBits_t bits = xEventGroupWaitBits(events_, kDisconnect | kGotIp,
                                                       pdTRUE, pdFALSE, portMAX_DELAY);
                if (bits & kDisconnect) {
                    ESP_LOGW(TAG, "link lost, reconnecting...");
                    break;
                }
            }
            mode_ = Mode::kIdle;
        } else {
            if (++connect_failures_ >= kMaxConnectFailures) {
                ESP_LOGE(TAG, "STA connect failed %d times, entering AP provisioning",
                         connect_failures_);
                connect_failures_ = 0;
                RunProvisioning();
            } else {
                ESP_LOGW(TAG, "STA connect failed (attempt %d), retrying", connect_failures_);
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
        }
    }
}

bool WifiManager::TryConnectStation()
{
    // 先完全停止 Wi-Fi，保证每次尝试都重发 STA_START，状态机确定
    esp_wifi_stop();
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(STA) failed: %s", esp_err_to_name(err));
        return false;
    }

    wifi_config_t wifi_cfg{};
    std::strncpy(reinterpret_cast<char*>(wifi_cfg.sta.ssid), config_->WifiSsid().c_str(), 32);
    std::strncpy(reinterpret_cast<char*>(wifi_cfg.sta.password), config_->WifiPassword().c_str(), 64);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(STA) failed: %s", esp_err_to_name(err));
        return false;
    }

    // 在 start 之前清位，避免 STA_START 事件早于清除而丢失
    xEventGroupClearBits(events_, kStaStarted | kDisconnect | kGotIp);

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return false;
    }
    return WaitGotIp(kConnectTimeoutMs);
}

bool WifiManager::WaitGotIp(TickType_t timeout_ms)
{
    // esp_wifi_start() 之后异步收到 STA_START 事件
    EventBits_t bits = xEventGroupWaitBits(events_, kStaStarted | kDisconnect,
                                           pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
    if ((bits & kStaStarted) == 0) {
        return false;
    }

    esp_wifi_connect();

    bits = xEventGroupWaitBits(events_, kGotIp | kDisconnect, pdTRUE, pdFALSE, timeout_ms);
    return (bits & kGotIp) != 0;
}

void WifiManager::RunProvisioning()
{
    mode_ = Mode::kApProvisioning;
    provision_requested_ = false;
    ESP_LOGI(TAG, "entering AP provisioning mode");

    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_APSTA);

    wifi_config_t ap_cfg{};
    std::string ssid = MakeApSsid();
    std::strncpy(reinterpret_cast<char*>(ap_cfg.ap.ssid), ssid.c_str(), sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len = static_cast<uint8_t>(ssid.size());
    ap_cfg.ap.channel = 1;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN; // 配网热点不加密，后续可通过 Portal 加密码

    esp_err_t err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) failed: %s", esp_err_to_name(err));
        mode_ = Mode::kIdle;
        return;
    }
    esp_wifi_start();

    // 启动 DNS 劫持（HTTPD 已常驻，这里只开 DNS）
    xEventGroupClearBits(events_, kSaved);
    portal_.EnterApMode(ap_netif_);
    ESP_LOGI(TAG, "portal running, waiting for credentials...");
    xEventGroupWaitBits(events_, kSaved, pdTRUE, pdFALSE, portMAX_DELAY);
    portal_.ExitApMode();
    ESP_LOGI(TAG, "credentials saved, leaving provisioning mode");

    esp_wifi_stop();
    mode_ = Mode::kIdle;
    // portal 启动失败时避免紧循环
    if (err != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

std::string WifiManager::MakeApSsid() const
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char ssid[32];
    std::snprintf(ssid, sizeof(ssid), "esp32-hub-%02X%02X", mac[4], mac[5]);
    return ssid;
}

std::string WifiManager::CurrentSsid() const
{
    if (mode_ != Mode::kOnline || config_ == nullptr) {
        return std::string();
    }
    return config_->WifiSsid();
}

std::string WifiManager::CurrentIp() const
{
    if (mode_ != Mode::kOnline || sta_netif_ == nullptr) {
        return std::string();
    }
    esp_netif_ip_info_t info{};
    if (esp_netif_get_ip_info(sta_netif_, &info) != ESP_OK) {
        return std::string();
    }
    char ip[16] = {0};
    std::snprintf(ip, sizeof(ip), IPSTR, IP2STR(&info.ip));
    return ip;
}

} // namespace esp32hub
