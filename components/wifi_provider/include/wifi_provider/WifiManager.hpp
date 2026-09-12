// wifi_provider：Wi-Fi STA 连接管理 + AP 配网回退
//
// 与 esp32-broker 的区别：
//   - 不再注入 Broker 指针：hub 是 MQTT 客户端，连远端 broker；本地状态页交给 display_service 走 LVGL
//   - StartPortal() 不带参数：HTTPD 只服务 portal.html，不再渲染 broker 状态页
//
// WifiManager 负责状态机（连接 STA -> 失败自动进 AP 配网态 -> 保存凭据后重连），
// 配网页/Portal 由 PortalServer 提供，两个类在 Init/Start 内各自 xTaskCreate。
#pragma once

#include <string>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "app_config/AppConfig.hpp"
#include "wifi_provider/PortalServer.hpp"

namespace esp32hub {

class WifiManager {
public:
    WifiManager() = default;
    ~WifiManager();

    // 创建默认 STA/AP netif、初始化 Wi-Fi、注册事件、创建 Portal DNS/HTTP 所需的资源
    esp_err_t Init(AppConfig* config);
    // 启动管理任务（内部 xTaskCreate），此后状态机自主运行
    void Start();
    // 用户主动触发重新配网（可接到 GPIO/Web 按钮）
    void RequestProvisioning();

    // HTTP 常驻服务启动（在 wifi Init 之后调用）
    // 凭据保存后写入 AppConfig 并唤醒状态机重连
    esp_err_t StartPortal();

    // 访问 PortalServer（供外部停止/查询）
    PortalServer& portal() { return portal_; }

    // ---- 状态查询（供 display_service 等消费者读取）----
    bool IsOnline() const { return mode_ == Mode::kOnline; }
    bool IsProvisioning() const { return mode_ == Mode::kApProvisioning; }
    // 当前连接的 STA SSID（未连接时为空）
    std::string CurrentSsid() const;
    // 当前 STA IP 字符串（如 "192.168.1.100"，未连接时为空）
    std::string CurrentIp() const;
    // 配网热点 SSID（开放网络、无密码，基于 MAC 生成，每次调用结果一致）
    std::string ApSsid() const { return MakeApSsid(); }

private:
    static constexpr int kMaxConnectFailures = 3;
    static constexpr TickType_t kConnectTimeoutMs = 20000;
    static constexpr EventBits_t kGotIp      = 1 << 0;
    static constexpr EventBits_t kDisconnect = 1 << 1;
    static constexpr EventBits_t kStaStarted = 1 << 2;
    static constexpr EventBits_t kSaved      = 1 << 3;

    enum class Mode { kIdle, kSta, kApProvisioning, kOnline };

    static void TaskMain(void* arg);
    static void WifiEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data);

    void Run();
    bool TryConnectStation();
    void RunProvisioning();
    bool WaitGotIp(TickType_t timeout_ms);
    std::string MakeApSsid() const;

    AppConfig* config_ = nullptr;
    EventGroupHandle_t events_ = nullptr;
    TaskHandle_t task_ = nullptr;
    esp_netif_t* sta_netif_ = nullptr;
    esp_netif_t* ap_netif_ = nullptr;
    PortalServer portal_;
    Mode mode_ = Mode::kIdle;
    int connect_failures_ = 0;
    bool provision_requested_ = false;
};

} // namespace esp32hub
