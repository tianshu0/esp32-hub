// wifi_provider：Web 服务（HTTP 常驻 + DNS 劫持仅 AP 模式）
//
// 与 esp32-broker 的区别：
//   - 状态页不在 Web 上展示（hub 是带屏设备，状态由 display_service 渲染到 JD9853 屏）
//   - PortalServer 只保留配网页（portal.html），不再注入 Broker 指针
//
// HTTP 服务器常驻：
//   - AP 配网态：服务 Wi-Fi 配置页（配合 DNS 劫持实现 Captive Portal）
//   - STA 在线态：HTTPD 仍常驻，但目前仅留 /save 兜底（后续可扩展远程查询接口）
// DNS 劫持（UDP:53 -> AP IP 192.168.4.1）仅在 AP 配网态启用。
#pragma once

#include <atomic>
#include <functional>
#include <string>
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esp32hub {
class WifiManager;

class PortalServer {
public:
    using CredentialCallback = std::function<void(const std::string& ssid, const std::string& password)>;

    PortalServer() = default;
    ~PortalServer();

    // 启动常驻 HTTPD；注册凭据保存回调
    esp_err_t Start(CredentialCallback on_credentials);

    // 切换到 AP 模式：启动 DNS 劫持（把所有域名解析到 AP IP）
    esp_err_t EnterApMode(esp_netif_t* ap_netif);
    // 退出 AP 模式：关闭 DNS 劫持（HTTPD 继续运行）
    void ExitApMode();

    // 完全停止（HTTPD + DNS）
    void Stop();

private:
    static void DnsTask(void* arg);
    void DnsLoop();
    esp_err_t StartDns(esp_netif_t* ap_netif);
    void StopDns();

    static esp_err_t HandleRoot(httpd_req_t* req);
    static esp_err_t HandleScan(httpd_req_t* req);
    static esp_err_t HandleSave(httpd_req_t* req);
    static esp_err_t HandleCatchAll(httpd_req_t* req);

    esp_err_t StartHttpd();
    void StopHttpd();

    CredentialCallback on_credentials_;
    httpd_handle_t server_ = nullptr;
    TaskHandle_t dns_task_ = nullptr;
    esp_netif_t* ap_netif_ = nullptr;
    std::atomic<bool> http_running_{false};
    std::atomic<bool> dns_running_{false};

    // 单例指针：静态 handler 通过它访问成员
    static PortalServer* instance_;
};

} // namespace esp32hub
