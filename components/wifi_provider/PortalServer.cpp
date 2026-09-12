// PortalServer 实现：常驻 HTTP + DNS 劫持（仅 AP 模式）
//
// 与 esp32-broker 的区别：
//   - 不再有 status.html / RenderStatusPage：hub 是带屏设备，状态走 LVGL
//   - Start 不需要 wifi/broker 指针，只接受 credentials 回调
//   - 修复了 /save 中 password 字段名错误（broker 是 findVal("pass") 与前端 password 不匹配）
#include "wifi_provider/PortalServer.hpp"

#include <cstring>
#include <string>
#include <vector>
#include <cstdint>
#include <cstdio>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_timer.h"

namespace esp32hub {

// portal.html 通过 CMake EMBED_FILES 内嵌进固件
extern const uint8_t portal_html_start[] asm("_binary_portal_html_start");
extern const uint8_t portal_html_end[]   asm("_binary_portal_html_end");

static const char* TAG = "portal";

// 单例指针：静态 handler 访问成员
PortalServer* PortalServer::instance_ = nullptr;

// ---- URL 解码 ----
static std::string UrlDecode(const std::string& in)
{
    auto hexVal = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        if (in[i] == '+') {
            out += ' ';
        } else if (in[i] == '%' && i + 2 < in.size()) {
            int hi = hexVal(in[i + 1]);
            int lo = hexVal(in[i + 2]);
            if (hi >= 0 && lo >= 0) {
                out += static_cast<char>((hi << 4) | lo);
                i += 2;
            } else {
                out += in[i];
            }
        } else {
            out += in[i];
        }
    }
    return out;
}

PortalServer::~PortalServer()
{
    Stop();
}

esp_err_t PortalServer::Start(CredentialCallback on_credentials)
{
    if (http_running_) {
        ESP_LOGW(TAG, "Start called but HTTPD already running");
        return ESP_OK;
    }
    on_credentials_ = std::move(on_credentials);
    instance_ = this;
    return StartHttpd();
}

esp_err_t PortalServer::EnterApMode(esp_netif_t* ap_netif)
{
    ap_netif_ = ap_netif;
    return StartDns(ap_netif);
}

void PortalServer::ExitApMode()
{
    StopDns();
}

void PortalServer::Stop()
{
    StopDns();
    StopHttpd();
    instance_ = nullptr;
}

// ================ DNS 劫持 ================

esp_err_t PortalServer::StartDns(esp_netif_t* ap_netif)
{
    if (dns_running_) {
        return ESP_OK;
    }
    // 任务栈不足时 xTaskCreate 只返回错误、不打印日志，必须显式检查
    if (xTaskCreate(&PortalServer::DnsTask, "portal_dns", 4096, this, 5, &dns_task_) != pdPASS) {
        ESP_LOGE(TAG, "create portal_dns task failed, free heap %u B",
                 (unsigned)esp_get_free_heap_size());
        return ESP_ERR_NO_MEM;
    }
    dns_running_ = true;
    ESP_LOGI(TAG, "DNS hijack started (all domains -> AP IP)");
    return ESP_OK;
}

void PortalServer::StopDns()
{
    if (!dns_running_) {
        return;
    }
    dns_running_ = false;
    if (dns_task_ != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(300));
        dns_task_ = nullptr;
    }
}

void PortalServer::DnsTask(void* arg)
{
    PortalServer* self = static_cast<PortalServer*>(arg);
    self->DnsLoop();
    vTaskDelete(nullptr);
}

void PortalServer::DnsLoop()
{
    esp_netif_ip_info_t ip_info{};
    if (esp_netif_get_ip_info(ap_netif_, &ip_info) != ESP_OK) {
        ESP_LOGE(TAG, "get ap ip info failed");
        dns_running_ = false;
        return;
    }
    uint8_t ap_ip[4];
    std::memcpy(ap_ip, &ip_info.ip.addr, sizeof(ap_ip));

    int sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "dns socket create failed");
        dns_running_ = false;
        return;
    }

    timeval tv{};
    tv.tv_usec = 200 * 1000;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "dns bind :53 failed");
        ::close(sock);
        dns_running_ = false;
        return;
    }

    uint8_t buf[512];
    while (dns_running_) {
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        ssize_t n = ::recvfrom(sock, buf, sizeof(buf), 0,
                               reinterpret_cast<sockaddr*>(&from), &from_len);
        if (n <= 0) {
            continue;
        }
        if (n < 12 || (buf[2] & 0xF8) != 0 || (buf[4] == 0 && buf[5] == 0)) {
            continue;
        }

        int offset = 12;
        while (offset < n && buf[offset] != 0) {
            offset += 1 + buf[offset];
        }
        int qend = offset + 1 + 4;
        if (offset >= n || qend > n) continue;
        int qlen = qend - 12;

        uint8_t resp[512];
        int len_ = 0;
        auto put16 = [&](uint16_t v) {
            uint16_t be = htons(v);
            std::memcpy(resp + len_, &be, 2);
            len_ += 2;
        };
        auto put32 = [&](uint32_t v) {
            uint32_t be = htonl(v);
            std::memcpy(resp + len_, &be, 4);
            len_ += 4;
        };

        std::memcpy(resp, buf, 2);
        len_ = 2;
        put16(0x8180); put16(1); put16(1); put16(0); put16(0);
        std::memcpy(resp + len_, buf + 12, qlen);
        len_ += qlen;
        resp[len_++] = 0xC0; resp[len_++] = 0x0C;
        put16(1); put16(1); put32(60); put16(4);
        std::memcpy(resp + len_, ap_ip, 4); len_ += 4;

        ::sendto(sock, resp, len_, 0, reinterpret_cast<sockaddr*>(&from), from_len);
    }
    ::close(sock);
}

// ================ HTTP ================

esp_err_t PortalServer::StartHttpd()
{
    if (http_running_) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.stack_size = 6144;
    esp_err_t err = httpd_start(&server_, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t uri_root = {};
    uri_root.uri = "/"; uri_root.method = HTTP_GET;
    uri_root.handler = &PortalServer::HandleRoot;
    httpd_uri_t uri_scan = {};
    uri_scan.uri = "/scan"; uri_scan.method = HTTP_GET;
    uri_scan.handler = &PortalServer::HandleScan;
    httpd_uri_t uri_save = {};
    uri_save.uri = "/save"; uri_save.method = HTTP_POST;
    uri_save.handler = &PortalServer::HandleSave;
    httpd_uri_t uri_catch = {};
    uri_catch.uri = "/*"; uri_catch.method = HTTP_GET;
    uri_catch.handler = &PortalServer::HandleCatchAll;

    httpd_register_uri_handler(server_, &uri_root);
    httpd_register_uri_handler(server_, &uri_scan);
    httpd_register_uri_handler(server_, &uri_save);
    httpd_register_uri_handler(server_, &uri_catch);
    http_running_ = true;
    ESP_LOGI(TAG, "HTTP server started on :80");
    return ESP_OK;
}

void PortalServer::StopHttpd()
{
    if (server_ != nullptr) {
        httpd_stop(server_);
        server_ = nullptr;
    }
    http_running_ = false;
}

// ---- 路由：直接返回内嵌的 portal.html ----

esp_err_t PortalServer::HandleRoot(httpd_req_t* req)
{
    const char* data = reinterpret_cast<const char*>(portal_html_start);
    size_t size = static_cast<size_t>(reinterpret_cast<uintptr_t>(portal_html_end)
                                      - reinterpret_cast<uintptr_t>(portal_html_start));
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, data, static_cast<int>(size));
}

esp_err_t PortalServer::HandleCatchAll(httpd_req_t* req)
{
    return HandleRoot(req);
}

esp_err_t PortalServer::HandleScan(httpd_req_t* req)
{
    // 扫描必须在 APSTA 模式下进行；STA 模式调用会失败但不致命
    std::vector<wifi_ap_record_t> records(16);
    uint16_t number = static_cast<uint16_t>(records.size());
    esp_err_t err = esp_wifi_scan_start(nullptr, true);
    if (err == ESP_OK) err = esp_wifi_scan_get_ap_records(&number, records.data());
    esp_wifi_scan_stop();

    std::string json = "[";
    for (uint16_t i = 0; i < number; ++i) {
        if (i > 0) json += ",";
        bool secure = records[i].authmode != WIFI_AUTH_OPEN;
        char item[128];
        std::snprintf(item, sizeof(item), "{\"ssid\":\"%s\",\"rssi\":%d,\"secure\":%s}",
                      reinterpret_cast<const char*>(records[i].ssid), records[i].rssi,
                      secure ? "true" : "false");
        json += item;
    }
    json += "]";
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json.c_str(), static_cast<int>(json.size()));
}

esp_err_t PortalServer::HandleSave(httpd_req_t* req)
{
    if (instance_ == nullptr) return ESP_FAIL;

    char buf[256] = {0};
    int received = 0;
    int remaining = req->content_len;
    if (remaining < static_cast<int>(sizeof(buf))) {
        received = httpd_req_recv(req, buf, remaining);
    }
    buf[received > 0 ? received : 0] = '\0';

    std::string body(buf);
    auto findVal = [&body](const char* key) -> std::string {
        std::string k = std::string(key) + "=";
        size_t pos = body.find(k);
        if (pos == std::string::npos) return "";
        pos += k.size();
        size_t end = body.find('&', pos);
        if (end == std::string::npos) end = body.size();
        return body.substr(pos, end - pos);
    };
    std::string ssid = UrlDecode(findVal("ssid"));
    std::string pass = UrlDecode(findVal("password"));  // 字段名需与 portal.html 一致

    if (ssid.empty()) {
        httpd_resp_send(req, "SSID 不能为空", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    if (instance_->on_credentials_) {
        instance_->on_credentials_(ssid, pass);
    }
    httpd_resp_send(req, "已保存，设备正在连接 Wi-Fi...", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

} // namespace esp32hub
