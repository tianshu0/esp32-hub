// BleCentral 实现：NimBLE Central 角色
//
// 状态机：
//   WiFi 在线 -> 扫描 -> 发现 esp32-node 广播 -> 连接
//              -> MTU 交换 -> 服务发现（ff00 服务 / ff01 写 / ff02 通知）
//              -> 订阅 CCC + 发 hello -> 收 hello_ack 写 registry -> 收数据交 pipeline
//   BLE 断开 -> 标记节点离线 -> 回到扫描
//
// 为避免循环依赖，ble_central 不引入 mqtt_reporter / display_service：
//   - 数据经 SensorPipeline::Submit 投递到事件总线，由 mqtt_reporter 订阅上报
//   - display_service 主动查询 NodeRegistry 获取节点表
#include "ble_central/BleCentral.hpp"
#include "app_config/AppConfig.hpp"
#include "node_registry/NodeRegistry.hpp"
#include "sensor_pipeline/SensorPipeline.hpp"

#include <cstring>
#include <cstdio>
#include <string>
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"

namespace esp32hub {

static const char* TAG = "ble_central";

// 自定义服务/特征值 UUID（esp32-node 协议）
// BLE_UUID128_INIT 的字节序为「小端」：value[0] 是 UUID 字符串的最后一个字节。
//   服务：  0000ff00-0000-1000-8000-00805f9b34fb
//   WRITE： 0000ff01-0000-1000-8000-00805f9b34fb
//   NOTIFY：0000ff02-0000-1000-8000-00805f9b34fb
static const ble_uuid128_t kSvcUuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00);
static const ble_uuid128_t kChrWriteUuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00);
static const ble_uuid128_t kChrNotifyUuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x02, 0xff, 0x00, 0x00);

// 扫描时长（单轮），之后重启扫描以持续发现新设备
static constexpr int32_t kScanDurationMs = 10000;
// 连接超时
static constexpr int32_t kConnectTimeoutMs = 10000;
// 首选 MTU
static constexpr uint16_t kPreferredMtu = 256;

BleCentral::~BleCentral()
{
    if (events_ != nullptr) {
        vEventGroupDelete(events_);
    }
}

esp_err_t BleCentral::Init(AppConfig* config, NodeRegistry* registry, SensorPipeline* pipeline)
{
    config_ = config;
    registry_ = registry;
    pipeline_ = pipeline;

    events_ = xEventGroupCreate();
    if (events_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // 声明本端偏好的 MTU（实际生效值在连接后由 MTU 交换协商）
    ble_att_set_preferred_mtu(kPreferredMtu);

    // 注册 Wi-Fi 事件：拿到 IP 才启动 BLE 扫描
    esp_err_t err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &BleCentral::WifiEventHandler, this);
    if (err != ESP_OK) return err;
    err = esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                     &BleCentral::WifiEventHandler, this);
    if (err != ESP_OK) return err;

    // 启动 NimBLE host 事件循环（nimble_port_init 已在 main 中调用）
    nimble_port_freertos_init(&BleCentral::HostTask);

    // 启动自身状态机任务（组件自包含生命周期）
    xTaskCreate(&BleCentral::TaskMain, "ble_central", 6144, this, 6, &task_);
    ESP_LOGI(TAG, "ble central initialized");
    return ESP_OK;
}

void BleCentral::HostTask(void* /*arg*/)
{
    // 事件循环返回即协议栈停止，届时释放 host
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void BleCentral::WifiEventHandler(void* arg, esp_event_base_t base, int32_t id, void* /*data*/)
{
    BleCentral* self = static_cast<BleCentral*>(arg);
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(self->events_, kWifiOnline);
        xEventGroupClearBits(self->events_, kWifiOffline);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupSetBits(self->events_, kWifiOffline);
        xEventGroupClearBits(self->events_, kWifiOnline);
    }
}

void BleCentral::TaskMain(void* arg)
{
    BleCentral* self = static_cast<BleCentral*>(arg);
    self->Run();
    vTaskDelete(nullptr);
}

void BleCentral::Run()
{
    ESP_LOGI(TAG, "ble central task started, waiting for wifi online");
    for (;;) {
        // 等 Wi-Fi 在线
        xEventGroupWaitBits(events_, kWifiOnline, pdTRUE, pdFALSE, portMAX_DELAY);

        ESP_LOGI(TAG, "wifi online, start ble scan loop");
        while ((xEventGroupGetBits(events_) & kWifiOffline) == 0) {
            if (IsConnected()) {
                // 已连接：休眠等待断开事件（断开时 GapEventCb 置 kNodeLost）
                xEventGroupWaitBits(events_, kNodeLost, pdTRUE, pdFALSE,
                                    pdMS_TO_TICKS(2000));
            } else {
                StartScan();
                // StartScan 是异步的；GAP 回调里发现设备会主动 StopScan + 连接
                xEventGroupWaitBits(events_, kNodeLost, pdTRUE, pdFALSE,
                                    pdMS_TO_TICKS(kScanDurationMs));
                StopScan();
            }
        }
        ESP_LOGW(TAG, "wifi offline, pause ble scan");
    }
}

// ================ 扫描 ================

void BleCentral::StartScan()
{
    if (scanning_) return;

    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    struct ble_gap_disc_params disc = {};
    disc.itvl = 0x60;              // 60ms 扫描间隔
    disc.window = 0x30;            // 30ms 扫描窗口
    disc.filter_duplicates = 1;    // 同一设备只上报一次
    disc.passive = 0;              // 主动扫描（可拿 scan response，含完整服务 UUID 列表）

    rc = ble_gap_disc(own_addr_type, kScanDurationMs, &disc,
                      &BleCentral::GapEventCb, this);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
        return;
    }
    scanning_ = true;
    ESP_LOGI(TAG, "scan started");
}

void BleCentral::StopScan()
{
    if (!scanning_) return;
    ble_gap_disc_cancel();
    scanning_ = false;
}

// ================ GAP 事件回调 ================

// 在广播数据中查找 128 位服务 UUID（AD type 0x06 完整列表 / 0x07 不完整列表）
static bool AdvContainsSvc(const uint8_t* data, uint8_t len, const ble_uuid128_t& svc)
{
    uint8_t i = 0;
    while (i + 1 < len) {
        uint8_t ad_len = data[i];
        if (ad_len == 0) break;               // 结束标记
        if (i + 1 + ad_len > len) break;      // 越界保护
        uint8_t ad_type = data[i + 1];
        if (ad_type == 0x06 || ad_type == 0x07) {
            const uint8_t* p = data + i + 2;
            int n = ad_len - 1;
            // 逐个 16 字节 UUID 比较（BLE_UUID128_INIT 已按小端存放）
            for (int off = 0; off + 16 <= n; off += 16) {
                if (std::memcmp(p + off, svc.value, 16) == 0) {
                    return true;
                }
            }
        }
        i += 1 + ad_len;
    }
    return false;
}

int BleCentral::GapEventCb(ble_gap_event* event, void* arg)
{
    BleCentral* self = static_cast<BleCentral*>(arg);

    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        const auto& disc = event->disc;
        if (!AdvContainsSvc(disc.data, disc.length_data, kSvcUuid)) {
            return 0;   // 非 esp32-node，忽略
        }
        ESP_LOGI(TAG, "found esp32-node: %02x:%02x:%02x:%02x:%02x:%02x",
                 disc.addr.val[5], disc.addr.val[4], disc.addr.val[3],
                 disc.addr.val[2], disc.addr.val[1], disc.addr.val[0]);

        self->StopScan();
        self->AttemptConnect(disc.addr);
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "connect failed: status=%d", event->connect.status);
            self->conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
            xEventGroupSetBits(self->events_, kNodeLost);
            return 0;
        }
        self->conn_handle_ = event->connect.conn_handle;
        ESP_LOGI(TAG, "connected, handle=%d", self->conn_handle_);

        // 发起 MTU 交换（异步，完成后触发 BLE_GAP_EVENT_MTU）
        ble_gattc_exchange_mtu(self->conn_handle_, nullptr, nullptr);
        // 开始服务发现（失败则断开重试）
        self->DiscoverService(self->conn_handle_);
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGW(TAG, "disconnected, reason=%d", event->disconnect.reason);
        if (self->registry_ != nullptr && !self->node_id_.empty()) {
            self->registry_->MarkOffline(self->node_id_);
        }
        self->conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
        self->paired_ = false;
        self->node_id_.clear();
        self->svc_start_ = self->svc_end_ = 0;
        self->h_write_ = self->h_notify_ = 0;
        xEventGroupSetBits(self->events_, kNodeLost);
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        struct os_mbuf* om = event->notify_rx.om;
        if (om != nullptr) {
            uint16_t len = OS_MBUF_PKTLEN(om);
            std::string json(len, '\0');
            if (os_mbuf_copydata(om, 0, len, &json[0]) == 0) {
                self->HandleNotify(json);
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu updated: %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

void BleCentral::AttemptConnect(const ble_addr_t& addr)
{
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    ble_hs_id_infer_auto(0, &own_addr_type);

    int rc = ble_gap_connect(own_addr_type, &addr, kConnectTimeoutMs,
                             nullptr, &BleCentral::GapEventCb, this);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
        xEventGroupSetBits(events_, kNodeLost);
    }
}

// ================ 服务发现 ================

void BleCentral::DiscoverService(uint16_t conn_handle)
{
    svc_start_ = svc_end_ = 0;
    h_write_ = h_notify_ = 0;

    int rc = ble_gattc_disc_all_svcs(conn_handle, &BleCentral::SvcDiscCb, this);
    if (rc != 0) {
        ESP_LOGE(TAG, "disc_all_svcs failed: %d", rc);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

int BleCentral::SvcDiscCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                          const struct ble_gatt_svc* service, void* arg)
{
    BleCentral* self = static_cast<BleCentral*>(arg);

    if (error->status == 0 && service != nullptr) {
        // 命中 esp32-node 专属服务
        if (ble_uuid_cmp(&service->uuid.u, &kSvcUuid.u) == 0) {
            self->svc_start_ = service->start_handle;
            self->svc_end_   = service->end_handle;
            ESP_LOGI(TAG, "service found: handles [%d..%d]",
                     self->svc_start_, self->svc_end_);
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        if (self->svc_start_ == 0) {
            ESP_LOGW(TAG, "esp32-node service not found, disconnect");
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        // 继续发现该服务下的特征值
        int rc = ble_gattc_disc_all_chrs(conn_handle, self->svc_start_,
                                         self->svc_end_,
                                         &BleCentral::ChrDiscCb, self);
        if (rc != 0) {
            ESP_LOGE(TAG, "disc_all_chrs failed: %d", rc);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }

    ESP_LOGW(TAG, "svc discovery error: %d", error->status);
    return 0;
}

int BleCentral::ChrDiscCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                          const struct ble_gatt_chr* chr, void* arg)
{
    BleCentral* self = static_cast<BleCentral*>(arg);

    if (error->status == 0 && chr != nullptr) {
        if (ble_uuid_cmp(&chr->uuid.u, &kChrWriteUuid.u) == 0) {
            self->h_write_ = chr->val_handle;
            ESP_LOGI(TAG, "write chr ff01: handle=%d", self->h_write_);
        } else if (ble_uuid_cmp(&chr->uuid.u, &kChrNotifyUuid.u) == 0) {
            self->h_notify_ = chr->val_handle;
            ESP_LOGI(TAG, "notify chr ff02: handle=%d", self->h_notify_);
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        if (self->h_write_ == 0 || self->h_notify_ == 0) {
            ESP_LOGW(TAG, "required characteristics missing (w=%d n=%d), disconnect",
                     self->h_write_, self->h_notify_);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        self->SubscribeAndHandshake(conn_handle);
        return 0;
    }

    ESP_LOGW(TAG, "chr discovery error: %d", error->status);
    return 0;
}

// ================ 订阅 + 握手 ================

void BleCentral::SubscribeAndHandshake(uint16_t conn_handle)
{
    // 1. 写 CCC 描述符订阅 NOTIFY（CCC 通常紧跟在特征值句柄之后）
    uint16_t ccc_val = 0x0001;  // 开启 notify
    int rc = ble_gattc_write_flat(conn_handle, static_cast<uint16_t>(h_notify_ + 1),
                                  &ccc_val, sizeof(ccc_val), nullptr, nullptr);
    if (rc != 0) {
        ESP_LOGW(TAG, "write CCC failed: %d", rc);
    }

    // 2. 发送握手请求 hello
    char hello[96];
    const char* relay_id = registry_ ? registry_->RelayId() : "hub-unknown";
    std::snprintf(hello, sizeof(hello),
                  "{\"act\":\"hello\",\"relay_id\":\"%s\",\"ver\":1}", relay_id);

    rc = ble_gattc_write_flat(conn_handle, h_write_,
                              hello, static_cast<uint16_t>(std::strlen(hello)),
                              nullptr, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "write hello failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "handshake hello sent: %s", hello);
}

// ================ Notify 解析 ================

void BleCentral::HandleNotify(const std::string& json)
{
    ESP_LOGI(TAG, "notify rx: %s", json.c_str());

    auto contains = [&](const char* key) {
        return json.find(key) != std::string::npos;
    };

    // 第 2 步：节点上报自身信息与能力清单
    if (contains("\"hello_ack\"")) {
        auto pos = json.find("\"node_id\":\"");
        if (pos == std::string::npos) {
            ESP_LOGW(TAG, "hello_ack without node_id");
            return;
        }
        pos += std::strlen("\"node_id\":\"");
        auto end = json.find("\"", pos);
        if (end == std::string::npos) return;
        std::string node_id = json.substr(pos, end - pos);

        if (registry_ != nullptr) {
            registry_->RegisterNode(node_id, json);
        }
        node_id_ = node_id;
        paired_ = true;

        // 第 3 步：回复接受配对（携带上报间隔）
        char accept[64];
        std::snprintf(accept, sizeof(accept),
                      "{\"act\":\"accept\",\"interval_ms\":5000}");
        if (conn_handle_ != BLE_HS_CONN_HANDLE_NONE && h_write_ != 0) {
            ble_gattc_write_flat(conn_handle_, h_write_,
                                 accept, static_cast<uint16_t>(std::strlen(accept)),
                                 nullptr, nullptr);
        }

        // 通知 mqtt_reporter 上报 register 主题
        if (pipeline_ != nullptr) {
            pipeline_->SubmitRegistration(node_id, json);
        }
        ESP_LOGI(TAG, "node paired: %s", node_id.c_str());
        return;
    }

    // 第 4 步：节点确认数据流开始
    if (contains("\"ack_done\"")) {
        ESP_LOGI(TAG, "handshake ack_done, data flow begins");
        return;
    }

    // 常规传感器数据
    if (registry_ != nullptr && !node_id_.empty()) {
        registry_->UpdateDataTime(node_id_);
    }
    if (pipeline_ != nullptr) {
        pipeline_->Submit(conn_handle_, h_notify_, json, node_id_);
    }
}

} // namespace esp32hub
