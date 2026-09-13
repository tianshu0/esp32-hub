// node_registry 组件：已配对节点表（持久化 + 内存索引）
//
// 设计要点：
//   - 内存中维护最多 kMaxNodes 个 NodeInfo（hub 同时最多连 4 个节点，由 BLE 配置限制）
//   - 持久化：SPIFFS 存储 /storage/registry.json（与 broker 一致用文件而非 NVS blob，
//     因为字段不固定，避免 NVS 单条 ≤4KB/31 字符键的限制）
//   - RelayId 也在这里：首次启动从 AppConfig 取，没有则基于 MAC 生成并保存到 NVS
//   - 不直接订阅事件：ble_central 主动调 RegisterNode/UpdateData
//   - display_service 主动调 GetByIndex 拉取（不做事件推送，避免循环依赖）
#pragma once

#include <cstdint>
#include <string>
#include "esp_err.h"
#include "esp_timer.h"

namespace esp32hub {

class AppConfig;

class NodeRegistry {
public:
    // 单个已配对节点信息
    struct NodeInfo {
        std::string node_id;          // 节点 ID（握手时由 node 上报）
        std::string sensor_summary;   // 传感器类型摘要（如 "temp_hum,pressure"）
        std::string capability;       // 能力清单原文（hello_ack JSON，供 MQTT 重连后补发 register）
        int64_t last_seen_us = 0;     // 最后一次收到数据的时间
        bool online = false;          // 是否当前在线（BLE 连接中）
    };

    NodeRegistry() = default;
    ~NodeRegistry();

    // 初始化：加载 SPIFFS 持久化文件 + 生成 RelayId（若 AppConfig 中没有）
    esp_err_t Init(AppConfig* config);

    // 注册/更新节点（握手成功时由 ble_central 调用）
    // capability_json 是握手时 node 上报的能力清单原文（{"types":[...]}）
    esp_err_t RegisterNode(const std::string& node_id, const std::string& capability_json);

    // 更新节点最新数据时间（ble_central 收到数据时调用）
    void UpdateDataTime(const std::string& node_id);

    // 标记节点离线（BLE 断开时调用）
    void MarkOffline(const std::string& node_id);

    // 中继 ID（持久化生成的，供 ble_central 握手与 mqtt_reporter 主题使用）
    const char* RelayId() const { return relay_id_.c_str(); }

    // 节点数量
    int Count() const { return count_; }

    // 按索引获取（供 display_service 渲染列表页）
    // 越界返回空 NodeInfo
    NodeInfo GetByIndex(int index) const;

private:
    static constexpr int kMaxNodes = 4;
    NodeInfo nodes_[kMaxNodes];
    int count_ = 0;
    std::string relay_id_;
    AppConfig* config_ = nullptr;

    esp_err_t LoadFromFile();
    esp_err_t SaveToFile() const;
    std::string StoragePath() const;
    void EnsureRelayId();
};

} // namespace esp32hub
