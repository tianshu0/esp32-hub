// sensor_pipeline 组件：GATT 数据解析 + 事件投递
//
// 数据流：
//   ble_central 收到 NOTIFY -> Submit(conn, handle, json)
//     -> cJSON 解析（type/ts/values 字段）
//     -> 查 node_registry 拿到 node_id（按 conn_handle 映射，由 ble_central 维护）
//     -> esp_event_post 投递 SENSOR_DATA_EVENT
//   mqtt_reporter 订阅 SENSOR_DATA_EVENT -> 按主题上报
//
// 设计要点：
//   - 单纯解析层：无状态机、无任务（Submit 在调用者上下文执行）
//   - 依赖 cJSON（已由 ESP-IDF 提供，不需额外依赖）
//   - 与 ble_central 解耦：ble_central 只传 JSON 原文，pipeline 自己解析
//   - 与 mqtt_reporter 解耦：通过 esp_event 总线投递，mqtt_reporter 订阅
//   - 失败的 JSON 直接丢弃并打日志（生产环境可换为统计计数器）
#pragma once

#include <cstdint>
#include <string>
#include "esp_err.h"
#include "esp_event.h"

namespace esp32hub {

// 自定义事件基（与 ESP_EVENT_ANY_BASE 区分）
extern const char* kSensorDataEventBase;

class SensorPipeline {
public:
    SensorPipeline() = default;
    ~SensorPipeline();

    // 注册事件基；mqtt_reporter 在订阅前必须先调 Init
    esp_err_t Init();

    // 失败注销事件基
    void Deinit();

    // 事件 ID：区分不同数据类型
    enum EventId : int32_t {
        kEventSensorData = 1,   // 传感器数据（type/ts/values）
        kEventNodeOnline,       // 节点上线
        kEventNodeOffline,     // 节点离线
    };

    // 事件 payload（投递到 esp_event 时拷贝一份，避免生命周期问题）
    struct SensorData {
        uint16_t conn_handle;
        char node_id[32];       // 节点 ID（防止 esp_event_post 引用栈对象）
        char type[32];          // 传感器类型（如 "temp_hum"）
        int64_t ts;             // 时间戳（毫秒）
        char values_json[128];  // values 字段原文 JSON（保留原样供 mqtt_reporter 转发）
    };

    // 节点注册事件 payload（握手成功时由 ble_central 触发）
    struct NodeRegistered {
        char node_id[32];
        // 能力清单原文 hello_ack（含 types + 每传感器 format，实测约 290 字节；
        // 此前 192 会把完整 JSON 截断导致下游解析出空能力清单）
        char capability_json[384];
    };

    // 提交数据（ble_central 收到 NOTIFY 后调用）
    // json 是 NOTIFY 原文，如 {"type":"temp_hum","ts":1234567890,"values":{...}}
    // node_id 来自握手时 node 上报（ble_central 从 registry 查询）
    void Submit(uint16_t conn_handle, uint16_t attr_handle, const std::string& json,
                const std::string& node_id);

    // 提交节点注册（ble_central 握手成功后调用，mqtt_reporter 据此上报 register 主题）
    void SubmitRegistration(const std::string& node_id, const std::string& capability_json);

private:
    bool inited_ = false;
};

} // namespace esp32hub
