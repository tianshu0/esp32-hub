# mqtt_reporter 组件：MQTT 客户端（上报中继状态与传感器数据）
idf_component_register(
    SRCS
        "MqttReporter.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        app_config
        node_registry
        sensor_pipeline
        mqtt
        esp_event
        esp_wifi
        esp_netif
        lwip
        log
)
