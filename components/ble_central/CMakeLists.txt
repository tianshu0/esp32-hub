# ble_central 组件：BLE Central 扫描/连接/握手/Notify 接收
idf_component_register(
    SRCS
        "BleCentral.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        app_config
        node_registry
        sensor_pipeline
        esp_event
        esp_wifi
        esp_netif
        bt
        log
)
