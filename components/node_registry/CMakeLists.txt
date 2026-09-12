# node_registry 组件：已配对节点表（内存 + SPIFFS 持久化）
idf_component_register(
    SRCS
        "NodeRegistry.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        app_config
        spiffs
        json
        esp_timer
        esp_hw_support
        log
)
