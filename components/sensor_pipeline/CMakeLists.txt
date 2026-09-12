# sensor_pipeline 组件：GATT 数据解析 + 事件投递
idf_component_register(
    SRCS
        "SensorPipeline.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_event
        json
        log
)
