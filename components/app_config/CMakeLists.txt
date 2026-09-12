# app_config：NVS 配置存储组件
idf_component_register(
    SRCS
        "AppConfig.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        nvs_flash
        esp_common
        log
)
