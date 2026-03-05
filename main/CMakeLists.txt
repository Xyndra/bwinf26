idf_component_register(SRCS "main.cpp" "sensor.cpp" "map_visualiser.cpp"
    INCLUDE_DIRS "." REQUIRES Watchy PRIV_REQUIRES nvs_flash)
idf_build_set_property(COMPILE_OPTIONS "-DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG" APPEND)
