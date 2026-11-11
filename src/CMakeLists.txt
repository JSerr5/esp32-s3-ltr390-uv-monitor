idf_component_register(
    SRCS "main.cpp"
    REQUIRES driver esp_timer esp_wifi esp_event esp_netif nvs_flash esp_http_server log
)
