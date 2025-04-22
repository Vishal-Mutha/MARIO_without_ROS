idf_component_register(SRCS "web_socket_control.c"
                       PRIV_REQUIRES driver esp_wifi nvs_flash esp_http_server
                       INCLUDE_DIRS ".")
