idf_component_register(SRCS "main.cpp" "DendoStepper.cpp" "http.c" "mqtt.c" "utils.c" "application.c"
                    INCLUDE_DIRS "." ${project_dir}/components/protocol_common/include
                    REQUIRES driver ultrasonic app_update bootloader_support mqtt esp_timer esp_https_ota esp_driver_gpio freertos nvs_flash esp_netif esp_wifi vfs console esp_eth esp_http_server esp_http_client esp_app_format esp_event
                   )
