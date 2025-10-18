idf_component_register(
    SRCS ultrasonic.c
    INCLUDE_DIRS .
    REQUIRES driver freertos esp_timer
)