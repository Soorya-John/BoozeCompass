idf_component_register(SRCS "motor_math.c" "motor.c" "uln2003_backend.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver freertos esp_timer navigation)
