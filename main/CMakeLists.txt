idf_component_register(SRCS "oneshot_read_main.c"
                    PRIV_REQUIRES esp_adc esp_driver_ledc esp_driver_gpio esp_driver_pcnt esp_driver_tsens esp_pm
                    INCLUDE_DIRS ".")