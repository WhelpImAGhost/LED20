idf_component_register(SRCS "defines.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    REQUIRES esp_driver_spi)