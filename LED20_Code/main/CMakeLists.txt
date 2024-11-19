idf_component_register(SRCS "main.c" "defines.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    REQUIRES esp_driver_spi)