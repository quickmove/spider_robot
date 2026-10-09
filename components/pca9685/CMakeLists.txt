idf_component_register(
    SRCS "src/pca9685.c" "src/pca9685_math.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_gpio
)
