idf_component_register(
    SRCS "src/servo.c" "src/servo_logic.c"
    INCLUDE_DIRS "include"
    REQUIRES
        pca9685
        kinematics
        nvs_flash
)
