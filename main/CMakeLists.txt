idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        log
        nvs_flash
        kinematics
        pca9685
        servo
        gait
        posture
        motion
        cli
        web_ctrl
)