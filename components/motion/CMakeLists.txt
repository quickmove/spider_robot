idf_component_register(
    SRCS "src/motion.c"
    INCLUDE_DIRS "include"
    REQUIRES
        gait
        posture
        servo
        kinematics
        log
        esp_timer
)
