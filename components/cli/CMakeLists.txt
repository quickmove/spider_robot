idf_component_register(
    SRCS "src/cli.c"
    INCLUDE_DIRS "include"
    REQUIRES
        motion
        posture
        servo
        gait
        kinematics
        log
)
