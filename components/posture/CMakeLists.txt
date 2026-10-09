idf_component_register(
    SRCS "src/posture.c" "src/keyframe.c" "src/kf_build.c" "src/kf_parse.c"
    INCLUDE_DIRS "include"
    REQUIRES
        kinematics
        log
        json
)