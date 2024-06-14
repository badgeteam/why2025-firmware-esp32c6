file(GLOB BADGE_SRC_FILES src/*.c)

idf_component_register(
    SRCS
        ${BADGE_SRC_FILES}
    INCLUDE_DIRS
        src
    REQUIRES
        driver ledstrip
)
