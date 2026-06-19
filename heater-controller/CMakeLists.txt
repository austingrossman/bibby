add_executable(heater-controller
    main.c
    ${CMAKE_SOURCE_DIR}/shared/config.c
)

target_include_directories(heater-controller PRIVATE ${CMAKE_SOURCE_DIR})

# BIBBY_CONFIG is the default ini location (overridable at runtime via the
# $BIBBY_CONFIG environment variable). Baked to the source tree for development;
# point it at the installed path when packaging.
target_compile_definitions(heater-controller PRIVATE
    BIBBY_CONFIG="${CMAKE_SOURCE_DIR}/bibby.ini")

find_library(GPIOD_LIB gpiod REQUIRED)
target_link_libraries(heater-controller PRIVATE ${GPIOD_LIB} rt)
