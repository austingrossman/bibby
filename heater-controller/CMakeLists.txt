add_executable(heater-controller main.c)

target_include_directories(heater-controller PRIVATE ${CMAKE_SOURCE_DIR})

find_library(GPIOD_LIB gpiod REQUIRED)
target_link_libraries(heater-controller PRIVATE ${GPIOD_LIB} rt)
