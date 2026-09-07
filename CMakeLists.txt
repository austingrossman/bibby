cmake_minimum_required(VERSION 3.16)
project(bibby C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# ── LVGL (UI toolkit) ─────────────────────────────────────────────────────────
include(FetchContent)
set(LV_BUILD_CONF_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/ui/lv_conf.h CACHE STRING "" FORCE)
FetchContent_Declare(
    lvgl
    GIT_REPOSITORY https://github.com/lvgl/lvgl.git
    GIT_TAG        v9.3.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(lvgl)

# ── bibby — the controller ────────────────────────────────────────────────────
find_library(GPIOD_LIB gpiod REQUIRED)

add_executable(bibby
    src/main.c
    src/config.c
    src/control.c
    src/csv_logger.c
    src/max31865.c
    src/mc_estimator.c
    src/power_split.c
    src/sampler_thread.c
    src/ssr_thread.c
    src/state.c
    src/temp_filter.c
    src/ui/ui.c
    src/ui/ui_kettle.c
    src/ui/ui_screen.c
)
target_include_directories(bibby PRIVATE src)
target_link_libraries(bibby PRIVATE lvgl ${GPIOD_LIB} m pthread rt)

# ── Unit tests (pure modules, no hardware) ────────────────────────────────────
add_executable(tests
    tests/test_units.c
    src/config.c
    src/control.c
    src/mc_estimator.c
    src/power_split.c
    src/state.c
    src/temp_filter.c
)
target_include_directories(tests PRIVATE src)
target_link_libraries(tests PRIVATE m pthread)
