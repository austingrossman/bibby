cmake_minimum_required(VERSION 3.16)
project(bibby C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# Default to an optimized build with symbols. Without this CMake configures an
# empty build type — no -O at all — which costs real time in the hot loops (the
# web mirror's pixel conversion runs ~4x slower, and so does LVGL's renderer).
# Override with -DCMAKE_BUILD_TYPE=Debug when stepping through code.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

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
find_package(ZLIB REQUIRED)          # PNG compression for the web screen mirror

# The web UI is a single HTML file, compiled into the binary so an installed
# bibby has no companion asset directory to lose.
set(WEB_ASSET_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/web/www/index.html)
set(WEB_ASSET_C   ${CMAKE_CURRENT_BINARY_DIR}/web_assets_generated.c)
add_custom_command(
    OUTPUT  ${WEB_ASSET_C}
    COMMAND ${CMAKE_COMMAND}
            -DIN=${WEB_ASSET_SRC} -DOUT=${WEB_ASSET_C} -DNAME=web_index_html
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_asset.cmake
    DEPENDS ${WEB_ASSET_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_asset.cmake
    COMMENT "Embedding web UI (src/web/www/index.html)"
    VERBATIM)

add_executable(bibby
    src/main.c
    src/config.c
    src/control.c
    src/csv_logger.c
    src/max31865.c
    src/mass_estimator.c
    src/power_split.c
    src/sampler_thread.c
    src/ssr_thread.c
    src/state.c
    src/temp_filter.c
    src/ui/ui.c
    src/ui/ui_kettle.c
    src/ui/ui_screen.c
    src/web/http.c
    src/web/web.c
    src/web/web_logs.c
    src/web/web_screen.c
    ${WEB_ASSET_C}
)
target_include_directories(bibby PRIVATE src src/web)
target_link_libraries(bibby PRIVATE lvgl ${GPIOD_LIB} ZLIB::ZLIB m pthread rt)

# ── Unit tests (pure modules, no hardware) ────────────────────────────────────
add_executable(tests
    tests/test_units.c
    src/config.c
    src/control.c
    src/mass_estimator.c
    src/power_split.c
    src/state.c
    src/temp_filter.c
)
target_include_directories(tests PRIVATE src)
target_link_libraries(tests PRIVATE m pthread)
