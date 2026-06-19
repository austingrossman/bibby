include(FetchContent)

set(SDL_SHARED             OFF CACHE BOOL "" FORCE)
set(SDL_STATIC             ON  CACHE BOOL "" FORCE)
set(SDL_UNIX_CONSOLE_BUILD ON  CACHE BOOL "" FORCE)
set(SDL_LIBINPUT           ON  CACHE BOOL "" FORCE)

FetchContent_Declare(
    SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG        release-3.2.14
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(SDL3)

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

add_executable(frontend
    main.cpp
    display.cpp
    temp_filter.cpp
    pid.cpp
    csv_logger.cpp
    ${CMAKE_SOURCE_DIR}/shared/config.c
    sensors/max31865.cpp
    gui/kettle.cpp
    gui/temp_history.cpp
    ${IMGUI_SOURCES}
)

target_include_directories(frontend PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

# GLES3 on Pi — no desktop OpenGL needed.
# FONT_DIR lets the frontend load the vendored TTFs regardless of the runtime cwd.
#
# BIBBY_PATH is the absolute build-tree path of the sibling binary so the
# frontend always launches the freshly built controller during development.
#
# TODO(install): this path is NOT relocatable — after `make install` it still
# points into the build tree, which may not exist on the target. Before adding
# any install() rule, switch this to runtime resolution (resolve /proc/self/exe,
# look for the sibling, fall back to a PATH search) or to
# ${CMAKE_INSTALL_FULL_BINDIR}. See memory: install-relocatable-heater-path.
# BIBBY_CONFIG is the default ini location, overridable at runtime via the
# $BIBBY_CONFIG environment variable (see shared/config.c). Like the paths above
# it is baked to the build/source tree and not yet relocatable for install.
target_compile_definitions(frontend PRIVATE
    IMGUI_IMPL_OPENGL_ES3
    FONT_DIR="${IMGUI_DIR}/misc/fonts"
    BIBBY_PATH="$<TARGET_FILE:heater-controller>"
    BIBBY_CONFIG="${CMAKE_SOURCE_DIR}/bibby.ini"
)

find_library(GLESV2_LIB GLESv2 REQUIRED)
find_library(GPIOD_LIB gpiod REQUIRED)
target_link_libraries(frontend PRIVATE
    SDL3::SDL3-static
    ${GLESV2_LIB}
    ${GPIOD_LIB}
    rt
)
