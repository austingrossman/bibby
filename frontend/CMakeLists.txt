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
    sensors/max31865.cpp
    ${IMGUI_SOURCES}
)

target_include_directories(frontend PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

# GLES3 on Pi — no desktop OpenGL needed
target_compile_definitions(frontend PRIVATE IMGUI_IMPL_OPENGL_ES3)

find_library(GLESV2_LIB GLESv2 REQUIRED)
target_link_libraries(frontend PRIVATE
    SDL3::SDL3-static
    ${GLESV2_LIB}
    rt
)
