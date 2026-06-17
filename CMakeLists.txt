cmake_minimum_required(VERSION 3.16)
project(bibby C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(heater-controller)
add_subdirectory(frontend)
