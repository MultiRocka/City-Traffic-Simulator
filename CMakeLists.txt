cmake_minimum_required(VERSION 3.16)

project(EasyRider VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(easy_rider_core
    src/core/Simulation.cpp
)

target_include_directories(easy_rider_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

add_executable(easy_rider_app
    src/app/main.cpp
)

target_link_libraries(easy_rider_app
    PRIVATE
        easy_rider_core
)
