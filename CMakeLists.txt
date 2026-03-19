cmake_minimum_required(VERSION 3.16)

project(EasyRider VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(easy_rider_core
    src/core/Simulation.cpp
    src/core/Intersection.cpp
    src/core/RoadSegment.cpp
    src/core/City.cpp
    src/core/Vehicle.cpp
    src/core/VehicleGenerator.cpp
    src/core/TrafficLight.cpp
    src/core/StatisticsCollector.cpp
    src/core/EventLog.cpp
)

target_include_directories(easy_rider_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src/app
)

add_executable(easy_rider_app
    src/app/main.cpp
)

target_link_libraries(easy_rider_app
    PRIVATE
        easy_rider_core
)
