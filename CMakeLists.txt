cmake_minimum_required(VERSION 3.16)

project(EasyRider VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(BUILD_SHARED_LIBS OFF)

# MSVC: decode UTF-8 source (no BOM) correctly for wide/u8 literals with Polish diacritics.
# /utf-8 sets both source and execution charset. Do not strip diacritics; keep u8path UTF-8 paths.
if(MSVC)
    add_compile_options(/utf-8)
    add_compile_definitions(UNICODE _UNICODE)
endif()

# nlohmann/json v3.11.3 — header-only, MIT License.
# Fetched once via FetchContent (see docs/OSM_IMPORT.md). Only JSON library in this repo.
include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(nlohmann_json)

add_library(easy_rider_core STATIC
    src/core/Simulation.cpp
    src/core/Scenario.cpp
    src/core/CollisionDetector.cpp
    src/core/Phase.cpp
    src/core/PhaseController.cpp
    src/core/PathFinder.cpp
    src/core/Intersection.cpp
    src/core/RoadSegment.cpp
    src/core/City.cpp
    src/core/Vehicle.cpp
    src/core/VehicleGenerator.cpp
    src/core/TrafficLight.cpp
    src/core/StatisticsCollector.cpp
    src/core/EventLog.cpp
    src/core/PedestrianCrossing.cpp
    src/core/Pedestrian.cpp
    src/core/PedestrianManager.cpp
    src/core/LaneBlockage.cpp
    src/core/CityBuilder.cpp
    src/core/IntersectionRules.cpp
    src/osm/OsmJsonParser.cpp
    src/osm/OsmToMapDescription.cpp
    src/osm/MapGeometryContinuity.cpp
    src/osm/OsmScenarioBuilder.cpp
    src/osm/GeoBbox.cpp
    src/osm/OverpassQuery.cpp
    src/osm/DownloadCache.cpp
    src/osm/FakeHttpTransport.cpp
    src/osm/MapDownloadService.cpp
)

# WinHTTP is Windows-only. Offline tests use FakeHttpTransport; GUI download uses WinHTTP.
if(WIN32)
    target_sources(easy_rider_core PRIVATE src/osm/WinHttpTransport.cpp)
endif()

target_include_directories(easy_rider_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(easy_rider_core
    PUBLIC
        nlohmann_json::nlohmann_json
)

if(WIN32)
    target_link_libraries(easy_rider_core PUBLIC winhttp)
endif()

add_library(easy_rider_ui STATIC
    src/ui/CityLayout.cpp
    src/ui/CityRenderer.cpp
    src/ui/VehicleRenderer.cpp
    src/ui/TrafficLightRenderer.cpp
    src/ui/PedestrianRenderer.cpp
    src/ui/LaneBlockageRenderer.cpp
    src/ui/RenderComposer.cpp
    src/ui/SimulationSession.cpp
    src/ui/ControlPanel.cpp
    src/ui/StatisticsPanel.cpp
    src/ui/EventLogPanel.cpp
)

target_include_directories(easy_rider_ui
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(easy_rider_ui
    PUBLIC
        easy_rider_core
)

# Native Win32 GDI GUI — Windows only (cloud Linux builds core/tests/console).
if(WIN32)
    add_library(easy_rider_gui STATIC
        src/gui/MapView.cpp
        src/gui/MainWindow.cpp
        src/gui/MapDownloadDialog.cpp
    )

    target_include_directories(easy_rider_gui
        PUBLIC
            ${PROJECT_SOURCE_DIR}/include
    )

    target_link_libraries(easy_rider_gui
        PUBLIC
            easy_rider_ui
    )

    add_executable(easy_rider_app WIN32
        src/app/main.cpp
    )

    target_include_directories(easy_rider_app
        PRIVATE
            ${PROJECT_SOURCE_DIR}/include
    )

    target_link_libraries(easy_rider_app
        PRIVATE
            easy_rider_gui
            user32
            gdi32
            comctl32
            comdlg32
    )
endif()

add_executable(easy_rider_console
    src/app/main_console.cpp
)

target_include_directories(easy_rider_console
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(easy_rider_console
    PRIVATE
        easy_rider_ui
)

enable_testing()

add_executable(easy_rider_motion_tests
    tests/motion_regression_tests.cpp
    tests/map_model_tests.cpp
    tests/map_camera_tests.cpp
    tests/intersection_priority_tests.cpp
    tests/pedestrian_tests.cpp
    tests/lane_blockage_tests.cpp
    tests/demo_integration_tests.cpp
    tests/osm_json_parser_tests.cpp
    tests/osm_to_map_tests.cpp
    tests/osm_scenario_tests.cpp
    tests/osm_larger_map_tests.cpp
    tests/osm_session_tests.cpp
    tests/map_download_tests.cpp
    tests/locator_selection_tests.cpp
    tests/geometry_control_tests.cpp
    tests/traffic_baseline_tests.cpp
    tests/etap3_traffic_tests.cpp
)

if(WIN32)
    target_sources(easy_rider_motion_tests PRIVATE tests/encoding_tests.cpp)
endif()

target_include_directories(easy_rider_motion_tests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(easy_rider_motion_tests
    PRIVATE
        easy_rider_ui
)

if(WIN32)
    target_link_libraries(easy_rider_motion_tests PRIVATE user32)
endif()

add_test(
    NAME motion_regression
    COMMAND easy_rider_motion_tests
)

set_tests_properties(motion_regression PROPERTIES
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
