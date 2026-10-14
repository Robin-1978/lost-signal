cmake_minimum_required(VERSION 3.25)

project(lost_signal VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LOST_SIGNAL_BUILD_CLIENT "Build the raylib + TGUI desktop client" ON)
option(LOST_SIGNAL_BUILD_TESTS "Build headless tests" ON)

include(FetchContent)

FetchContent_Declare(
 nlohmann_json
 URL https://codeload.github.com/nlohmann/json/tar.gz/refs/tags/v3.11.3
 URL_HASH SHA256=0d8ef5af7f9794e3263480193c491549b2ba6cc74bb018906202ada498a79406
 DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

add_library(lost_signal_sim STATIC
 src/hw/chassis/ackermann.cpp
 src/algo/astar.cpp
 src/hw/chassis/diff_drive.cpp
 src/algo/dynamic_obstacle_map.cpp
 src/algo/dwa.cpp
 src/algo/grid_map.cpp
  src/algo/predictor.cpp
  src/hw/chassis/motor_drive.cpp
  src/hw/chassis/tracked.cpp
 src/hw/dynamics/contact.cpp
 src/algo/state_estimator.cpp
 src/algo/pocket_shadow.cpp
 src/algo/estimator_replay.cpp
src/hw/feedforward.cpp
 src/algo/pid.cpp
 src/hw/sensors/sensor.cpp
)
target_include_directories(lost_signal_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(lost_signal_sim PUBLIC cxx_std_20)

add_library(lost_signal_game STATIC
 src/game/campaign_progress.cpp
 src/game/content_catalog.cpp
 src/game/map_serialize.cpp
 src/game/mission_simulation.cpp
)
target_include_directories(lost_signal_game PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(lost_signal_game PUBLIC lost_signal_sim nlohmann_json::nlohmann_json)

add_executable(lost_signal_headless src/game/headless_main.cpp)
target_link_libraries(lost_signal_headless PRIVATE lost_signal_game)

if(LOST_SIGNAL_BUILD_CLIENT)
 set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
 set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
 set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
 FetchContent_Declare(
 raylib
 URL https://codeload.github.com/raysan5/raylib/tar.gz/refs/tags/6.0
 URL_HASH SHA256=2b3ee1e2120c7a0796b33062c7e9a694dd8a8caa56a96319ac8c8ecf54a90d0b
 DOWNLOAD_EXTRACT_TIMESTAMP TRUE
 )
 FetchContent_MakeAvailable(raylib)

 set(TGUI_BACKEND RAYLIB CACHE STRING "" FORCE)
 set(TGUI_SHARED_LIBS OFF CACHE BOOL "" FORCE)
 set(TGUI_BUILD_GUI_BUILDER OFF CACHE BOOL "" FORCE)
 set(TGUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
 set(TGUI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
 set(TGUI_BUILD_FRAMEWORK OFF CACHE BOOL "" FORCE)
 FetchContent_Declare(
 tgui
 URL https://codeload.github.com/texus/TGUI/tar.gz/refs/tags/v1.13.0
 URL_HASH SHA256=6e6000b5b130d6ddf73d593ff62cdd6f5c2045a1f8ffacb10262aedcb7ea7465
 DOWNLOAD_EXTRACT_TIMESTAMP TRUE
 PATCH_COMMAND
 ${CMAKE_COMMAND}
 -DSOURCE_FILE=<SOURCE_DIR>/src/Backend/Font/Raylib/BackendFontRaylib.cpp
 -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/patch_tgui_raylib.cmake
 )
 FetchContent_MakeAvailable(tgui)

 add_executable(lost_signal
 src/client/main.cpp
 src/client/game_ui.cpp
 src/client/map_editor.cpp
 src/client/scene_renderer_3d.cpp
 src/client/world_renderer.cpp
 )
 target_link_libraries(lost_signal PRIVATE lost_signal_game raylib TGUI::TGUI)
 target_compile_definitions(lost_signal PRIVATE
 LOST_SIGNAL_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_custom_command(TARGET lost_signal POST_BUILD
 COMMAND ${CMAKE_COMMAND} -E copy_directory
 ${CMAKE_CURRENT_SOURCE_DIR}/assets $<TARGET_FILE_DIR:lost_signal>/assets
 COMMENT "Copying data-driven game content"
 )
endif()

if(LOST_SIGNAL_BUILD_TESTS)
 enable_testing()

 add_executable(test_content_catalog tests/new/test_content_catalog.cpp)
 target_link_libraries(test_content_catalog PRIVATE lost_signal_game)
 target_compile_definitions(test_content_catalog PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME content_catalog COMMAND test_content_catalog)

 add_executable(test_mission_simulation tests/new/test_mission_simulation.cpp)
 target_link_libraries(test_mission_simulation PRIVATE lost_signal_game)
 target_compile_definitions(test_mission_simulation PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME mission_simulation COMMAND test_mission_simulation)

 add_executable(test_dynamic_perception tests/new/test_dynamic_perception.cpp)
 target_link_libraries(test_dynamic_perception PRIVATE lost_signal_sim)
 add_test(NAME dynamic_perception COMMAND test_dynamic_perception)

 add_executable(test_model_validation tests/new/test_model_validation.cpp)
 target_link_libraries(test_model_validation PRIVATE lost_signal_game)
 target_compile_definitions(test_model_validation PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME model_validation COMMAND test_model_validation)

 add_executable(test_chassis tests/new/test_chassis.cpp)
 target_link_libraries(test_chassis PRIVATE lost_signal_sim)
 add_test(NAME chassis COMMAND test_chassis)

 add_executable(test_map_editor tests/new/test_map_editor.cpp src/client/map_editor.cpp)
 target_link_libraries(test_map_editor PRIVATE lost_signal_game)
 target_compile_definitions(test_map_editor PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 target_include_directories(test_map_editor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
 add_test(NAME map_editor COMMAND test_map_editor)

 add_executable(test_map_roundtrip tests/new/test_map_roundtrip.cpp)
 target_link_libraries(test_map_roundtrip PRIVATE lost_signal_game)
 target_compile_definitions(test_map_roundtrip PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME map_roundtrip COMMAND test_map_roundtrip)

 add_executable(test_predictor_parity tests/new/test_predictor_parity.cpp)
 target_link_libraries(test_predictor_parity PRIVATE lost_signal_sim)
 add_test(NAME predictor_parity COMMAND test_predictor_parity)

 add_executable(test_slip_detect tests/new/test_slip_detect.cpp)
 target_link_libraries(test_slip_detect PRIVATE lost_signal_sim)
 add_test(NAME slip_detect COMMAND test_slip_detect)

 add_executable(test_slip_shadow tests/new/test_slip_shadow.cpp)
 target_link_libraries(test_slip_shadow PRIVATE lost_signal_sim lost_signal_game)
 target_compile_definitions(test_slip_shadow PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME slip_shadow COMMAND test_slip_shadow)

 add_executable(test_departure_replay tests/new/test_departure_replay.cpp)
 target_link_libraries(test_departure_replay PRIVATE lost_signal_sim)
 add_test(NAME departure_replay COMMAND test_departure_replay)

 add_executable(test_phase_legs tests/new/test_phase_legs.cpp)
 target_link_libraries(test_phase_legs PRIVATE lost_signal_sim)
 add_test(NAME phase_legs COMMAND test_phase_legs)

 add_executable(test_pocket_search tests/new/test_pocket_search.cpp)
 target_link_libraries(test_pocket_search PRIVATE lost_signal_sim)
 add_test(NAME pocket_search COMMAND test_pocket_search)

 add_executable(test_perturbation tests/new/test_perturbation.cpp)
 target_link_libraries(test_perturbation PRIVATE lost_signal_sim)
 add_test(NAME perturbation COMMAND test_perturbation)

 add_executable(test_estimation_error tests/new/test_estimation_error.cpp)
 target_link_libraries(test_estimation_error PRIVATE lost_signal_game)
 target_compile_definitions(test_estimation_error PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME estimation_error COMMAND test_estimation_error)

 add_executable(test_release_error tests/new/test_release_error.cpp)
 target_link_libraries(test_release_error PRIVATE lost_signal_game)
 target_compile_definitions(test_release_error PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME release_error COMMAND test_release_error)

 add_executable(test_signal_quality tests/new/test_signal_quality.cpp)
 target_link_libraries(test_signal_quality PRIVATE lost_signal_sim lost_signal_game)
 target_compile_definitions(test_signal_quality PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME signal_quality COMMAND test_signal_quality)

 add_executable(test_inverse_consistency tests/new/test_inverse_consistency.cpp)
 target_link_libraries(test_inverse_consistency PRIVATE lost_signal_sim)
 add_test(NAME inverse_consistency COMMAND test_inverse_consistency)

 add_executable(test_feedforward tests/new/test_feedforward.cpp)
 target_link_libraries(test_feedforward PRIVATE lost_signal_sim)
 add_test(NAME feedforward COMMAND test_feedforward)

 add_executable(test_state_estimator tests/new/test_state_estimator.cpp)
 target_link_libraries(test_state_estimator PRIVATE lost_signal_sim)
 add_test(NAME state_estimator COMMAND test_state_estimator)

 add_executable(test_ro2_pocket_shadow tests/new/test_ro2_pocket_shadow.cpp)
 target_link_libraries(test_ro2_pocket_shadow PRIVATE lost_signal_sim)
 add_test(NAME ro2_pocket_shadow COMMAND test_ro2_pocket_shadow)

 add_executable(test_estimator_replay tests/new/test_estimator_replay.cpp)
 target_link_libraries(test_estimator_replay PRIVATE lost_signal_sim)
 add_test(NAME estimator_replay COMMAND test_estimator_replay)

 add_executable(test_ff_sensitivity tests/new/test_ff_sensitivity.cpp)
 target_link_libraries(test_ff_sensitivity PRIVATE lost_signal_sim lost_signal_game)
 target_compile_definitions(test_ff_sensitivity PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME ff_sensitivity COMMAND test_ff_sensitivity)

 find_package(Python3 COMPONENTS Interpreter QUIET)
 if(Python3_Interpreter_FOUND)
 execute_process(
 COMMAND ${Python3_EXECUTABLE} -c "import jsonschema"
 RESULT_VARIABLE LOST_SIGNAL_HAS_JSONSCHEMA
 OUTPUT_QUIET ERROR_QUIET
 )
 if(LOST_SIGNAL_HAS_JSONSCHEMA EQUAL 0)
 add_test(
 NAME content_schema
 COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_content.py -q
 )
 set_tests_properties(content_schema PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
 endif()
 add_test(
 NAME playtest_summary
 COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_playtest.py -q
 )
 set_tests_properties(playtest_summary PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
 add_test(
 NAME content_roundtrip
 COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_content_roundtrip.py -q
 )
 set_tests_properties(content_roundtrip PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
 endif()
endif()
