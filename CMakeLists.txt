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
 src/algo/behavior_registry.cpp
 src/algo/entry_shadow.cpp
 src/algo/entry_governor.cpp
 src/hw/chassis/diff_drive.cpp
 src/algo/dynamic_obstacle_map.cpp
 src/algo/dwa.cpp
 src/algo/grid_map.cpp
  src/algo/predictor.cpp
  src/hw/chassis/motor_drive.cpp
 src/hw/chassis/chassis_factory.cpp
 src/hw/chassis/chassis_factory.cpp
  src/hw/chassis/tracked.cpp
 src/hw/dynamics/contact.cpp
 src/algo/state_estimator.cpp
 src/algo/pocket_shadow.cpp
 src/algo/estimator_replay.cpp
 src/algo/slip_observer.cpp
 src/algo/estimator_filter.cpp
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

 add_executable(test_dwell_behavior tests/new/test_dwell_behavior.cpp)
 target_link_libraries(test_dwell_behavior PRIVATE lost_signal_sim)
 add_test(NAME dwell_behavior COMMAND test_dwell_behavior)

 add_executable(test_behavior_registry tests/new/test_behavior_registry.cpp)
 target_link_libraries(test_behavior_registry PRIVATE lost_signal_sim)
 add_test(NAME behavior_registry COMMAND test_behavior_registry)

 add_executable(test_entry_shadow tests/new/test_entry_shadow.cpp)
 target_link_libraries(test_entry_shadow PRIVATE lost_signal_sim)
 add_test(NAME entry_shadow COMMAND test_entry_shadow)

 add_executable(test_entry_shadow_field tests/new/test_entry_shadow_field.cpp)
 target_link_libraries(test_entry_shadow_field PRIVATE lost_signal_game)
 target_include_directories(test_entry_shadow_field PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
 target_compile_definitions(test_entry_shadow_field PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME entry_shadow_field COMMAND test_entry_shadow_field)

 add_executable(test_stop_line_study tests/new/test_stop_line_study.cpp)
 target_link_libraries(test_stop_line_study PRIVATE lost_signal_game)
 target_compile_definitions(test_stop_line_study PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME stop_line_study COMMAND test_stop_line_study)

 add_executable(test_stop_whatif tests/new/test_stop_whatif.cpp)
 target_link_libraries(test_stop_whatif PRIVATE lost_signal_game)
 target_compile_definitions(test_stop_whatif PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 target_include_directories(test_stop_whatif PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
 add_test(NAME stop_whatif COMMAND test_stop_whatif)

 add_executable(test_entry_shadow_town tests/new/test_entry_shadow_town.cpp)
 target_link_libraries(test_entry_shadow_town PRIVATE lost_signal_game)
 target_compile_definitions(test_entry_shadow_town PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 target_include_directories(test_entry_shadow_town PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
 add_test(NAME entry_shadow_town COMMAND test_entry_shadow_town)

 add_executable(test_manual_ram tests/new/test_manual_ram.cpp)
 target_link_libraries(test_manual_ram PRIVATE lost_signal_game)
 target_compile_definitions(test_manual_ram PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 target_include_directories(test_manual_ram PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
 add_test(NAME manual_ram COMMAND test_manual_ram)

 add_executable(test_entry_governor tests/new/test_entry_governor.cpp)
 target_link_libraries(test_entry_governor PRIVATE lost_signal_sim)
 add_test(NAME entry_governor COMMAND test_entry_governor)

 add_executable(test_summon_recharge tests/new/test_summon_recharge.cpp)
 target_link_libraries(test_summon_recharge PRIVATE lost_signal_game)
 target_compile_definitions(test_summon_recharge PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME summon_recharge COMMAND test_summon_recharge)


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

 add_executable(test_player_pack tests/new/test_player_pack.cpp)
 target_link_libraries(test_player_pack PRIVATE lost_signal_game)
 target_compile_definitions(test_player_pack PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME player_pack COMMAND test_player_pack)

 add_executable(test_chassis_choice tests/new/test_chassis_choice.cpp)
 target_link_libraries(test_chassis_choice PRIVATE lost_signal_game)
 target_compile_definitions(test_chassis_choice PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME chassis_choice COMMAND test_chassis_choice)

 add_executable(test_abilities tests/new/test_abilities.cpp)
 target_link_libraries(test_abilities PRIVATE lost_signal_game)
 target_compile_definitions(test_abilities PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME abilities COMMAND test_abilities)

 add_executable(test_contract_rep tests/new/test_contract_rep.cpp)
 target_link_libraries(test_contract_rep PRIVATE lost_signal_game)
 target_compile_definitions(test_contract_rep PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME contract_rep COMMAND test_contract_rep)

 add_executable(test_signal_cross tests/new/test_signal_cross.cpp)
 target_link_libraries(test_signal_cross PRIVATE lost_signal_game)
 target_compile_definitions(test_signal_cross PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME signal_cross COMMAND test_signal_cross)

 add_executable(test_shadow_accumulation tests/new/test_shadow_accumulation.cpp)
 target_link_libraries(test_shadow_accumulation PRIVATE lost_signal_game)
 target_compile_definitions(test_shadow_accumulation PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME shadow_accumulation COMMAND test_shadow_accumulation)

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
 add_executable(test_slip_observer tests/new/test_slip_observer.cpp)
 target_link_libraries(test_slip_observer PRIVATE lost_signal_sim)
 add_test(NAME slip_observer COMMAND test_slip_observer)
 add_executable(test_estimator_filter tests/new/test_estimator_filter.cpp)
 target_link_libraries(test_estimator_filter PRIVATE lost_signal_sim)
 add_test(NAME estimator_filter COMMAND test_estimator_filter)
 add_executable(test_estimator_chain tests/new/test_estimator_chain.cpp)
 target_link_libraries(test_estimator_chain PRIVATE lost_signal_sim)
 add_test(NAME estimator_chain COMMAND test_estimator_chain)

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
