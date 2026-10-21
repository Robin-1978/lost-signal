cmake_minimum_required(VERSION 3.25)

project(lost_signal VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LOST_SIGNAL_BUILD_SIM "Build simulation, scenarios, and headless runner" ON)
option(LOST_SIGNAL_BUILD_CLIENT "Build the raylib + TGUI desktop client" ON)
option(LOST_SIGNAL_BUILD_TESTS "Build tests available for the selected product domains" ON)
option(LOST_SIGNAL_WARNINGS_AS_ERRORS "Treat first-party compiler warnings as errors" OFF)
option(LOST_SIGNAL_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

add_library(lost_signal_project_options INTERFACE)
if(MSVC)
 target_compile_options(lost_signal_project_options INTERFACE /W4 /permissive- /utf-8)
 if(LOST_SIGNAL_WARNINGS_AS_ERRORS)
 target_compile_options(lost_signal_project_options INTERFACE /WX)
 endif()
else()
 target_compile_options(lost_signal_project_options INTERFACE -Wall -Wextra -Wpedantic)
 if(LOST_SIGNAL_WARNINGS_AS_ERRORS)
 target_compile_options(lost_signal_project_options INTERFACE -Werror)
 endif()
 if(LOST_SIGNAL_ENABLE_SANITIZERS)
 target_compile_options(lost_signal_project_options INTERFACE
 -fsanitize=address,undefined -fno-omit-frame-pointer)
 target_link_options(lost_signal_project_options INTERFACE -fsanitize=address,undefined)
 endif()
endif()

include(FetchContent)
FetchContent_Declare(
 nlohmann_json
 URL https://codeload.github.com/nlohmann/json/tar.gz/refs/tags/v3.11.3
 URL_HASH SHA256=0d8ef5af7f9794e3263480193c491549b2ba6cc74bb018906202ada498a79406
 DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# ROBOT is the complete deployable stack. These targets must remain buildable
# after src/sim is removed and must never link a simulation target.
add_library(lost_signal_robot_core INTERFACE)
target_include_directories(lost_signal_robot_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(lost_signal_robot_core INTERFACE cxx_std_20)
target_link_libraries(lost_signal_robot_core INTERFACE lost_signal_project_options)

# The pose-graph backend uses Eigen's header-only sparse Cholesky solver. It
# remains an implementation dependency of ROBOT and is not exposed through
# LSRI, map, localization or planner interfaces.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(lost_signal_robot_platform STATIC
 src/robot/platform/ackermann_kinematics.cpp
 src/robot/platform/differential_kinematics.cpp
 src/robot/platform/holonomic_kinematics.cpp
 src/robot/platform/serial_manipulator.cpp
 src/robot/control/ackermann_actuator_controller.cpp
 src/robot/control/pid.cpp
 src/robot/control/feedforward.cpp
 src/robot/control/differential_wheel_controller.cpp
 src/robot/control/holonomic_actuator_controller.cpp
 src/robot/control/manipulator_joint_controller.cpp
)
target_link_libraries(lost_signal_robot_platform PUBLIC lost_signal_robot_core)

add_library(lost_signal_robot_autonomy STATIC
 src/robot/autonomy/planning/local/ackermann_local.cpp
 src/robot/autonomy/planning/global/astar.cpp
 src/robot/autonomy/planning/global/global_planner.cpp
 src/robot/autonomy/planning/global/hybrid_astar.cpp
 src/robot/autonomy/planning/local/holonomic_local.cpp
 src/robot/autonomy/planning/local/local_planning_system.cpp
 src/robot/autonomy/validation/collision_monitor.cpp
 src/robot/autonomy/diagnostics/decision_evidence.cpp
 src/robot/autonomy/diagnostics/vehicle_measurement.cpp
 src/robot/autonomy/planning/path_smoothing.cpp
 src/robot/autonomy/planning/task/task_ordering.cpp
 src/robot/autonomy/behavior/high_level_policy.cpp
 src/robot/autonomy/planning/validation/entry_shadow.cpp
 src/robot/autonomy/planning/validation/trajectory_validator.cpp
 src/robot/autonomy/behavior/entry_governor.cpp
 src/robot/autonomy/behavior/crossing_behavior.cpp
 src/robot/autonomy/behavior/crossing_map_semantics.cpp
 src/robot/autonomy/localization/estimation_system.cpp
 src/robot/autonomy/localization/known_map_localizer.cpp
 src/robot/autonomy/localization/global_map_localizer.cpp
 src/robot/autonomy/localization/startup_localization.cpp
 src/robot/autonomy/localization/dead_reckon.cpp
 src/robot/autonomy/perception/dynamic_obstacle_map.cpp
 src/robot/autonomy/perception/lidar_perception_system.cpp
 src/robot/autonomy/perception/perception_pipeline.cpp
 src/robot/autonomy/pipeline/autonomy_pipeline.cpp
 src/robot/autonomy/planning/local/dwa.cpp
 src/robot/autonomy/mapping/grid_map.cpp
 src/robot/autonomy/mapping/online_slam.cpp
 src/robot/autonomy/mapping/pose_graph.cpp
 src/robot/autonomy/mapping/observed_obstacle_layer.cpp
 src/robot/autonomy/planning/local/predictor.cpp
 src/robot/autonomy/localization/state_estimator.cpp
 src/robot/autonomy/mapping/terrain.cpp
 src/robot/autonomy/planning/local/pocket_shadow.cpp
 src/robot/autonomy/localization/estimator_replay.cpp
 src/robot/autonomy/localization/slip_observer.cpp
 src/robot/autonomy/localization/estimator_filter.cpp
 src/robot/autonomy/control/safe_stop_limiter.cpp
 src/robot/autonomy/control/route_tracking_system.cpp
 src/robot/autonomy/control/trajectory_tracking_system.cpp
)
target_link_libraries(lost_signal_robot_autonomy PUBLIC
 lost_signal_robot_core lost_signal_robot_platform Eigen3::Eigen)

add_library(lost_signal_robot_runtime STATIC
 src/robot/runtime/endpoint_registry.cpp
 src/robot/runtime/map_provider.cpp
 src/robot/runtime/runtime_channel.cpp
 src/robot/runtime/replay_driver.cpp
)
target_link_libraries(lost_signal_robot_runtime PUBLIC
 lost_signal_robot_core lost_signal_robot_platform)

add_library(lost_signal_robot_map_io STATIC
 src/robot/map_io/robot_map_package.cpp
)
target_link_libraries(lost_signal_robot_map_io PUBLIC
 lost_signal_robot_core lost_signal_robot_runtime nlohmann_json::nlohmann_json)

add_library(lost_signal_robot_executive STATIC
 src/robot/executive/task_executive.cpp
)
target_link_libraries(lost_signal_robot_executive PUBLIC
 lost_signal_robot_core lost_signal_robot_runtime lost_signal_robot_autonomy)

add_library(lost_signal_robot_stack INTERFACE)
target_link_libraries(lost_signal_robot_stack INTERFACE
 lost_signal_robot_core
 lost_signal_robot_platform
 lost_signal_robot_runtime
 lost_signal_robot_autonomy
 lost_signal_robot_executive
 lost_signal_robot_map_io)

# A robot-only configuration is the executable architecture gate: no MuJoCo,
# renderer, scenario content, or virtual device is configured or fetched.
if(NOT LOST_SIGNAL_BUILD_SIM)
 if(LOST_SIGNAL_BUILD_CLIENT)
 message(STATUS "LOST_SIGNAL_BUILD_CLIENT ignored because LOST_SIGNAL_BUILD_SIM=OFF")
 endif()
 if(LOST_SIGNAL_BUILD_TESTS)
 enable_testing()
 find_package(Threads REQUIRED)

 add_executable(test_runtime_interface tests/new/test_runtime_interface.cpp)
 target_link_libraries(test_runtime_interface PRIVATE lost_signal_robot_runtime Threads::Threads)
 add_test(NAME runtime_interface COMMAND test_runtime_interface)

add_executable(test_planar_odometry tests/new/test_planar_odometry.cpp)
target_link_libraries(test_planar_odometry PRIVATE lost_signal_robot_autonomy)
add_test(NAME planar_odometry COMMAND test_planar_odometry)

 add_executable(test_estimation_attitude tests/new/test_estimation_attitude.cpp)
 target_link_libraries(test_estimation_attitude PRIVATE lost_signal_robot_autonomy)
 add_test(NAME estimation_attitude COMMAND test_estimation_attitude)

 add_executable(test_sha256 tests/new/test_sha256.cpp)
 target_link_libraries(test_sha256 PRIVATE lost_signal_robot_core)
 add_test(NAME sha256 COMMAND test_sha256)

 add_executable(test_global_planner_extension tests/new/test_global_planner_extension.cpp)
 target_link_libraries(test_global_planner_extension PRIVATE lost_signal_robot_autonomy)
 add_test(NAME global_planner_extension COMMAND test_global_planner_extension)

 add_executable(test_path_smoothing tests/new/test_path_smoothing.cpp)
 target_link_libraries(test_path_smoothing PRIVATE lost_signal_robot_autonomy)
 add_test(NAME path_smoothing COMMAND test_path_smoothing)

 add_executable(test_dwa_dynamic tests/new/test_dwa_dynamic.cpp)
 target_link_libraries(test_dwa_dynamic PRIVATE lost_signal_robot_autonomy)
 add_test(NAME dwa_dynamic COMMAND test_dwa_dynamic)

 add_executable(test_autonomy_pipeline tests/new/test_autonomy_pipeline.cpp)
 target_link_libraries(test_autonomy_pipeline PRIVATE lost_signal_robot_autonomy lost_signal_robot_runtime)
 add_test(NAME autonomy_pipeline COMMAND test_autonomy_pipeline)

 add_executable(test_safe_stop_limiter tests/new/test_safe_stop_limiter.cpp)
 target_link_libraries(test_safe_stop_limiter PRIVATE lost_signal_robot_autonomy)
 add_test(NAME safe_stop_limiter COMMAND test_safe_stop_limiter)

 add_executable(test_perception_pipeline tests/new/test_perception_pipeline.cpp)
 target_link_libraries(test_perception_pipeline PRIVATE lost_signal_robot_autonomy)
 add_test(NAME perception_pipeline COMMAND test_perception_pipeline)

 add_executable(test_trajectory_validator tests/new/test_trajectory_validator.cpp)
 target_link_libraries(test_trajectory_validator PRIVATE lost_signal_robot_autonomy)
 add_test(NAME trajectory_validator COMMAND test_trajectory_validator)

 add_executable(test_task_executive tests/new/test_task_executive.cpp)
 target_link_libraries(test_task_executive PRIVATE lost_signal_robot_executive)
 add_test(NAME task_executive COMMAND test_task_executive)

 add_executable(test_map_provider tests/new/test_map_provider.cpp)
 target_link_libraries(test_map_provider PRIVATE lost_signal_robot_runtime)
 add_test(NAME map_provider COMMAND test_map_provider)

 add_executable(test_known_map_localizer tests/new/test_known_map_localizer.cpp)
 target_link_libraries(test_known_map_localizer PRIVATE lost_signal_robot_autonomy lost_signal_robot_runtime)
 add_test(NAME known_map_localizer COMMAND test_known_map_localizer)

 add_executable(test_large_map_localization tests/new/test_large_map_localization.cpp)
 target_link_libraries(test_large_map_localization PRIVATE lost_signal_robot_autonomy lost_signal_robot_runtime)
 add_test(NAME large_map_localization COMMAND test_large_map_localization)

 add_executable(test_online_slam tests/new/test_online_slam.cpp)
 target_link_libraries(test_online_slam PRIVATE lost_signal_robot_autonomy lost_signal_robot_runtime)
 add_test(NAME online_slam COMMAND test_online_slam)

 add_executable(test_pose_graph tests/new/test_pose_graph.cpp)
 target_link_libraries(test_pose_graph PRIVATE lost_signal_robot_autonomy)
 add_test(NAME pose_graph COMMAND test_pose_graph)

 add_executable(test_startup_localization tests/new/test_startup_localization.cpp)
 target_link_libraries(test_startup_localization PRIVATE lost_signal_robot_autonomy lost_signal_robot_runtime)
 add_test(NAME startup_localization COMMAND test_startup_localization)

 add_executable(test_localization_mapping_replay tests/new/test_localization_mapping_replay.cpp)
 target_link_libraries(test_localization_mapping_replay PRIVATE lost_signal_robot_autonomy lost_signal_robot_runtime)
 add_test(NAME localization_mapping_replay COMMAND test_localization_mapping_replay)

 add_executable(test_robot_map_package tests/new/test_robot_map_package.cpp)
 target_link_libraries(test_robot_map_package PRIVATE lost_signal_robot_map_io)
 add_test(NAME robot_map_package COMMAND test_robot_map_package)

 add_executable(test_feedforward tests/new/test_feedforward.cpp)
 target_link_libraries(test_feedforward PRIVATE lost_signal_robot_platform)
 add_test(NAME feedforward COMMAND test_feedforward)

 add_executable(test_differential_wheel_controller
 tests/new/test_differential_wheel_controller.cpp)
 target_link_libraries(test_differential_wheel_controller PRIVATE
 lost_signal_robot_platform)
 add_test(NAME differential_wheel_controller COMMAND
 test_differential_wheel_controller)

 add_executable(test_holonomic_actuator_controller
 tests/new/test_holonomic_actuator_controller.cpp)
 target_link_libraries(test_holonomic_actuator_controller PRIVATE
 lost_signal_robot_platform)
 add_test(NAME holonomic_actuator_controller COMMAND
 test_holonomic_actuator_controller)

 add_executable(test_ackermann_actuator_controller
 tests/new/test_ackermann_actuator_controller.cpp)
 target_link_libraries(test_ackermann_actuator_controller PRIVATE
 lost_signal_robot_platform lost_signal_robot_runtime)
 add_test(NAME ackermann_actuator_controller COMMAND
 test_ackermann_actuator_controller)

 add_executable(test_serial_manipulator tests/new/test_serial_manipulator.cpp)
 target_link_libraries(test_serial_manipulator PRIVATE lost_signal_robot_platform)
 add_test(NAME serial_manipulator COMMAND test_serial_manipulator)
 endif()
 return()
endif()

# SIM implements an LSRI provider with virtual devices and physical plants. It
# may depend on ROBOT; the reverse dependency is prohibited.
add_library(lost_signal_sim_physics STATIC
 src/sim/hardware/physics/frame_transform.cpp
 src/sim/hardware/physics/mujoco_simulation.cpp
 src/sim/hardware/physics/raycast_geometry.cpp
 src/sim/hardware/physics/tire_contact.cpp
)
target_include_directories(lost_signal_sim_physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(lost_signal_sim_physics PUBLIC cxx_std_20)
target_link_libraries(lost_signal_sim_physics PUBLIC
 lost_signal_project_options lost_signal_robot_core)

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_STUDIO OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)
set(MUJOCO_ENABLE_LTO OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
 mujoco
 URL https://github.com/google-deepmind/mujoco/archive/refs/tags/3.16.0.tar.gz
 URL_HASH SHA256=45fe830707c80cf2fa54b37811ea13d7cc1774400704f69d85d2ecc1a151a236
 DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(mujoco)
target_link_libraries(lost_signal_sim_physics PRIVATE mujoco::mujoco)

add_library(lost_signal_sim_vehicle STATIC
 src/sim/hardware/actuators/electric_drive.cpp
 src/sim/hardware/actuators/service_brake.cpp
 src/sim/hardware/vehicle/ackermann.cpp
 src/sim/hardware/vehicle/holonomic.cpp
 src/sim/hardware/vehicle/motor_drive.cpp
 src/sim/hardware/vehicle/chassis_factory.cpp
 src/sim/hardware/vehicle/tracked.cpp
 src/sim/hardware/vehicle/contact.cpp
 src/sim/hardware/vehicle/planar_diff_drive.cpp
)
target_link_libraries(lost_signal_sim_vehicle PUBLIC
 lost_signal_robot_stack lost_signal_sim_physics)

add_library(lost_signal_sim_sensors STATIC
 src/sim/hardware/sensors/sensor.cpp
 src/sim/hardware/sensors/sensor_system.cpp
)
target_link_libraries(lost_signal_sim_sensors PUBLIC
 lost_signal_robot_stack lost_signal_sim_vehicle lost_signal_sim_physics)

add_library(lost_signal_sim_geometry STATIC
 src/sim/hardware/physics/lidar_raycast.cpp
)
target_link_libraries(lost_signal_sim_geometry PUBLIC
 lost_signal_robot_core lost_signal_robot_autonomy)

add_library(lost_signal_sim_hardware INTERFACE)
target_link_libraries(lost_signal_sim_hardware INTERFACE
 lost_signal_robot_stack
 lost_signal_sim_physics
 lost_signal_sim_vehicle
 lost_signal_sim_sensors
 lost_signal_sim_geometry)

add_library(lost_signal_sim_game STATIC
 src/sim/game/mission/campaign_progress.cpp
 src/sim/game/mission/component_registry.cpp
 src/sim/game/mission/construction_plan.cpp
 src/sim/game/mission/content_catalog.cpp
 src/sim/game/mission/deployable_map_adapter.cpp
 src/sim/game/mission/map_serialize.cpp
 src/sim/game/mission/performance_score.cpp
 src/sim/game/mission/world_geometry.cpp
 src/sim/game/platform/simulated_platform.cpp
 src/sim/game/scenario/scenario_session.cpp
 src/sim/game/scenario/scenario_autonomy.cpp
 src/sim/game/mission/world/actor_behavior_registry.cpp
 src/sim/game/mission/systems/field_dispatch_system.cpp
 src/sim/game/mission/systems/field_operator_system.cpp
 src/sim/game/mission/systems/goal_system.cpp
 src/sim/game/mission/systems/world_activity_system.cpp
)
target_include_directories(lost_signal_sim_game PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(lost_signal_sim_game PUBLIC
 lost_signal_sim_hardware nlohmann_json::nlohmann_json)

set(LOST_SIGNAL_BUILD_INFO_HEADER
 ${CMAKE_CURRENT_BINARY_DIR}/generated/lost_signal/build_info.hpp)
add_custom_target(lost_signal_build_info
 COMMAND ${CMAKE_COMMAND}
 -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
 -DOUTPUT_FILE=${LOST_SIGNAL_BUILD_INFO_HEADER}
 -DPROJECT_VERSION_STRING=${PROJECT_VERSION}
 -DCOMPILER_ID=${CMAKE_CXX_COMPILER_ID}
 -DCOMPILER_VERSION=${CMAKE_CXX_COMPILER_VERSION}
 -DSYSTEM_NAME=${CMAKE_SYSTEM_NAME}
 -DSYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}
 -DBUILD_TYPE=$<CONFIG>
 -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_build_info.cmake
 BYPRODUCTS ${LOST_SIGNAL_BUILD_INFO_HEADER}
 COMMENT "Refreshing source and build identity"
 VERBATIM)

add_executable(lost_signal_headless
 src/sim/game/app/headless_main.cpp
 src/sim/game/app/run_artifact.cpp
)
add_dependencies(lost_signal_headless lost_signal_build_info)
target_include_directories(lost_signal_headless PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(lost_signal_headless PRIVATE lost_signal_sim_game)

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
 src/sim/game/client/audio_director.cpp
 src/sim/game/client/main.cpp
 src/sim/game/client/game_ui.cpp
 src/sim/game/client/map_editor.cpp
 src/sim/game/client/scene_renderer_3d.cpp
 src/sim/game/client/world_renderer.cpp
 )
 target_link_libraries(lost_signal PRIVATE lost_signal_sim_game raylib TGUI::TGUI)
 get_target_property(LOST_SIGNAL_RAYLIB_INCLUDE_DIRS raylib INTERFACE_INCLUDE_DIRECTORIES)
 if(LOST_SIGNAL_RAYLIB_INCLUDE_DIRS)
 target_include_directories(lost_signal SYSTEM PRIVATE ${LOST_SIGNAL_RAYLIB_INCLUDE_DIRS})
 endif()
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
 find_package(Threads REQUIRED)

 add_executable(test_content_catalog tests/new/test_content_catalog.cpp)
 target_link_libraries(test_content_catalog PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_content_catalog PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME content_catalog COMMAND test_content_catalog)

 add_executable(test_construction_plan tests/new/test_construction_plan.cpp)
 target_link_libraries(test_construction_plan PRIVATE lost_signal_sim_game)
 add_test(NAME construction_plan COMMAND test_construction_plan)

 add_executable(test_bridge_washout tests/new/test_bridge_washout.cpp)
 target_link_libraries(test_bridge_washout PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_bridge_washout PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME bridge_washout COMMAND test_bridge_washout)

 add_executable(test_world_geometry tests/new/test_world_geometry.cpp)
 target_link_libraries(test_world_geometry PRIVATE lost_signal_sim_game)
 add_test(NAME world_geometry COMMAND test_world_geometry)

 add_executable(test_component_registry tests/new/test_component_registry.cpp)
 target_link_libraries(test_component_registry PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_component_registry PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME component_registry COMMAND test_component_registry)

 add_executable(test_runtime_interface tests/new/test_runtime_interface.cpp)
 target_link_libraries(test_runtime_interface PRIVATE lost_signal_sim_hardware Threads::Threads)
 add_test(NAME runtime_interface COMMAND test_runtime_interface)

 add_executable(test_serial_manipulator tests/new/test_serial_manipulator.cpp)
 target_link_libraries(test_serial_manipulator PRIVATE lost_signal_robot_platform)
 add_test(NAME serial_manipulator COMMAND test_serial_manipulator)

 add_executable(test_planar_odometry tests/new/test_planar_odometry.cpp)
 target_link_libraries(test_planar_odometry PRIVATE lost_signal_robot_autonomy)
 add_test(NAME planar_odometry COMMAND test_planar_odometry)

 add_executable(test_estimation_attitude tests/new/test_estimation_attitude.cpp)
 target_link_libraries(test_estimation_attitude PRIVATE lost_signal_robot_autonomy)
 add_test(NAME estimation_attitude COMMAND test_estimation_attitude)

 add_executable(test_sha256 tests/new/test_sha256.cpp)
 target_link_libraries(test_sha256 PRIVATE lost_signal_robot_core)
 add_test(NAME sha256 COMMAND test_sha256)

 add_executable(test_global_planner_extension tests/new/test_global_planner_extension.cpp)
 target_link_libraries(test_global_planner_extension PRIVATE lost_signal_robot_autonomy)
 add_test(NAME global_planner_extension COMMAND test_global_planner_extension)

 add_executable(test_field_dispatch tests/new/test_field_dispatch.cpp)
 target_link_libraries(test_field_dispatch PRIVATE lost_signal_sim_game)
 add_test(NAME field_dispatch COMMAND test_field_dispatch)

 add_executable(test_world_activity tests/new/test_world_activity.cpp)
 target_link_libraries(test_world_activity PRIVATE lost_signal_sim_game)
 add_test(NAME world_activity COMMAND test_world_activity)

 add_executable(test_physics_world tests/new/test_physics_world.cpp)
 target_link_libraries(test_physics_world PRIVATE lost_signal_sim_physics)
 add_test(NAME physics_world COMMAND test_physics_world)

 add_executable(test_simulated_platform tests/new/test_simulated_platform.cpp)
 target_link_libraries(test_simulated_platform PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_simulated_platform PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME simulated_platform COMMAND test_simulated_platform)

add_executable(test_ro1_dynamics_acceptance tests/new/test_ro1_dynamics_acceptance.cpp)
target_link_libraries(test_ro1_dynamics_acceptance PRIVATE lost_signal_sim_game)
target_compile_definitions(test_ro1_dynamics_acceptance PRIVATE
  LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
add_test(NAME ro1_dynamics_acceptance COMMAND test_ro1_dynamics_acceptance)

 add_executable(test_ro2_dynamics_acceptance tests/new/test_ro2_dynamics_acceptance.cpp)
 target_link_libraries(test_ro2_dynamics_acceptance PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_ro2_dynamics_acceptance PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME ro2_dynamics_acceptance COMMAND test_ro2_dynamics_acceptance)

add_executable(test_ro3_dynamics_acceptance tests/new/test_ro3_dynamics_acceptance.cpp)
target_link_libraries(test_ro3_dynamics_acceptance PRIVATE lost_signal_sim_game)
target_compile_definitions(test_ro3_dynamics_acceptance PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
add_test(NAME ro3_dynamics_acceptance COMMAND test_ro3_dynamics_acceptance)

add_executable(test_holonomic_dynamics_acceptance
 tests/new/test_holonomic_dynamics_acceptance.cpp)
target_link_libraries(test_holonomic_dynamics_acceptance PRIVATE lost_signal_sim_game)
target_compile_definitions(test_holonomic_dynamics_acceptance PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
add_test(NAME holonomic_dynamics_acceptance COMMAND
 test_holonomic_dynamics_acceptance)

add_executable(test_ro6_manipulator_acceptance
 tests/new/test_ro6_manipulator_acceptance.cpp)
target_link_libraries(test_ro6_manipulator_acceptance PRIVATE lost_signal_sim_game)
target_compile_definitions(test_ro6_manipulator_acceptance PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
add_test(NAME ro6_manipulator_acceptance COMMAND
 test_ro6_manipulator_acceptance)

add_executable(test_ro1_lidar_acceptance tests/new/test_ro1_lidar_acceptance.cpp)
target_link_libraries(test_ro1_lidar_acceptance PRIVATE lost_signal_sim_game)
target_compile_definitions(test_ro1_lidar_acceptance PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
add_test(NAME ro1_lidar_acceptance COMMAND test_ro1_lidar_acceptance)

 add_executable(test_terrain tests/new/test_terrain.cpp)
 target_link_libraries(test_terrain PRIVATE lost_signal_sim_hardware)
 add_test(NAME terrain COMMAND test_terrain)

 add_executable(test_kinematic_navigation tests/new/test_kinematic_navigation.cpp)
 target_link_libraries(test_kinematic_navigation PRIVATE lost_signal_sim_hardware)
 add_test(NAME kinematic_navigation COMMAND test_kinematic_navigation)

 add_executable(test_high_level_policy tests/new/test_high_level_policy.cpp)
 target_link_libraries(test_high_level_policy PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_high_level_policy PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME high_level_policy COMMAND test_high_level_policy)

	 add_executable(test_field_operator tests/new/test_field_operator.cpp)
	 target_link_libraries(test_field_operator PRIVATE lost_signal_sim_game)
	 target_compile_definitions(test_field_operator PRIVATE
	 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
	 )
	 add_test(NAME field_operator COMMAND test_field_operator)

 add_executable(test_vehicle_measurement_system
 tests/new/test_vehicle_measurement_system.cpp)
 target_link_libraries(test_vehicle_measurement_system PRIVATE lost_signal_sim_game)
 add_test(NAME vehicle_measurement_system COMMAND test_vehicle_measurement_system)

 add_executable(test_mission_simulation tests/new/test_mission_simulation.cpp)
 target_link_libraries(test_mission_simulation PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_mission_simulation PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 if(MSVC)
 # This integration test constructs many full simulation worlds in one
 # function. MSVC reserves their aggregate frame at entry, beyond its 1 MiB
 # executable default, even though the worlds have disjoint lexical scopes.
 target_link_options(test_mission_simulation PRIVATE /STACK:8388608)
 endif()
 add_test(NAME mission_simulation COMMAND test_mission_simulation)

 add_executable(test_performance_score tests/new/test_performance_score.cpp)
 target_link_libraries(test_performance_score PRIVATE lost_signal_sim_game)
 add_test(NAME performance_score COMMAND test_performance_score)

 add_executable(test_dynamic_perception tests/new/test_dynamic_perception.cpp)
 target_link_libraries(test_dynamic_perception PRIVATE lost_signal_sim_hardware)
 add_test(NAME dynamic_perception COMMAND test_dynamic_perception)

add_executable(test_dwa_dynamic tests/new/test_dwa_dynamic.cpp)
target_link_libraries(test_dwa_dynamic PRIVATE lost_signal_robot_autonomy)
add_test(NAME dwa_dynamic COMMAND test_dwa_dynamic)

 add_executable(test_path_smoothing tests/new/test_path_smoothing.cpp)
 target_link_libraries(test_path_smoothing PRIVATE lost_signal_robot_autonomy)
 add_test(NAME path_smoothing COMMAND test_path_smoothing)

 add_executable(test_dwell_behavior tests/new/test_dwell_behavior.cpp)
 target_link_libraries(test_dwell_behavior PRIVATE lost_signal_sim_hardware)
 add_test(NAME dwell_behavior COMMAND test_dwell_behavior)

 add_executable(test_behavior_registry tests/new/test_behavior_registry.cpp)
target_link_libraries(test_behavior_registry PRIVATE lost_signal_sim_game)
 add_test(NAME behavior_registry COMMAND test_behavior_registry)

 add_executable(test_entry_shadow tests/new/test_entry_shadow.cpp)
 target_link_libraries(test_entry_shadow PRIVATE lost_signal_sim_hardware)
 add_test(NAME entry_shadow COMMAND test_entry_shadow)

 add_executable(test_entry_shadow_field tests/new/test_entry_shadow_field.cpp)
 target_link_libraries(test_entry_shadow_field PRIVATE lost_signal_sim_game)
 target_include_directories(test_entry_shadow_field PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
 target_compile_definitions(test_entry_shadow_field PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME entry_shadow_field COMMAND test_entry_shadow_field)

 add_executable(test_stop_line_study tests/new/test_stop_line_study.cpp)
 target_link_libraries(test_stop_line_study PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_stop_line_study PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME stop_line_study COMMAND test_stop_line_study)

 add_executable(test_stop_whatif tests/new/test_stop_whatif.cpp)
 target_link_libraries(test_stop_whatif PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_stop_whatif PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 target_include_directories(test_stop_whatif PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
 add_test(NAME stop_whatif COMMAND test_stop_whatif)

 add_executable(test_entry_shadow_town tests/new/test_entry_shadow_town.cpp)
 target_link_libraries(test_entry_shadow_town PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_entry_shadow_town PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 target_include_directories(test_entry_shadow_town PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
 add_test(NAME entry_shadow_town COMMAND test_entry_shadow_town)

 add_executable(test_manual_ram tests/new/test_manual_ram.cpp)
 target_link_libraries(test_manual_ram PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_manual_ram PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 target_include_directories(test_manual_ram PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
 add_test(NAME manual_ram COMMAND test_manual_ram)

 add_executable(test_entry_governor tests/new/test_entry_governor.cpp)
 target_link_libraries(test_entry_governor PRIVATE lost_signal_sim_hardware)
 add_test(NAME entry_governor COMMAND test_entry_governor)

 add_executable(test_crossing_map_semantics tests/new/test_crossing_map_semantics.cpp)
 target_link_libraries(test_crossing_map_semantics PRIVATE lost_signal_robot_autonomy)
 add_test(NAME crossing_map_semantics COMMAND test_crossing_map_semantics)

 add_executable(test_trajectory_validator tests/new/test_trajectory_validator.cpp)
 target_link_libraries(test_trajectory_validator PRIVATE lost_signal_robot_autonomy)
 add_test(NAME trajectory_validator COMMAND test_trajectory_validator)

 add_executable(test_observed_obstacle_layer tests/new/test_observed_obstacle_layer.cpp)
 target_link_libraries(test_observed_obstacle_layer PRIVATE lost_signal_robot_autonomy)
 add_test(NAME observed_obstacle_layer COMMAND test_observed_obstacle_layer)

 add_executable(test_perception_pipeline tests/new/test_perception_pipeline.cpp)
 target_link_libraries(test_perception_pipeline PRIVATE lost_signal_robot_autonomy)
 add_test(NAME perception_pipeline COMMAND test_perception_pipeline)

add_executable(test_autonomy_pipeline tests/new/test_autonomy_pipeline.cpp)
target_link_libraries(test_autonomy_pipeline PRIVATE lost_signal_robot_autonomy lost_signal_robot_runtime)
add_test(NAME autonomy_pipeline COMMAND test_autonomy_pipeline)

add_executable(test_safe_stop_limiter tests/new/test_safe_stop_limiter.cpp)
target_link_libraries(test_safe_stop_limiter PRIVATE lost_signal_robot_autonomy)
add_test(NAME safe_stop_limiter COMMAND test_safe_stop_limiter)

 add_executable(test_deterministic_scheduler tests/new/test_deterministic_scheduler.cpp)
 target_link_libraries(test_deterministic_scheduler PRIVATE lost_signal_sim_game)
 add_test(NAME deterministic_scheduler COMMAND test_deterministic_scheduler)

add_executable(test_collision_monitor tests/new/test_collision_monitor.cpp)
target_link_libraries(test_collision_monitor PRIVATE lost_signal_sim_game)
add_test(NAME collision_monitor COMMAND test_collision_monitor)

add_executable(test_decision_evidence_system tests/new/test_decision_evidence_system.cpp)
target_link_libraries(test_decision_evidence_system PRIVATE lost_signal_sim_game)
add_test(NAME decision_evidence_system COMMAND test_decision_evidence_system)

 add_executable(test_trajectory_tracking_system tests/new/test_trajectory_tracking_system.cpp)
 target_link_libraries(test_trajectory_tracking_system PRIVATE lost_signal_robot_autonomy)
 add_test(NAME trajectory_tracking_system COMMAND test_trajectory_tracking_system)

 add_executable(test_algo_invariants tests/new/test_algo_invariants.cpp)
 target_link_libraries(test_algo_invariants PRIVATE lost_signal_sim_hardware)
 add_test(NAME algo_invariants COMMAND test_algo_invariants)

 add_executable(test_summon_recharge tests/new/test_summon_recharge.cpp)
 target_link_libraries(test_summon_recharge PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_summon_recharge PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME summon_recharge COMMAND test_summon_recharge)


 add_executable(test_model_validation tests/new/test_model_validation.cpp)
 target_link_libraries(test_model_validation PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_model_validation PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME model_validation COMMAND test_model_validation)

 add_executable(test_chassis tests/new/test_chassis.cpp)
 target_link_libraries(test_chassis PRIVATE lost_signal_sim_hardware)
 add_test(NAME chassis COMMAND test_chassis)

 add_executable(test_map_editor tests/new/test_map_editor.cpp src/sim/game/client/map_editor.cpp)
 target_link_libraries(test_map_editor PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_map_editor PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 target_include_directories(test_map_editor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
 add_test(NAME map_editor COMMAND test_map_editor)

 add_executable(test_map_roundtrip tests/new/test_map_roundtrip.cpp)
 target_link_libraries(test_map_roundtrip PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_map_roundtrip PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME map_roundtrip COMMAND test_map_roundtrip)

 add_executable(test_predictor_parity tests/new/test_predictor_parity.cpp)
 target_link_libraries(test_predictor_parity PRIVATE lost_signal_sim_hardware)
 add_test(NAME predictor_parity COMMAND test_predictor_parity)

 add_executable(test_slip_detect tests/new/test_slip_detect.cpp)
 target_link_libraries(test_slip_detect PRIVATE lost_signal_sim_hardware)
 add_test(NAME slip_detect COMMAND test_slip_detect)

 add_executable(test_slip_shadow tests/new/test_slip_shadow.cpp)
 target_link_libraries(test_slip_shadow PRIVATE lost_signal_sim_hardware lost_signal_sim_game)
 target_compile_definitions(test_slip_shadow PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME slip_shadow COMMAND test_slip_shadow)

 add_executable(test_departure_replay tests/new/test_departure_replay.cpp)
 target_link_libraries(test_departure_replay PRIVATE lost_signal_sim_hardware)
 add_test(NAME departure_replay COMMAND test_departure_replay)

 add_executable(test_phase_legs tests/new/test_phase_legs.cpp)
 target_link_libraries(test_phase_legs PRIVATE lost_signal_sim_hardware)
 add_test(NAME phase_legs COMMAND test_phase_legs)

 add_executable(test_goal_system tests/new/test_goal_system.cpp)
 target_link_libraries(test_goal_system PRIVATE lost_signal_sim_game)
 add_test(NAME goal_system COMMAND test_goal_system)

 add_executable(test_player_goal_order tests/new/test_player_goal_order.cpp)
 target_link_libraries(test_player_goal_order PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_player_goal_order PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME player_goal_order COMMAND test_player_goal_order)

 add_executable(test_task_executive tests/new/test_task_executive.cpp)
 target_link_libraries(test_task_executive PRIVATE lost_signal_robot_executive)
 add_test(NAME task_executive COMMAND test_task_executive)

 add_executable(test_map_provider tests/new/test_map_provider.cpp)
 target_link_libraries(test_map_provider PRIVATE lost_signal_robot_runtime)
 add_test(NAME map_provider COMMAND test_map_provider)

 add_executable(test_known_map_localizer tests/new/test_known_map_localizer.cpp)
 target_link_libraries(test_known_map_localizer PRIVATE lost_signal_robot_autonomy lost_signal_robot_runtime)
 add_test(NAME known_map_localizer COMMAND test_known_map_localizer)

 add_executable(test_large_map_localization tests/new/test_large_map_localization.cpp)
 target_link_libraries(test_large_map_localization PRIVATE lost_signal_robot_autonomy lost_signal_robot_runtime)
 add_test(NAME large_map_localization COMMAND test_large_map_localization)

 add_executable(test_robot_map_package tests/new/test_robot_map_package.cpp)
 target_link_libraries(test_robot_map_package PRIVATE lost_signal_robot_map_io)
 add_test(NAME robot_map_package COMMAND test_robot_map_package)

 add_executable(test_deployable_map_adapter tests/new/test_deployable_map_adapter.cpp)
 target_link_libraries(test_deployable_map_adapter PRIVATE lost_signal_sim_game)
 add_test(NAME deployable_map_adapter COMMAND test_deployable_map_adapter)

 add_executable(test_orchard_scramble tests/new/test_orchard_scramble.cpp)
 target_link_libraries(test_orchard_scramble PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_orchard_scramble PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME orchard_scramble COMMAND test_orchard_scramble)

 add_executable(test_pocket_search tests/new/test_pocket_search.cpp)
 target_link_libraries(test_pocket_search PRIVATE lost_signal_sim_hardware)
 add_test(NAME pocket_search COMMAND test_pocket_search)

 add_executable(test_perturbation tests/new/test_perturbation.cpp)
 target_link_libraries(test_perturbation PRIVATE lost_signal_sim_hardware)
 add_test(NAME perturbation COMMAND test_perturbation)

 add_executable(test_estimation_error tests/new/test_estimation_error.cpp)
 target_link_libraries(test_estimation_error PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_estimation_error PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME estimation_error COMMAND test_estimation_error)

 add_executable(test_release_error tests/new/test_release_error.cpp)
 target_link_libraries(test_release_error PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_release_error PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME release_error COMMAND test_release_error)

 add_executable(test_player_pack tests/new/test_player_pack.cpp)
 target_link_libraries(test_player_pack PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_player_pack PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME player_pack COMMAND test_player_pack)

 add_executable(test_chassis_choice tests/new/test_chassis_choice.cpp)
 target_link_libraries(test_chassis_choice PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_chassis_choice PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME chassis_choice COMMAND test_chassis_choice)

 add_executable(test_abilities tests/new/test_abilities.cpp)
 target_link_libraries(test_abilities PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_abilities PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME abilities COMMAND test_abilities)

 add_executable(test_contract_rep tests/new/test_contract_rep.cpp)
 target_link_libraries(test_contract_rep PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_contract_rep PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME contract_rep COMMAND test_contract_rep)

 add_executable(test_signal_cross tests/new/test_signal_cross.cpp)
 target_link_libraries(test_signal_cross PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_signal_cross PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME signal_cross COMMAND test_signal_cross)

 add_executable(test_shadow_accumulation tests/new/test_shadow_accumulation.cpp)
 target_link_libraries(test_shadow_accumulation PRIVATE lost_signal_sim_game)
 target_compile_definitions(test_shadow_accumulation PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME shadow_accumulation COMMAND test_shadow_accumulation)

 add_executable(test_signal_quality tests/new/test_signal_quality.cpp)
 target_link_libraries(test_signal_quality PRIVATE lost_signal_sim_hardware lost_signal_sim_game)
 target_compile_definitions(test_signal_quality PRIVATE
 LOST_SIGNAL_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets"
 )
 add_test(NAME signal_quality COMMAND test_signal_quality)

 add_executable(test_inverse_consistency tests/new/test_inverse_consistency.cpp)
 target_link_libraries(test_inverse_consistency PRIVATE lost_signal_sim_hardware)
 add_test(NAME inverse_consistency COMMAND test_inverse_consistency)

 add_executable(test_feedforward tests/new/test_feedforward.cpp)
 target_link_libraries(test_feedforward PRIVATE lost_signal_sim_hardware)
 add_test(NAME feedforward COMMAND test_feedforward)

 add_executable(test_differential_wheel_controller
 tests/new/test_differential_wheel_controller.cpp)
 target_link_libraries(test_differential_wheel_controller PRIVATE
 lost_signal_robot_platform)
 add_test(NAME differential_wheel_controller COMMAND
 test_differential_wheel_controller)

 add_executable(test_holonomic_actuator_controller
 tests/new/test_holonomic_actuator_controller.cpp)
 target_link_libraries(test_holonomic_actuator_controller PRIVATE
 lost_signal_robot_platform)
 add_test(NAME holonomic_actuator_controller COMMAND
 test_holonomic_actuator_controller)

 add_executable(test_ackermann_actuator_controller
 tests/new/test_ackermann_actuator_controller.cpp)
 target_link_libraries(test_ackermann_actuator_controller PRIVATE
 lost_signal_robot_platform lost_signal_robot_runtime)
 add_test(NAME ackermann_actuator_controller COMMAND
 test_ackermann_actuator_controller)

 add_executable(test_service_brake tests/new/test_service_brake.cpp)
 target_link_libraries(test_service_brake PRIVATE lost_signal_sim_vehicle)
 add_test(NAME service_brake COMMAND test_service_brake)

 add_executable(test_state_estimator tests/new/test_state_estimator.cpp)
 target_link_libraries(test_state_estimator PRIVATE lost_signal_sim_hardware)
 add_test(NAME state_estimator COMMAND test_state_estimator)

 add_executable(test_ro2_pocket_shadow tests/new/test_ro2_pocket_shadow.cpp)
 target_link_libraries(test_ro2_pocket_shadow PRIVATE lost_signal_sim_hardware)
 add_test(NAME ro2_pocket_shadow COMMAND test_ro2_pocket_shadow)

 add_executable(test_estimator_replay tests/new/test_estimator_replay.cpp)
 target_link_libraries(test_estimator_replay PRIVATE lost_signal_sim_hardware)
 add_test(NAME estimator_replay COMMAND test_estimator_replay)
 add_executable(test_slip_observer tests/new/test_slip_observer.cpp)
 target_link_libraries(test_slip_observer PRIVATE lost_signal_sim_hardware)
 add_test(NAME slip_observer COMMAND test_slip_observer)
 add_executable(test_estimator_filter tests/new/test_estimator_filter.cpp)
 target_link_libraries(test_estimator_filter PRIVATE lost_signal_sim_hardware)
 add_test(NAME estimator_filter COMMAND test_estimator_filter)
 add_executable(test_estimator_chain tests/new/test_estimator_chain.cpp)
 target_link_libraries(test_estimator_chain PRIVATE lost_signal_sim_hardware)
 add_test(NAME estimator_chain COMMAND test_estimator_chain)

 add_executable(test_ff_sensitivity tests/new/test_ff_sensitivity.cpp)
 target_link_libraries(test_ff_sensitivity PRIVATE lost_signal_sim_hardware lost_signal_sim_game)
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
 add_test(
 NAME contract_content
 COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_contracts.py -q
 )
 set_tests_properties(contract_content PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
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
 add_test(
 NAME compare_runs
 COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_compare_runs.py -q
 )
 set_tests_properties(compare_runs PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
 add_test(
 NAME deployable_boundary
 COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_deployable_boundary.py -q
 )
 set_tests_properties(deployable_boundary PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
 add_test(
 NAME run_artifact
 COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_run_artifact.py
 $<TARGET_FILE:lost_signal_headless> -q
 )
 set_tests_properties(run_artifact PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
 endif()
endif()
