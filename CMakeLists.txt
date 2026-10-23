cmake_minimum_required(VERSION 3.20)
project(lost_signal_refactor VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Robot stack: deployable, must never depend on sim.
add_library(lost_signal_robot_core STATIC
  src/robot/core/version.cpp
  src/robot/core/json.cpp
  src/robot/core/transform.cpp
)
target_include_directories(lost_signal_robot_core PUBLIC src)

add_library(lost_signal_robot_iface STATIC
  src/robot/iface/iface.cpp
  src/robot/iface/endpoint.cpp
  src/robot/iface/channel.cpp
  src/robot/iface/message.cpp
  src/robot/iface/conformance.cpp
)
target_include_directories(lost_signal_robot_iface PUBLIC src)
target_link_libraries(lost_signal_robot_iface PUBLIC lost_signal_robot_core)

add_library(lost_signal_robot_control STATIC
  src/robot/control/pid.cpp
  src/robot/control/feedforward.cpp
  src/robot/control/wheel_controller.cpp
)
target_include_directories(lost_signal_robot_control PUBLIC src)
target_link_libraries(lost_signal_robot_control PUBLIC lost_signal_robot_iface)

add_library(lost_signal_robot_platform STATIC
  src/robot/platform/description.cpp
  src/robot/platform/chassis.cpp
  src/robot/platform/differential_kinematics.cpp
  src/robot/platform/ackermann_kinematics.cpp
  src/robot/platform/holonomic_kinematics.cpp
  src/robot/platform/drive_profile.cpp
  src/robot/platform/chassis_registry.cpp
  src/robot/platform/transform_tree.cpp
  src/robot/platform/robot_config.cpp
  # Ackermann/holonomic controllers live here (not in robot_control) because
  # they call the canonical platform kinematics; robot_control must stay
  # below robot_platform (cf. the inlined twist math in wheel_controller).
  src/robot/control/ackermann_controller.cpp
  src/robot/control/holonomic_controller.cpp
)
target_include_directories(lost_signal_robot_platform PUBLIC src)
target_link_libraries(lost_signal_robot_platform PUBLIC
  lost_signal_robot_iface
  lost_signal_robot_control
)

add_library(lost_signal_robot_lifecycle STATIC
  src/robot/lifecycle/bringup.cpp
)
target_include_directories(lost_signal_robot_lifecycle PUBLIC src)
target_link_libraries(lost_signal_robot_lifecycle PUBLIC lost_signal_robot_core)

add_library(lost_signal_robot_autonomy STATIC
  src/robot/autonomy/coordinator.cpp
  src/robot/autonomy/behavior/behavior.cpp
  src/robot/autonomy/estimation/estimator.cpp
  src/robot/autonomy/estimation/slip_observer.cpp
  src/robot/autonomy/localization/scan_matcher.cpp
  src/robot/autonomy/localization/global_search.cpp
  src/robot/autonomy/localization/supervisor.cpp
  src/robot/autonomy/mapping/mapper.cpp
  src/robot/autonomy/mapping/pose_graph.cpp
  src/robot/autonomy/mapping/loop_closure.cpp
  src/robot/autonomy/mapping/slam.cpp
  src/robot/autonomy/perception/lidar_perception.cpp
  src/robot/autonomy/planning/astar_planner.cpp
  src/robot/autonomy/planning/dwa_planner.cpp
  src/robot/autonomy/planning/global_planner.cpp
  src/robot/autonomy/planning/prediction.cpp
  src/robot/autonomy/planning/task/task_ordering.cpp
  src/robot/autonomy/control/stop_latch.cpp
  src/robot/autonomy/diagnostics/decision_evidence.cpp
  src/robot/autonomy/diagnostics/vehicle_measurement.cpp
  src/robot/autonomy/safety/safety.cpp
)
target_include_directories(lost_signal_robot_autonomy PUBLIC src)
# Pose-graph backend solves sparse normal equations with Eigen (header-only,
# deployment dependency only — no GTSAM/Ceres).
find_path(EIGEN3_INCLUDE_DIR signature_of_eigen3_matrix_library
  PATHS /usr/include/eigen3 /usr/include NO_DEFAULT_PATH)
if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen3 headers not found (need signature_of_eigen3_matrix_library)")
endif()
target_include_directories(lost_signal_robot_autonomy PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(lost_signal_robot_autonomy PUBLIC
  lost_signal_robot_iface
  lost_signal_robot_lifecycle
  lost_signal_robot_platform
)

add_library(lost_signal_robot_executive STATIC
  src/robot/executive/executive.cpp
)
target_include_directories(lost_signal_robot_executive PUBLIC src)
target_link_libraries(lost_signal_robot_executive PUBLIC
  lost_signal_robot_iface
  lost_signal_robot_lifecycle
)

add_library(lost_signal_robot_app STATIC
  src/robot/app/robot_application.cpp
)
target_include_directories(lost_signal_robot_app PUBLIC src)
target_link_libraries(lost_signal_robot_app PUBLIC
  lost_signal_robot_autonomy
  lost_signal_robot_executive
  lost_signal_robot_platform
  lost_signal_robot_lifecycle
)

# Deployable chassis robots (differential today, ackermann/holonomic later).
add_library(lost_signal_robot_robots STATIC
  src/robot/robots/differential_robot.cpp
)
target_include_directories(lost_signal_robot_robots PUBLIC src)
target_link_libraries(lost_signal_robot_robots PUBLIC lost_signal_robot_app)

# Deployable replay/native providers (no hardware SDK in this repo).
add_library(lost_signal_robot_providers STATIC
  src/robot/providers/real_lidar2d.cpp
)
target_include_directories(lost_signal_robot_providers PUBLIC src)
target_link_libraries(lost_signal_robot_providers PUBLIC lost_signal_robot_iface)

add_library(lost_signal_robot_stack INTERFACE)
target_link_libraries(lost_signal_robot_stack INTERFACE
  lost_signal_robot_app
  lost_signal_robot_robots
  lost_signal_robot_providers
  lost_signal_robot_control
)

# Native robot binary: provider-neutral, links only the robot stack.
add_executable(lost_signal_robot
  src/robot/app/main.cpp
)
target_link_libraries(lost_signal_robot PRIVATE lost_signal_robot_stack)

# Sim side: owns world truth, physics, presentation. May depend on robot.
option(LOST_SIGNAL_WITH_MUJOCO "Build MuJoCo physics backend (SIM-side only)" ON)
if(LOST_SIGNAL_WITH_MUJOCO)
  include(FetchContent)
  set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG 3.16.0
  )
  FetchContent_MakeAvailable(mujoco)
endif()

if(LOST_SIGNAL_WITH_MUJOCO)
  add_executable(test_mujoco
    tests/test_mujoco.cpp
  )
  target_link_libraries(test_mujoco PRIVATE mujoco::mujoco)
  add_test(NAME mujoco COMMAND test_mujoco)
endif()

if(LOST_SIGNAL_WITH_MUJOCO)
  add_executable(test_mujoco_backend
    tests/test_mujoco_backend.cpp
  )
  target_link_libraries(test_mujoco_backend PRIVATE lost_signal_sim_scenario)
  add_test(NAME mujoco_backend COMMAND test_mujoco_backend)
endif()

# Viewer side: SIM-side presentation only (reads truth, never ROBOT input).
# Same FetchContent pattern as MuJoCo; the viewer binary is excluded from
# headless runs (needs a display) but frame logic is unit-tested.
option(LOST_SIGNAL_WITH_VIEWER "Build raylib viewer + map editor (SIM-side only)" ON)
if(LOST_SIGNAL_WITH_VIEWER)
  include(FetchContent)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
  )
  FetchContent_MakeAvailable(raylib)
endif()

if(LOST_SIGNAL_WITH_VIEWER)
  add_executable(sim_viewer
    src/sim/viewer/viewer_main.cpp
  )
  target_link_libraries(sim_viewer PRIVATE lost_signal_sim_viewer
    lost_signal_sim_scenario raylib)
endif()

add_library(lost_signal_sim_world STATIC
  src/sim/world/world.cpp
  src/sim/world/motor.cpp
  src/sim/world/service_brake.cpp
  src/sim/world/contact.cpp
  src/sim/world/actor_behavior.cpp
)
if(LOST_SIGNAL_WITH_MUJOCO)
  target_sources(lost_signal_sim_world PRIVATE src/sim/world/mujoco_backend.cpp)
  target_link_libraries(lost_signal_sim_world PUBLIC mujoco::mujoco)
  target_compile_definitions(lost_signal_sim_world PUBLIC LOST_SIGNAL_WITH_MUJOCO)
endif()
target_include_directories(lost_signal_sim_world PUBLIC src)
target_link_libraries(lost_signal_sim_world PUBLIC
  lost_signal_robot_core
  lost_signal_robot_iface
  lost_signal_robot_platform
)

add_library(lost_signal_sim_providers STATIC
  src/sim/providers/sim_motion.cpp
  src/sim/providers/sim_imu.cpp
  src/sim/providers/lidar2d.cpp
  src/sim/providers/lidar3d.cpp
  src/sim/providers/sensor_suite.cpp
  src/sim/providers/static_map.cpp
  src/sim/providers/replay.cpp
)
target_include_directories(lost_signal_sim_providers PUBLIC src)
target_link_libraries(lost_signal_sim_providers PUBLIC
  lost_signal_robot_stack
  lost_signal_sim_world
)

add_library(lost_signal_sim_eval STATIC
  src/sim/eval/scoring.cpp
  src/sim/eval/recording.cpp
  src/sim/eval/drive_profile_io.cpp
  src/sim/eval/map_package.cpp
  src/sim/eval/world_io.cpp
  src/sim/eval/robot_config.cpp
  src/sim/eval/sim_config.cpp
)
target_include_directories(lost_signal_sim_eval PUBLIC src)
target_link_libraries(lost_signal_sim_eval PUBLIC
  lost_signal_robot_stack
  lost_signal_sim_world
)

add_library(lost_signal_sim_scenario STATIC
  src/sim/scenario/scenario_session.cpp
  src/sim/scenario/collision_judge.cpp
  src/sim/scenario/objectives.cpp
  src/sim/robot/sim_robot.cpp
)
target_include_directories(lost_signal_sim_scenario PUBLIC src)
target_link_libraries(lost_signal_sim_scenario PUBLIC
  lost_signal_robot_stack
  lost_signal_sim_providers
  lost_signal_sim_eval
)

# Sim binary: the only target allowed to link sim + rendering/scoring later.
add_executable(lost_signal_sim
  src/sim/scenario/sim_main.cpp
)
target_link_libraries(lost_signal_sim PRIVATE lost_signal_sim_scenario)

# Viewer model library: frame building + map editing. Pure SIM-side logic,
# no raylib dependency (headless unit-tested). The raylib frontend links
# this plus raylib; only the frontend opens a window.
add_library(lost_signal_sim_viewer STATIC
  src/sim/viewer/frame_builder.cpp
  src/sim/viewer/map_editor.cpp
)
target_include_directories(lost_signal_sim_viewer PUBLIC src)
target_link_libraries(lost_signal_sim_viewer PUBLIC
  lost_signal_sim_eval
  lost_signal_sim_world
)

# Tests
enable_testing()

add_executable(test_robot_cycle
  tests/test_robot_cycle.cpp
)
target_link_libraries(test_robot_cycle PRIVATE lost_signal_robot_stack)
add_test(NAME robot_cycle COMMAND test_robot_cycle)

add_executable(test_transport
  tests/test_transport.cpp
)
target_link_libraries(test_transport PRIVATE lost_signal_robot_stack)
add_test(NAME transport COMMAND test_transport)

add_executable(test_truth_isolation
  tests/test_truth_isolation.cpp
)
target_link_libraries(test_truth_isolation PRIVATE lost_signal_sim_scenario)
add_test(NAME truth_isolation COMMAND test_truth_isolation)

add_executable(test_lifecycle
  tests/test_lifecycle.cpp
)
target_link_libraries(test_lifecycle PRIVATE lost_signal_robot_lifecycle)
add_test(NAME lifecycle COMMAND test_lifecycle)

add_executable(test_autonomy
  tests/test_autonomy.cpp
)
target_link_libraries(test_autonomy PRIVATE lost_signal_robot_autonomy)
add_test(NAME autonomy COMMAND test_autonomy)

add_executable(test_replay
  tests/test_replay.cpp
)
target_link_libraries(test_replay PRIVATE lost_signal_sim_scenario)
add_test(NAME replay COMMAND test_replay)

add_executable(test_conformance
  tests/test_conformance.cpp
)
target_link_libraries(test_conformance PRIVATE lost_signal_sim_scenario lost_signal_robot_providers)
add_test(NAME conformance COMMAND test_conformance)

add_executable(test_slam
  tests/test_slam.cpp
)
target_link_libraries(test_slam PRIVATE lost_signal_robot_autonomy)
add_test(NAME slam COMMAND test_slam)

add_executable(test_task_ordering
  tests/test_task_ordering.cpp
)
target_link_libraries(test_task_ordering PRIVATE lost_signal_robot_autonomy)
add_test(NAME task_ordering COMMAND test_task_ordering)

add_executable(test_places
  tests/test_places.cpp
)
target_link_libraries(test_places PRIVATE lost_signal_robot_stack)
add_test(NAME places COMMAND test_places)

add_executable(test_goal_auth
  tests/test_goal_auth.cpp
)
target_link_libraries(test_goal_auth PRIVATE lost_signal_robot_stack)
add_test(NAME goal_auth COMMAND test_goal_auth)

add_executable(test_localization
  tests/test_localization.cpp
)
target_link_libraries(test_localization PRIVATE lost_signal_robot_autonomy)
add_test(NAME localization COMMAND test_localization)

add_executable(test_perception
  tests/test_perception.cpp
)
target_link_libraries(test_perception PRIVATE lost_signal_robot_autonomy)
add_test(NAME perception COMMAND test_perception)

add_executable(test_sim_actor
  tests/test_sim_actor.cpp
)
target_link_libraries(test_sim_actor PRIVATE lost_signal_sim_scenario)
add_test(NAME sim_actor COMMAND test_sim_actor)

add_executable(test_astar
  tests/test_astar.cpp
)
target_link_libraries(test_astar PRIVATE lost_signal_robot_autonomy)
add_test(NAME astar COMMAND test_astar)



add_executable(test_sim_nav
  tests/test_sim_nav.cpp
)
target_link_libraries(test_sim_nav PRIVATE lost_signal_sim_scenario)
add_test(NAME sim_nav COMMAND test_sim_nav)

add_executable(test_actuation
  tests/test_actuation.cpp
)
target_link_libraries(test_actuation PRIVATE lost_signal_robot_control lost_signal_robot_platform lost_signal_robot_iface lost_signal_robot_autonomy lost_signal_sim_world)
add_test(NAME actuation COMMAND test_actuation)

add_executable(test_dwa
  tests/test_dwa.cpp
)
target_link_libraries(test_dwa PRIVATE lost_signal_robot_autonomy)
add_test(NAME dwa COMMAND test_dwa)

add_executable(test_safety
  tests/test_safety.cpp
)
target_link_libraries(test_safety PRIVATE lost_signal_robot_autonomy)
add_test(NAME safety COMMAND test_safety)

add_executable(test_behavior
  tests/test_behavior.cpp
)
target_link_libraries(test_behavior PRIVATE lost_signal_robot_autonomy)
add_test(NAME behavior COMMAND test_behavior)

add_executable(test_chassis
  tests/test_chassis.cpp
)
target_link_libraries(test_chassis PRIVATE lost_signal_robot_platform)
add_test(NAME chassis COMMAND test_chassis)

add_executable(test_providers
  tests/test_providers.cpp
)
target_link_libraries(test_providers PRIVATE lost_signal_sim_providers lost_signal_sim_eval)
add_test(NAME providers COMMAND test_providers)

add_executable(test_motor
  tests/test_motor.cpp
)
target_link_libraries(test_motor PRIVATE lost_signal_sim_eval)
add_test(NAME motor COMMAND test_motor)

add_executable(test_contact
  tests/test_contact.cpp
)
target_link_libraries(test_contact PRIVATE lost_signal_sim_eval)
add_test(NAME contact COMMAND test_contact)

add_executable(test_encoder
  tests/test_encoder.cpp
)
target_link_libraries(test_encoder PRIVATE
  lost_signal_sim_providers
  lost_signal_robot_platform
)
add_test(NAME encoder COMMAND test_encoder)

add_executable(test_actor_behavior
  tests/test_actor_behavior.cpp
)
target_link_libraries(test_actor_behavior PRIVATE lost_signal_sim_scenario)
add_test(NAME actor_behavior COMMAND test_actor_behavior)

add_executable(test_scenario
  tests/test_scenario.cpp
)
target_link_libraries(test_scenario PRIVATE lost_signal_sim_scenario)
add_test(NAME scenario COMMAND test_scenario)

add_executable(test_loop_closure
  tests/test_loop_closure.cpp
)
target_link_libraries(test_loop_closure PRIVATE lost_signal_sim_scenario)
add_test(NAME loop_closure COMMAND test_loop_closure)

add_executable(test_map_package
  tests/test_map_package.cpp
)
target_link_libraries(test_map_package PRIVATE lost_signal_sim_scenario)
add_test(NAME map_package COMMAND test_map_package)

add_executable(test_realism
  tests/test_realism.cpp
)
target_link_libraries(test_realism PRIVATE lost_signal_sim_scenario)
add_test(NAME realism COMMAND test_realism)

add_executable(test_lidar
  tests/test_lidar.cpp
)
target_link_libraries(test_lidar PRIVATE
  lost_signal_robot_providers
  lost_signal_sim_providers
)
add_test(NAME lidar COMMAND test_lidar)

add_executable(test_lidar3d
  tests/test_lidar3d.cpp
)
target_link_libraries(test_lidar3d PRIVATE
  lost_signal_sim_providers
)
add_test(NAME lidar3d COMMAND test_lidar3d)

add_executable(test_composite_mission
  tests/test_composite_mission.cpp
)
target_compile_definitions(test_composite_mission PRIVATE
  TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_composite_mission PRIVATE lost_signal_sim_scenario)
add_test(NAME composite_mission COMMAND test_composite_mission)

add_executable(test_sim_robot
  tests/test_sim_robot.cpp
)
target_link_libraries(test_sim_robot PRIVATE lost_signal_sim_scenario)
add_test(NAME sim_robot COMMAND test_sim_robot)

add_executable(test_viewer_frame
  tests/test_viewer_frame.cpp
)
target_link_libraries(test_viewer_frame PRIVATE lost_signal_sim_viewer)
add_test(NAME viewer_frame COMMAND test_viewer_frame)

add_executable(test_map_editor
  tests/test_map_editor.cpp
)
target_link_libraries(test_map_editor PRIVATE lost_signal_sim_viewer)
add_test(NAME map_editor COMMAND test_map_editor)

add_executable(test_terrain
  tests/test_terrain.cpp
)
target_compile_definitions(test_terrain PRIVATE
  TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_terrain PRIVATE lost_signal_sim_providers lost_signal_sim_eval lost_signal_sim_scenario)
add_test(NAME terrain COMMAND test_terrain)

add_executable(test_terrain_mission
  tests/test_terrain_mission.cpp
)
target_compile_definitions(test_terrain_mission PRIVATE
  TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_terrain_mission PRIVATE lost_signal_sim_scenario)
add_test(NAME terrain_mission COMMAND test_terrain_mission)

add_executable(test_transform_tree
  tests/test_transform_tree.cpp
)
target_link_libraries(test_transform_tree PRIVATE lost_signal_robot_platform)
add_test(NAME transform_tree COMMAND test_transform_tree)

add_executable(test_coordinator_tf
  tests/test_coordinator_tf.cpp
)
target_link_libraries(test_coordinator_tf PRIVATE lost_signal_robot_autonomy)
add_test(NAME coordinator_tf COMMAND test_coordinator_tf)

add_executable(test_robot_config
  tests/test_robot_config.cpp
)
target_compile_definitions(test_robot_config PRIVATE
  TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_robot_config PRIVATE lost_signal_sim_scenario)
add_test(NAME robot_config COMMAND test_robot_config)

add_executable(test_robot_config_native
  tests/test_robot_config_native.cpp
)
target_compile_definitions(test_robot_config_native PRIVATE
  TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_robot_config_native PRIVATE lost_signal_robot_platform)
add_test(NAME robot_config_native COMMAND test_robot_config_native)

add_executable(test_sim_config
  tests/test_sim_config.cpp
)
target_compile_definitions(test_sim_config PRIVATE
  TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_sim_config PRIVATE lost_signal_sim_scenario)
add_test(NAME sim_config COMMAND test_sim_config)

# Boundary gate: robot/ must never include sim/ or link sim libs.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME robot_boundary
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/check_boundary.py
)

