cmake_minimum_required(VERSION 3.20)
project(lost_signal_refactor VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Robot stack: deployable, must never depend on sim.
add_library(lost_signal_robot_core STATIC
  src/robot/core/version.cpp
)
target_include_directories(lost_signal_robot_core PUBLIC src)

add_library(lost_signal_robot_iface STATIC
  src/robot/iface/iface.cpp
  src/robot/iface/endpoint.cpp
  src/robot/iface/channel.cpp
  src/robot/iface/message.cpp
)
target_include_directories(lost_signal_robot_iface PUBLIC src)
target_link_libraries(lost_signal_robot_iface PUBLIC lost_signal_robot_core)

add_library(lost_signal_robot_platform STATIC
  src/robot/platform/description.cpp
  src/robot/platform/chassis.cpp
)
target_include_directories(lost_signal_robot_platform PUBLIC src)
target_link_libraries(lost_signal_robot_platform PUBLIC lost_signal_robot_iface)

add_library(lost_signal_robot_lifecycle STATIC
  src/robot/lifecycle/bringup.cpp
)
target_include_directories(lost_signal_robot_lifecycle PUBLIC src)
target_link_libraries(lost_signal_robot_lifecycle PUBLIC lost_signal_robot_core)

add_library(lost_signal_robot_autonomy STATIC
  src/robot/autonomy/coordinator.cpp
  src/robot/autonomy/behavior/behavior.cpp
  src/robot/autonomy/estimation/estimator.cpp
  src/robot/autonomy/planning/global_planner.cpp
  src/robot/autonomy/safety/safety.cpp
)
target_include_directories(lost_signal_robot_autonomy PUBLIC src)
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
)

# Native robot binary: provider-neutral, links only the robot stack.
add_executable(lost_signal_robot
  src/robot/app/main.cpp
)
target_link_libraries(lost_signal_robot PRIVATE lost_signal_robot_stack)

# Sim side: owns world truth, physics, presentation. May depend on robot.
add_library(lost_signal_sim_world STATIC
  src/sim/world/world.cpp
)
target_include_directories(lost_signal_sim_world PUBLIC src)
target_link_libraries(lost_signal_sim_world PUBLIC lost_signal_robot_core)

add_library(lost_signal_sim_providers STATIC
  src/sim/providers/sim_motion.cpp
  src/sim/providers/sim_imu.cpp
  src/sim/providers/lidar2d.cpp
  src/sim/providers/lidar16.cpp
  src/sim/providers/sensor_suite.cpp
)
target_include_directories(lost_signal_sim_providers PUBLIC src)
target_link_libraries(lost_signal_sim_providers PUBLIC
  lost_signal_robot_stack
  lost_signal_sim_world
)

add_library(lost_signal_sim_eval STATIC
  src/sim/eval/scoring.cpp
)
target_include_directories(lost_signal_sim_eval PUBLIC src)
target_link_libraries(lost_signal_sim_eval PUBLIC
  lost_signal_robot_stack
  lost_signal_sim_world
)

add_library(lost_signal_sim_scenario STATIC
  src/sim/scenario/scenario_session.cpp
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
target_link_libraries(test_providers PRIVATE lost_signal_sim_providers)
add_test(NAME providers COMMAND test_providers)

add_executable(test_lidar
  tests/test_lidar.cpp
)
target_link_libraries(test_lidar PRIVATE
  lost_signal_robot_providers
  lost_signal_sim_providers
)
add_test(NAME lidar COMMAND test_lidar)

add_executable(test_sim_robot
  tests/test_sim_robot.cpp
)
target_link_libraries(test_sim_robot PRIVATE lost_signal_sim_scenario)
add_test(NAME sim_robot COMMAND test_sim_robot)

# Boundary gate: robot/ must never include sim/ or link sim libs.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME robot_boundary
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/check_boundary.py
)
