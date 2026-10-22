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
)
target_include_directories(lost_signal_robot_iface PUBLIC src)
target_link_libraries(lost_signal_robot_iface PUBLIC lost_signal_robot_core)

add_library(lost_signal_robot_platform STATIC
  src/robot/platform/description.cpp
)
target_include_directories(lost_signal_robot_platform PUBLIC src)
target_link_libraries(lost_signal_robot_platform PUBLIC lost_signal_robot_iface)

add_library(lost_signal_robot_autonomy STATIC
  src/robot/autonomy/coordinator.cpp
)
target_include_directories(lost_signal_robot_autonomy PUBLIC src)
target_link_libraries(lost_signal_robot_autonomy PUBLIC lost_signal_robot_iface)

add_library(lost_signal_robot_executive STATIC
  src/robot/executive/executive.cpp
)
target_include_directories(lost_signal_robot_executive PUBLIC src)
target_link_libraries(lost_signal_robot_executive PUBLIC lost_signal_robot_iface)

add_library(lost_signal_robot_app STATIC
  src/robot/app/robot_application.cpp
)
target_include_directories(lost_signal_robot_app PUBLIC src)
target_link_libraries(lost_signal_robot_app PUBLIC
  lost_signal_robot_autonomy
  lost_signal_robot_executive
  lost_signal_robot_platform
)

add_library(lost_signal_robot_stack INTERFACE)
target_link_libraries(lost_signal_robot_stack INTERFACE
  lost_signal_robot_app
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
  src/sim/providers/sim_platform.cpp
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

add_library(lost_signal_sim_game STATIC
  src/sim/game/scenario_session.cpp
)
target_include_directories(lost_signal_sim_game PUBLIC src)
target_link_libraries(lost_signal_sim_game PUBLIC
  lost_signal_robot_stack
  lost_signal_sim_providers
  lost_signal_sim_eval
)

# Sim binary: the only target allowed to link sim + rendering/scoring later.
add_executable(lost_signal_sim
  src/sim/game/sim_main.cpp
)
target_link_libraries(lost_signal_sim PRIVATE lost_signal_sim_game)

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
target_link_libraries(test_truth_isolation PRIVATE lost_signal_sim_game)
add_test(NAME truth_isolation COMMAND test_truth_isolation)

# Boundary gate: robot/ must never include sim/ or link sim libs.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME robot_boundary
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/check_boundary.py
)
