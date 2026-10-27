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
 src/robot/iface/motion_contract.cpp
  src/robot/iface/endpoint.cpp
  src/robot/iface/channel.cpp
  src/robot/iface/message.cpp
  src/robot/iface/conformance.cpp
 src/robot/transport/ring_buffer.cpp
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
 src/robot/autonomy/decision/autonomy_pipeline.cpp
 src/robot/autonomy/decision/goal_arrival.cpp
 src/robot/autonomy/decision/motion_pipeline.cpp
 src/robot/autonomy/decision/terrain_pipeline.cpp
  src/robot/autonomy/behavior/behavior.cpp
  src/robot/autonomy/estimation/estimator.cpp
 src/robot/autonomy/estimation/measurement_synchronizer.cpp
 src/robot/autonomy/estimation/observation_pipeline.cpp
  src/robot/autonomy/estimation/slip_observer.cpp
  src/robot/autonomy/localization/scan_matcher.cpp
  src/robot/autonomy/localization/global_search.cpp
  src/robot/autonomy/localization/localization_system.cpp
  src/robot/autonomy/localization/supervisor.cpp
  src/robot/autonomy/mapping/mapper.cpp
  src/robot/autonomy/mapping/pose_graph.cpp
  src/robot/autonomy/mapping/loop_closure.cpp
  src/robot/autonomy/mapping/slam.cpp
  src/robot/autonomy/perception/lidar_perception.cpp
  src/robot/autonomy/planning/astar_planner.cpp
 src/robot/autonomy/planning/ackermann_planner.cpp
 src/robot/autonomy/planning/route_smoothing.cpp
  src/robot/autonomy/planning/dwa_planner.cpp
  src/robot/autonomy/planning/global_planner.cpp
 src/robot/autonomy/planning/local_planner.cpp
 src/robot/autonomy/planning/route_pipeline.cpp
 src/robot/autonomy/planning/teb_planner.cpp
  src/robot/autonomy/planning/prediction.cpp
 src/robot/autonomy/planning/prediction_queries.cpp
  src/robot/autonomy/planning/task/task_ordering.cpp
  src/robot/autonomy/control/stop_latch.cpp
  src/robot/autonomy/diagnostics/decision_evidence.cpp
 src/robot/autonomy/diagnostics/trajectory_tracking.cpp
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
 src/robot/app/native_runtime.cpp
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
 # GCC LTO objects linked through lld can produce a tiny shared library that
 # exports only __gnu_lto_slim. Keep release builds portable across the
 # toolchains used by the simulator and let normal -O3 optimize MuJoCo.
 set(MUJOCO_ENABLE_LTO OFF CACHE BOOL "" FORCE)
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
 src/sim/viewer/viewer_audio.cpp
  )
  target_link_libraries(sim_viewer PRIVATE lost_signal_sim_viewer
    lost_signal_sim_scenario raylib)

 add_executable(test_viewer_audio
 tests/test_viewer_audio.cpp
 src/sim/viewer/viewer_audio.cpp
 )
 target_link_libraries(test_viewer_audio PRIVATE lost_signal_sim_viewer raylib)
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
  target_sources(lost_signal_sim_world PRIVATE src/sim/world/mujoco_build.cpp)
  target_sources(lost_signal_sim_world PRIVATE src/sim/world/mujoco_step.cpp)
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
# Replay providers serialize through the eval recording schema. Declare that
# real dependency so static-link order remains correct for every consumer.
target_link_libraries(lost_signal_sim_providers PUBLIC
 lost_signal_sim_eval
)

add_library(lost_signal_sim_scenario STATIC
  src/sim/scenario/scenario_session.cpp
 src/sim/scenario/virtual_mcu.cpp
  src/sim/scenario/collision_judge.cpp
  src/sim/scenario/objectives.cpp
 src/sim/scenario/actor_sidecar.cpp
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
  src/sim/viewer/asset_manifest.cpp
 src/sim/viewer/actor_render_policy.cpp
 src/sim/viewer/collision_geom_wireframe.cpp
 src/sim/viewer/experience_flow.cpp
 src/sim/viewer/experience_manifest.cpp
  src/sim/viewer/frame_builder.cpp
 src/sim/viewer/keyframe_review.cpp
  src/sim/viewer/map_editor.cpp
 src/sim/viewer/particle_system.cpp
 src/sim/viewer/physical_audio.cpp
 src/sim/viewer/playback_controller.cpp
 src/sim/viewer/scene_manifest.cpp
)
target_include_directories(lost_signal_sim_viewer PUBLIC src)
target_link_libraries(lost_signal_sim_viewer PUBLIC
  lost_signal_sim_eval
  lost_signal_sim_world
)

add_library(lost_signal_sim_experiment STATIC
 src/sim/experiment/ab_experiment.cpp
 src/sim/experiment/episode_pair.cpp
 src/sim/experiment/experience_runtime.cpp
)
set(LOST_SIGNAL_BUILD_REVISION_HEADER
 ${CMAKE_CURRENT_BINARY_DIR}/generated/lost_signal/build_revision.hpp)
add_custom_target(lost_signal_build_revision
 COMMAND ${CMAKE_COMMAND}
 -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
 -DOUTPUT_FILE=${LOST_SIGNAL_BUILD_REVISION_HEADER}
 -P ${CMAKE_SOURCE_DIR}/cmake/write_build_revision.cmake
 BYPRODUCTS ${LOST_SIGNAL_BUILD_REVISION_HEADER}
 VERBATIM)
add_dependencies(lost_signal_sim_experiment lost_signal_build_revision)
target_include_directories(lost_signal_sim_experiment
 PUBLIC src
 PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(lost_signal_sim_experiment PUBLIC
 lost_signal_sim_scenario lost_signal_sim_viewer)

if(TARGET sim_viewer)
 target_link_libraries(sim_viewer PRIVATE lost_signal_sim_experiment)
endif()

add_executable(sim_ab_experiment
 src/sim/experiment/ab_experiment_main.cpp
)
target_link_libraries(sim_ab_experiment PRIVATE lost_signal_sim_experiment)

# Tests
enable_testing()

if(TARGET test_viewer_audio)
 add_test(NAME viewer_audio COMMAND test_viewer_audio)
endif()

add_executable(test_robot_cycle
  tests/test_robot_cycle.cpp
)
target_link_libraries(test_robot_cycle PRIVATE lost_signal_robot_stack)
add_test(NAME robot_cycle COMMAND test_robot_cycle)

add_executable(test_native_runtime
 tests/test_native_runtime.cpp
)
target_link_libraries(test_native_runtime PRIVATE lost_signal_robot_stack)
add_test(NAME native_runtime COMMAND test_native_runtime)

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

add_executable(test_measurement_synchronizer
 tests/test_measurement_synchronizer.cpp
)
target_link_libraries(test_measurement_synchronizer PRIVATE
 lost_signal_robot_autonomy
)
add_test(NAME measurement_synchronizer COMMAND test_measurement_synchronizer)

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

add_executable(test_traversal
  tests/test_traversal.cpp
)
target_link_libraries(test_traversal PRIVATE lost_signal_robot_autonomy)
add_test(NAME traversal COMMAND test_traversal)

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
target_compile_definitions(test_sim_actor PRIVATE
 TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_sim_actor PRIVATE lost_signal_sim_scenario)
add_test(NAME sim_actor COMMAND test_sim_actor)

add_executable(test_astar
  tests/test_astar.cpp
)
target_link_libraries(test_astar PRIVATE lost_signal_robot_autonomy)
add_test(NAME astar COMMAND test_astar)

add_executable(test_industrial_200k
 tests/test_industrial_200k.cpp
)
target_compile_definitions(test_industrial_200k PRIVATE
 TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_industrial_200k PRIVATE lost_signal_sim_scenario)
add_test(NAME industrial_200k COMMAND test_industrial_200k)

add_executable(test_industrial_200k_mission
 tests/test_industrial_200k_mission.cpp
)
target_compile_definitions(test_industrial_200k_mission PRIVATE
 TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_industrial_200k_mission PRIVATE
 lost_signal_sim_scenario)
add_test(NAME industrial_200k_mission COMMAND test_industrial_200k_mission)
set_tests_properties(industrial_200k_mission PROPERTIES
 LABELS "long;physics"
 RUN_SERIAL TRUE
 TIMEOUT 1200)
add_test(NAME industrial_200k_direct_mission
 COMMAND test_industrial_200k_mission central_gate)
set_tests_properties(industrial_200k_direct_mission PROPERTIES
 LABELS "long;physics"
 RUN_SERIAL TRUE
 TIMEOUT 300)

add_executable(test_ackermann_planner
 tests/test_ackermann_planner.cpp
)
target_link_libraries(test_ackermann_planner PRIVATE lost_signal_robot_autonomy)
add_test(NAME ackermann_planner COMMAND test_ackermann_planner)



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

add_executable(test_motion_contract
 tests/test_motion_contract.cpp
)
target_link_libraries(test_motion_contract PRIVATE lost_signal_robot_iface)
add_test(NAME motion_contract COMMAND test_motion_contract)

add_executable(test_dwa
  tests/test_dwa.cpp
)
target_link_libraries(test_dwa PRIVATE lost_signal_robot_autonomy)
add_test(NAME dwa COMMAND test_dwa)

add_executable(test_local_planner_compare
 tests/test_local_planner_compare.cpp
)
target_link_libraries(test_local_planner_compare PRIVATE lost_signal_robot_autonomy)
add_test(NAME local_planner_compare COMMAND test_local_planner_compare)

add_executable(test_teb
 tests/test_teb.cpp
)
target_link_libraries(test_teb PRIVATE lost_signal_robot_autonomy)
add_test(NAME teb COMMAND test_teb)

add_executable(test_dynamic_escape
 tests/test_dynamic_escape.cpp
)
target_link_libraries(test_dynamic_escape PRIVATE lost_signal_robot_autonomy)
add_test(NAME dynamic_escape COMMAND test_dynamic_escape)

add_executable(test_dynamic_crossing_mission
 tests/test_dynamic_crossing_mission.cpp
)
target_compile_definitions(test_dynamic_crossing_mission PRIVATE
 TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/configs"
)
target_link_libraries(test_dynamic_crossing_mission PRIVATE
 lost_signal_sim_scenario
 lost_signal_sim_eval
)
add_test(NAME dynamic_crossing_mission COMMAND test_dynamic_crossing_mission)

add_executable(test_motion_pipeline
 tests/test_motion_pipeline.cpp
)
target_link_libraries(test_motion_pipeline PRIVATE lost_signal_robot_autonomy)
add_test(NAME motion_pipeline COMMAND test_motion_pipeline)

add_executable(test_trajectory_tracking
 tests/test_trajectory_tracking.cpp
)
target_link_libraries(test_trajectory_tracking PRIVATE lost_signal_robot_autonomy)
add_test(NAME trajectory_tracking COMMAND test_trajectory_tracking)

add_executable(test_prediction
 tests/test_prediction.cpp
)
target_link_libraries(test_prediction PRIVATE lost_signal_robot_autonomy)
add_test(NAME prediction COMMAND test_prediction)

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

add_executable(test_virtual_mcu
 tests/test_virtual_mcu.cpp
)
target_link_libraries(test_virtual_mcu PRIVATE lost_signal_sim_scenario)
add_test(NAME virtual_mcu COMMAND test_virtual_mcu)

add_executable(test_ring_buffer
 tests/test_ring_buffer.cpp
)
target_link_libraries(test_ring_buffer PRIVATE lost_signal_robot_iface)
add_test(NAME ring_buffer COMMAND test_ring_buffer)

add_test(NAME signal_town_mission
 COMMAND lost_signal_sim
 --robot ${CMAKE_SOURCE_DIR}/configs/robot/differential.json
 --sim ${CMAKE_SOURCE_DIR}/configs/sim/default.json
 --map ${CMAKE_SOURCE_DIR}/configs/worlds/signal_town.json
 --objectives energy_hub,freight_yard,housing_door,market_gate,town_gate
 --steps 8000 --require-pass)
set_tests_properties(signal_town_mission PROPERTIES
 LABELS "long;physics"
 RUN_SERIAL TRUE
 TIMEOUT 300)

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

add_executable(test_collision_geom_wireframe
 tests/test_collision_geom_wireframe.cpp
)
target_link_libraries(test_collision_geom_wireframe PRIVATE
 lost_signal_sim_viewer
)
add_test(NAME collision_geom_wireframe COMMAND test_collision_geom_wireframe)

add_executable(test_actor_render_policy
 tests/test_actor_render_policy.cpp
)
target_link_libraries(test_actor_render_policy PRIVATE lost_signal_sim_viewer)
add_test(NAME actor_render_policy COMMAND test_actor_render_policy)

add_executable(test_asset_manifest
  tests/test_asset_manifest.cpp
)
target_compile_definitions(test_asset_manifest PRIVATE
  TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_asset_manifest PRIVATE lost_signal_sim_viewer)
add_test(NAME asset_manifest COMMAND test_asset_manifest)

add_executable(test_scene_manifest
 tests/test_scene_manifest.cpp
)
target_compile_definitions(test_scene_manifest PRIVATE
 TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_scene_manifest PRIVATE lost_signal_sim_viewer)
add_test(NAME scene_manifest COMMAND test_scene_manifest)

add_executable(test_world_place_clearance
  tests/test_world_place_clearance.cpp
)
target_compile_definitions(test_world_place_clearance PRIVATE
  TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/configs"
)
target_link_libraries(test_world_place_clearance PRIVATE lost_signal_sim_eval)
add_test(NAME world_place_clearance COMMAND test_world_place_clearance)

add_executable(test_experience_manifest
 tests/test_experience_manifest.cpp
)
target_compile_definitions(test_experience_manifest PRIVATE
 TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_experience_manifest PRIVATE lost_signal_sim_viewer)
add_test(NAME experience_manifest COMMAND test_experience_manifest)

add_executable(test_experience_flow
 tests/test_experience_flow.cpp
)
target_link_libraries(test_experience_flow PRIVATE lost_signal_sim_viewer)
add_test(NAME experience_flow COMMAND test_experience_flow)

add_executable(test_experience_mission
 tests/test_experience_mission.cpp
)
target_compile_definitions(test_experience_mission PRIVATE
 TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_experience_mission PRIVATE
 lost_signal_sim_viewer lost_signal_sim_scenario)
add_test(NAME experience_mission COMMAND test_experience_mission)

add_executable(test_safety_experience_mission
 tests/test_safety_experience_mission.cpp
)
target_compile_definitions(test_safety_experience_mission PRIVATE
 TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_safety_experience_mission PRIVATE
 lost_signal_sim_viewer lost_signal_sim_scenario)
add_test(NAME safety_experience_mission COMMAND test_safety_experience_mission)

add_executable(test_slam_experience_mission
 tests/test_slam_experience_mission.cpp
)
target_compile_definitions(test_slam_experience_mission PRIVATE
 TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_slam_experience_mission PRIVATE
 lost_signal_sim_experiment
)
add_test(NAME slam_experience_mission COMMAND test_slam_experience_mission)

add_executable(test_keyframe_review
 tests/test_keyframe_review.cpp
)
target_link_libraries(test_keyframe_review PRIVATE lost_signal_sim_viewer)
add_test(NAME keyframe_review COMMAND test_keyframe_review)

add_executable(test_ab_experiment
 tests/test_ab_experiment.cpp
)
target_compile_definitions(test_ab_experiment PRIVATE
 TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_ab_experiment PRIVATE lost_signal_sim_experiment)
add_test(NAME ab_experiment COMMAND test_ab_experiment)

add_executable(test_experience_runtime
 tests/test_experience_runtime.cpp
)
target_compile_definitions(test_experience_runtime PRIVATE
 TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_experience_runtime PRIVATE
 lost_signal_sim_experiment)
add_test(NAME experience_runtime COMMAND test_experience_runtime)

add_executable(test_playback_controller
 tests/test_playback_controller.cpp
)
target_link_libraries(test_playback_controller PRIVATE
 lost_signal_sim_viewer lost_signal_sim_scenario)
add_test(NAME playback_controller COMMAND test_playback_controller)

add_executable(test_particle_system
 tests/test_particle_system.cpp
)
target_link_libraries(test_particle_system PRIVATE lost_signal_sim_viewer)
add_test(NAME particle_system COMMAND test_particle_system)

add_executable(test_physical_audio
 tests/test_physical_audio.cpp
)
target_link_libraries(test_physical_audio PRIVATE lost_signal_sim_viewer)
add_test(NAME physical_audio COMMAND test_physical_audio)

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

add_executable(test_multichassis_mission
 tests/test_multichassis_mission.cpp
)
target_compile_definitions(test_multichassis_mission PRIVATE
 TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/configs"
)
target_link_libraries(test_multichassis_mission PRIVATE lost_signal_sim_scenario)
add_test(NAME multichassis_mission COMMAND test_multichassis_mission)

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

add_executable(test_artifact
 tests/test_artifact.cpp
)
target_link_libraries(test_artifact PRIVATE lost_signal_sim_scenario
 lost_signal_sim_providers lost_signal_sim_eval)
add_test(NAME artifact COMMAND test_artifact)

# Boundary gate: robot/ must never include sim/ or link sim libs.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME convert_master_map
 COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/test_convert_master_map.py
)
add_test(NAME robot_boundary
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/check_boundary.py
)

# The native tests use assert() as their check primitive, and several tests
# intentionally place state-changing setup calls inside those expressions.
# Release flags define NDEBUG globally; undo that definition for test
# executables so release validation executes the same checks and setup as a
# debug build. Production libraries and binaries retain their normal release
# flags.
get_property(LOST_SIGNAL_DIRECTORY_TARGETS DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(target IN LISTS LOST_SIGNAL_DIRECTORY_TARGETS)
 if(target MATCHES "^test_")
 if(MSVC)
 target_compile_options(${target} PRIVATE /UNDEBUG)
 else()
 target_compile_options(${target} PRIVATE -UNDEBUG)
 endif()
 endif()
endforeach()

