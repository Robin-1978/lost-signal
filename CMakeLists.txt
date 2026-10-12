cmake_minimum_required(VERSION 3.16)
project(robot_simu CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- core: pure C++, no graphics ----
add_library(simcore STATIC
  core/grid_map.cpp
  core/bicycle.cpp
  core/diff_drive.cpp
  core/pid.cpp
  core/astar.cpp
  core/dwa.cpp
  core/sensor.cpp
)
target_include_directories(simcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ---- task layer: no graphics, depends on core ----
add_library(simtask STATIC game/tasks.cpp game/playground.cpp game/campaign.cpp)
target_link_libraries(simtask PUBLIC simcore)

# ---- headless tests (no SFML, CI-able) ----
add_executable(test_core tests/test_core.cpp)
target_link_libraries(test_core PRIVATE simcore)
add_executable(test_tasks tests/test_tasks.cpp)
target_link_libraries(test_tasks PRIVATE simtask)
add_executable(test_playground tests/test_playground.cpp)
target_link_libraries(test_playground PRIVATE simtask)
add_executable(test_campaign tests/test_campaign.cpp)
target_link_libraries(test_campaign PRIVATE simtask sfml-system)
add_executable(test_playthrough tests/test_playthrough.cpp)
target_link_libraries(test_playthrough PRIVATE simtask)

# ---- game view (SFML only here) ----
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
add_executable(simu_app
  app/main.cpp
  view/sfml_view.cpp
  view/tech_panel.cpp
  view/campaign_panel.cpp
)
target_link_libraries(simu_app PRIVATE simtask simcore sfml-graphics sfml-window sfml-system)
