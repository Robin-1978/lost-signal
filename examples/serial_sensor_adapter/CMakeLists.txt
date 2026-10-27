cmake_minimum_required(VERSION 3.20)
project(serial_sensor_adapter_reference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_HOST_TEST "Build the deterministic serial decoder test" ON)

add_library(serial_sensor_adapter STATIC
 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/robot/transport/serial_sensor_protocol.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/robot/transport/serial_observation_adapter.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/robot/transport/posix_serial_port.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/robot/transport/ring_buffer.cpp
)
target_include_directories(serial_sensor_adapter PUBLIC
 ${CMAKE_CURRENT_SOURCE_DIR}
 ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_compile_features(serial_sensor_adapter PUBLIC cxx_std_17)

if(BUILD_HOST_TEST)
 enable_testing()
 add_executable(serial_sensor_adapter_test host_test.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/robot/iface/iface.cpp)
 target_link_libraries(serial_sensor_adapter_test PRIVATE serial_sensor_adapter)
 add_test(NAME serial_sensor_adapter COMMAND serial_sensor_adapter_test)
endif()
