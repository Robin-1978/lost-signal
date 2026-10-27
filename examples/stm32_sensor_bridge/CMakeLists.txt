cmake_minimum_required(VERSION 3.20)
project(stm32_sensor_bridge_reference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(STM32_FIRMWARE "Build with embedded-friendly compiler options" OFF)
option(BUILD_HOST_TEST "Build the deterministic host reference test" ON)

add_library(stm32_sensor_bridge STATIC sensor_bridge.cpp)
target_include_directories(stm32_sensor_bridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(stm32_sensor_bridge PUBLIC cxx_std_17)
if(STM32_FIRMWARE)
 target_compile_options(stm32_sensor_bridge PRIVATE
 -ffreestanding -fno-exceptions -fno-rtti -fno-threadsafe-statics)
 target_compile_definitions(stm32_sensor_bridge PUBLIC STM32_FIRMWARE=1)
endif()

if(BUILD_HOST_TEST)
 enable_testing()
 add_executable(stm32_sensor_bridge_test host_test.cpp)
 target_include_directories(stm32_sensor_bridge_test PRIVATE
 ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
 target_link_libraries(stm32_sensor_bridge_test PRIVATE stm32_sensor_bridge)
 target_sources(stm32_sensor_bridge_test PRIVATE
 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/robot/iface/iface.cpp)
 add_test(NAME stm32_sensor_bridge COMMAND stm32_sensor_bridge_test)
endif()

# A board project supplies the STM32 toolchain, startup code, DMA/interrupt
# handlers and a field-by-field CAN/UART serializer.  The core above remains
# the exact code compiled in that firmware target and is tested on the host.
