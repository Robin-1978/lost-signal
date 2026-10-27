cmake_minimum_required(VERSION 3.20)
project(stm32_chassis_mcu_reference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(STM32_FIRMWARE "Build with embedded-friendly compiler options" OFF)
option(BUILD_HOST_TEST "Build the deterministic host reference test" ON)

add_library(stm32_chassis_controller STATIC
 chassis_controller.cpp
)
target_include_directories(stm32_chassis_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(stm32_chassis_controller PUBLIC cxx_std_17)
if(STM32_FIRMWARE)
 target_compile_options(stm32_chassis_controller PRIVATE
 -ffreestanding -fno-exceptions -fno-rtti -fno-threadsafe-statics)
 target_compile_definitions(stm32_chassis_controller PUBLIC STM32_FIRMWARE=1)
endif()

if(BUILD_HOST_TEST)
 enable_testing()
 add_executable(stm32_chassis_controller_test host_test.cpp)
 target_link_libraries(stm32_chassis_controller_test PRIVATE
 stm32_chassis_controller)
 add_test(NAME stm32_chassis_controller
 COMMAND stm32_chassis_controller_test)
endif()

# A CubeMX/STM32 project supplies the toolchain file, startup code, HAL,
# CAN/UART transport and the final firmware target. This target intentionally
# stays HAL-free so the same PID and watchdog code is testable on the host.
