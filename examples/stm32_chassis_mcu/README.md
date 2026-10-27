# STM32 chassis control reference

PNC on the host produces a body intent (`v`, `w`) or a joint target. The MCU
owns the fixed-period loop: it checks `sequence` and `expiry`, converts body
twist to differential wheel targets, runs one PID per driven joint with
conditional anti-windup, clamps duty, and applies a local brake on watchdog
expiry. Encoder feedback is the only closed-loop measurement.

`ChassisConfig::owner` is part of the board profile. A body-loop board accepts
`kBodyTwist`, a joint-loop board accepts `kJointVelocity`, and a host-loop board
accepts `kDuty`; an ownership mismatch is rejected before PID state changes.

The reference is HAL-free and uses fixed-width wire DTOs. A CubeMX project
should keep `chassis_controller.cpp` and `serial_codec.cpp` in the firmware
target. `serial_codec.cpp` is a complete UART framing example: explicit
little-endian fields, version, message kind, payload length and CRC32, with a
split-read/粘包-safe parser. The same codec can be replaced by CAN or Ethernet
framing without changing the controller.

The host adapter maps the repository `ChassisSetpoint` to `WireSetpoint`; it
never sends SIM truth or PNC internals. `WireSetpoint` is a field contract, not
a structure to transmit with raw `memcpy`.

Host check:

```sh
cmake -S examples/stm32_chassis_mcu -B build/stm32-chassis
cmake --build build/stm32-chassis
ctest --test-dir build/stm32-chassis --output-on-failure
```

For an STM32 toolchain, pass the CubeMX-generated toolchain file and
`-DSTM32_FIRMWARE=ON -DBUILD_HOST_TEST=OFF`. The board timer calls
`ChassisController::tick()` at the configured `control_period_ns`; it must not
derive `dt` from wall-clock calls inside the controller.

This is the firmware core and protocol example, not a board-independent binary.
To flash a real robot, the CubeMX project still supplies startup/linker code,
clock and UART DMA/interrupt callbacks, PWM/encoder/estop drivers, a measured
`ChassisConfig`, and a hardware watchdog. The controller enters safe hold until
the board profile and motor direction/encoder polarity have passed bring-up.
