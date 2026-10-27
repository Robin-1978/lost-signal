# STM32 sensor bridge reference

This is the sensor half of the real-robot example.  It is intentionally split
from the board HAL:

* the board driver timestamps a completed LiDAR scan, IMU sample, or encoder
  snapshot using the robot monotonic clock;
* `SensorBridge` validates version, boot/session, per-sensor sequence, finite
  values and configured channel counts;
* a Linux/native adapter converts the accepted fixed-width measurement into one
  `ObservationFrame` section, preserving the acquisition timestamp and source
  endpoint.  It must never substitute receive time or publish a simulator
  pose;
* the transport serializes fields explicitly and adds its own framing/CRC.
  The C++ structs are not wire-packed and must not be sent with `memcpy`.

The chassis and sensor cores can be checked independently:

```sh
cmake -S examples/stm32_chassis_mcu -B build/stm32-chassis
cmake --build build/stm32-chassis
ctest --test-dir build/stm32-chassis --output-on-failure

cmake -S examples/stm32_sensor_bridge -B build/stm32-sensors
cmake --build build/stm32-sensors
ctest --test-dir build/stm32-sensors --output-on-failure
```

For a real board, a CubeMX project supplies the toolchain/startup, timer,
DMA/interrupt handlers and CAN/UART serializer, then links these HAL-free
libraries with `-DSTM32_FIRMWARE=ON`.  The reference is therefore directly
usable as the control/validation core, while the board-specific pinout,
clock, transport and safety IO still have to be filled from the actual STM32
board profile.  Without those measured profile values the firmware must stay
in safe hold; there is no guessed universal motor or sensor configuration.
