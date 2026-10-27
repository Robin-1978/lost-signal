# Generic serial sensor adapter reference

This is the host-side serial path for a real robot. It is deliberately not an
STM32 component: a LiDAR, IMU, encoder board, USB CDC device, or STM32 can be
the other endpoint.

`PosixSerialPort` opens `/dev/ttyUSB*` or `/dev/ttyACM*` in non-blocking 8N1
mode. `SerialFrameDecoder` receives arbitrary byte chunks and handles split
frames, concatenated frames, noise, CRC failures, boot/session changes and
out-of-order sequence numbers. A byte ring absorbs short bursts; the decoded
frame is then copied into a complete snapshot before `ObservationAdapter`
publishes it. The adapter preserves the sensor acquisition timestamp and never
uses host receive time as the measurement stamp.

The reference wire frame is little-endian and contains a magic prefix,
protocol version, sensor kind, boot id, sequence, acquisition timestamp,
payload length, payload and CRC32. The sample payloads cover:

* 2D LiDAR: angle origin/increment and range millimetres;
* IMU: three gyro and three acceleration floats;
* chassis joints: drive speed and optional steering angle/rate.

Use a real device profile for source names, frame mounts, channel counts,
range, baud rate and sensor-specific calibration. The sample codec is a
reference transport, not a claim that every vendor's native serial protocol
has this layout.

Host check:

```sh
cmake -S examples/serial_sensor_adapter -B build/serial-sensors
cmake --build build/serial-sensors
ctest --test-dir build/serial-sensors --output-on-failure
```

The normal adapter loop is:

```cpp
std::uint8_t bytes[512];
while (port.is_open()) {
  const int n = port.read(bytes, sizeof(bytes));
  if (n <= 0) continue;
  decoder.feed(bytes, static_cast<std::size_t>(n));
  SerialPacket packet;
  while (decoder.next(packet) == DecodeStatus::kAccepted) {
    ObservationFrame observation;
    if (adapter.make(packet, profile, observation))
      application.publish_observation(observation);
  }
}
```

The board-side STM32 chassis example uses a separate, chassis-owned UART
codec in `examples/stm32_chassis_mcu/serial_codec.*`; sensor transport and
actuator transport may share a physical UART only when the board profile
provides independent framing and bandwidth guarantees.

Other native links live in `src/robot/transport/native_links.*` and share the
same byte/frame boundary:

* `UdpLink` is a bounded POSIX datagram socket for Ethernet sensors and remote
  controllers;
* `SocketCanLink` uses Linux SocketCAN and `CanSegmenter` reassembles bounded
  multi-frame transfers without exposing partial measurements;
* `SpiLink` wraps Linux `spidev` for board-local devices; the device tree, chip
  select and IRQ still belong to the machine profile;
* `UsbCdcLink` aliases the same non-blocking byte stream for `/dev/ttyACM*`.

All four are production transport boundaries under `src`; this directory
contains the serial sensor walkthrough and deterministic tests only.
