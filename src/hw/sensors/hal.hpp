#pragma once

#include "runtime/sensor_messages.hpp"

#include <optional>

namespace sim {

// Raw device-provider boundary. A native driver and a simulated device both
// publish the same timestamped measurement contract. World geometry and
// autonomy algorithms are deliberately absent from this interface.
class LidarBackend {
 public:
 virtual ~LidarBackend() = default;
 virtual std::optional<Scan> read() = 0;
};

} // namespace sim
