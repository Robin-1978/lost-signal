#pragma once

#include "runtime_channel.hpp"

#include <cstddef>
#include <vector>

namespace sim::runtime {

// Transport-independent replay producer. It drives the platform-facing port,
// so algorithms receive recorded observations through the same channel used
// by simulation and future native hardware adapters.
class ReplayDriver {
 public:
 explicit ReplayDriver(std::vector<ObservationFrame> observations);

 void reset();
 std::size_t publish_until(TimePoint now, IPlatformPort& platform);
 bool finished() const { return next_ == observations_.size(); }
 std::size_t size() const { return observations_.size(); }

 private:
 std::vector<ObservationFrame> observations_;
 std::size_t next_{0};
};

} // namespace sim::runtime
