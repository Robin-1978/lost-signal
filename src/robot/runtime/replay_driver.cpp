#include "replay_driver.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace sim::runtime {

ReplayDriver::ReplayDriver(std::vector<ObservationFrame> observations)
    : observations_(std::move(observations)) {
 double previous_time_s = -1.0;
 for (const auto& observation : observations_) {
  std::string reason;
  if (!valid_observation_frame(observation, &reason)) {
   throw std::invalid_argument("invalid replay observation: " + reason);
  }
  if (observation.time_s < previous_time_s) {
   throw std::invalid_argument("replay observations must be ordered by sample time");
  }
  previous_time_s = observation.time_s;
 }
}

void ReplayDriver::reset() {
 next_ = 0;
}

std::size_t ReplayDriver::publish_until(TimePoint now, IPlatformPort& platform) {
 std::size_t published = 0;
 while (next_ < observations_.size() && observations_[next_].time_s <= now.seconds() + 1e-12) {
  if (!platform.publish_observation(observations_[next_])) {
   throw std::runtime_error("runtime channel rejected a valid replay observation");
  }
  ++next_;
  ++published;
 }
 return published;
}

} // namespace sim::runtime
