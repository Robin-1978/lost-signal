#include "robot/autonomy/control/controlled_stop_latch.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

bool ControlledStopLatch::update(bool carrying_stop, double peak_wheel_speed_mps,
 const Config& config) {
 const int required = std::max(1, config.settle_cycles);
 const double release_speed =
 std::isfinite(config.release_speed_mps) && config.release_speed_mps >= 0.0
 ? config.release_speed_mps
 : 0.0;
 const bool settled = std::isfinite(peak_wheel_speed_mps) &&
 peak_wheel_speed_mps >= 0.0 && peak_wheel_speed_mps <= release_speed;
 if (!carrying_stop) {
 settle_cycles_ = 0;
 } else if (settled) {
 settle_cycles_ = std::min(required, settle_cycles_ + 1);
 } else {
 settle_cycles_ = 0;
 }
 return carrying_stop && (settle_cycles_ < required || !settled);
}

} // namespace sim
