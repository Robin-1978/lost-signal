#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "robot/iface/observation.hpp"

namespace lost_signal::sim::eval {

// One recorded step, simulator-agnostic: observation in, command out.
// Truth and diagnostics ride along for offline scoring but never enter the
// replayed decision path.
struct RecordedStep {
  std::uint64_t sequence = 0;
  std::int64_t stamp_ns = 0;
  std::string source;
  std::vector<float> ranges_m;
  std::vector<float> static_ranges_m;
  bool scan_valid = false;
  bool scan_static_valid = false;
  double angle_min_rad = 0.0;
  double angle_increment_rad = 0.0;
  bool odom_valid = false;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;
  bool imu_valid = false;
  double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
  double ax = 0.0, ay = 0.0, az = 0.0;
  bool wheels_valid = false;
  double left_radps = 0.0;
  double right_radps = 0.0;
};

// JSONL v1: one header line then one step object per line.
// schema: {"v":1,"map":"<id>","rev":<n>}
struct RecordedEpisode {
  std::string map_id;
  std::uint64_t map_revision = 0;
  std::vector<RecordedStep> steps;
};

// Serialize one step. Floats use %.9g (exact float round-trip) and doubles
// %.17g (exact double round-trip), so import replays bit-identical frames.
std::string step_to_json_line(const RecordedStep& step);
// Parse one step line. Returns false with a reason; never throws.
bool step_from_json_line(const std::string& line, RecordedStep& step,
                         std::string* error);

// Pack a live frame into a recordable step.
RecordedStep record_step(const robot::iface::ObservationFrame& obs);
// Unpack a recorded step back into a live-equivalent frame.
robot::iface::ObservationFrame step_to_observation(const RecordedStep& step);

}  // namespace lost_signal::sim::eval
