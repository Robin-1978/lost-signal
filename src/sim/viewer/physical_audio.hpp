#pragma once

#include <cstdint>

#include "sim/viewer/frame.hpp"

namespace lost_signal::sim::viewer {

// Presentation response times are expressed in simulation seconds. They
// smooth audible gain only; they are not navigation, collision, or scoring
// thresholds.
struct PhysicalAudioConfig {
 double sim_step_s = 0.02;
 double attack_time_s = 0.06;
 double release_time_s = 0.18;
 double impact_decay_time_s = 0.20;
};

// Unitless [0, 1] signals derived from completed physical evidence. The
// reference speed and load come from the configured plant in the same Frame,
// so different chassis do not depend on a scene-specific tuning constant.
struct PhysicalAudioSample {
 std::uint64_t step = 0;
 bool advanced = false;
 bool rewound = false;
 bool impact_event = false;
 double wheel_speed_ratio = 0.0;
 double contact_load_ratio = 0.0;
 double motor_intensity = 0.0;
 double rolling_intensity = 0.0;
 double slip_intensity = 0.0;
 double brake_intensity = 0.0;
 double impact_intensity = 0.0;
};

// Pure, deterministic evidence reducer. It has no raylib or audio-device
// dependency, which makes pause/repeat/rewind/Turbo timelines testable.
class PhysicalAudioEvidence {
 public:
 explicit PhysicalAudioEvidence(PhysicalAudioConfig config = {});

 void reset();
 const PhysicalAudioSample& advance(const Frame& frame);
 const PhysicalAudioSample& sample() const { return sample_; }

 private:
 PhysicalAudioConfig config_;
 PhysicalAudioSample sample_;
 bool has_frame_ = false;
 bool impact_active_ = false;
};

} // namespace lost_signal::sim::viewer
