#pragma once

#include "robot/platform/differential_kinematics.hpp"

namespace sim {

// Lightweight simulation plant used by the planar regression backend. The
// MuJoCo vehicle plant remains authoritative for rigid-body runs.
struct PlanarDiffDriveParams {
 double max_forward_mps{2.0};
 double max_reverse_mps{1.0};
 double max_yaw_rate_radps{2.0};
 double max_acceleration_mps2{3.0};
 double max_yaw_acceleration_radps2{8.0};
 double track_width_m{0.5};
};

class PlanarDiffDrivePlant {
 public:
 explicit PlanarDiffDrivePlant(PlanarDiffDriveParams params = {}) : params_(params) {}

 DiffState step(const DiffState& state, const DiffCmd& command, double dt_s) const;
 const PlanarDiffDriveParams& params() const { return params_; }

 private:
 PlanarDiffDriveParams params_;
};

} // namespace sim
