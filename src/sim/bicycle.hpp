#pragma once
namespace sim {
// Kinematic bicycle + first-order actuator limits. Enough to feel "real"
// vs game NPC steering: speed/steer saturate, steer has lag.
struct BikeState { double x{0},y{0},theta{0},v{0},delta{0}; };
struct BikeCmd { double target_v{0}; double target_delta{0}; };
struct BikeParams { double L{2.0}; double v_max{5.0}; double v_min{-2.0};
  double accel_max{3.0}; double delta_max{0.6}; double steer_rate_max{2.5}; };
class Bicycle {
public:
  explicit Bicycle(BikeParams p={}) : p_(p) {}
  BikeState step(const BikeState& s, const BikeCmd& cmd, double dt);
  const BikeParams& params() const { return p_; }
private:
  BikeParams p_;
};
}
