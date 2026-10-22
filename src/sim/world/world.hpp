#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lost_signal::sim::world {

// SIM-owned ground truth. Must never be included from src/robot/.
// Physics, actors, contacts, and scoring inputs live here.
struct WorldTruth {
  double body_x_m = 0.0;
  double body_y_m = 0.0;
  double body_yaw_rad = 0.0;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;  // derived from wheel speeds, like a driver
  // Per-wheel joint truth (mechanical rad/s). The plant integrates these;
  // providers publish them as measured feedback.
  double left_radps = 0.0;
  double right_radps = 0.0;
  double duty_left = 0.0;
  double duty_right = 0.0;
  std::uint64_t step = 0;
};

// Authored scenario geometry (static) and moving actors (dynamic).
// Raycast sources, never ROBOT inputs: the suite turns them into measured
// scans (full raycast) and a static reference (static geometry only).
struct StaticBox {
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  double height_m = 0.0;  // surveyed deck height; ground is 0
};

struct Actor {
  double x = 0.0, y = 0.0, vx_mps = 0.0, vy_mps = 0.0, radius_m = 0.4;
};

// Authored named place: semantic task target resolved by the ROBOT map
// provider, never a raw coordinate crossing from SIM.
struct NamedPlace {
  std::string place_id;
  double x_m = 0.0, y_m = 0.0, yaw_rad = 0.0;
};

struct MotorParams {
  double free_speed_mps = 3.0;   // no-load linear speed at duty 1
  double time_constant_s = 0.12;  // first-order lag to commanded speed
  double wheel_radius_m = 0.1;
  double track_width_m = 0.5;
};

class World {
 public:
  void step(double dt_s);
  void set_motor_params(const MotorParams& params) { motor_ = params; }
  // Actuator-level plant input: normalized duties. Wheel speeds first-order
  // track duty-implied targets; the body follows by differential kinematics.
  void apply_duty(double left, double right) {
    truth_.duty_left = left;
    truth_.duty_right = right;
  }
  const WorldTruth& truth() const { return truth_; }

  void set_static_boxes(std::vector<StaticBox> boxes) {
    static_boxes_ = std::move(boxes);
    ++geometry_version_;
  }
  const std::vector<StaticBox>& static_boxes() const { return static_boxes_; }
  std::uint64_t geometry_version() const { return geometry_version_; }
  void add_actor(const Actor& actor) { actors_.push_back(actor); }
  const std::vector<Actor>& actors() const { return actors_; }
  void add_place(const NamedPlace& place) {
    places_.push_back(place);
    ++geometry_version_;
  }
  const std::vector<NamedPlace>& places() const { return places_; }

 private:
  WorldTruth truth_{};
  MotorParams motor_{};
  std::vector<StaticBox> static_boxes_;
  std::vector<Actor> actors_;
  std::vector<NamedPlace> places_;
  std::uint64_t geometry_version_ = 0;
};

}  // namespace lost_signal::sim::world
