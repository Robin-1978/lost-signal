#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "robot/iface/motion.hpp"

namespace lost_signal::sim::world {

// SIM-owned ground truth. Must never be included from src/robot/.
// Physics, actors, contacts, and scoring inputs live here.
//
// Joint layout mirrors the actuator mode that produced it: diff 2 drives,
// ackermann 2 drives + 2 steers, swerve 4 + 4, mecanum 4 drives.
// drive_count == 0 means the plant has not yet received a command.
struct WorldTruth {
  double body_x_m = 0.0;
  double body_y_m = 0.0;
  double body_yaw_rad = 0.0;
  double forward_mps = 0.0;
  double lateral_mps = 0.0;   // holonomic body-left velocity
  double yaw_rate_radps = 0.0;  // derived from joints, like a driver
  std::uint8_t drive_count = 0;
  std::uint8_t steer_count = 0;
  std::array<double, 4> drive_radps = {};  // mechanical rad/s
  std::array<double, 4> drive_duty = {};
  std::array<double, 4> steer_rad = {};
  std::array<double, 4> steer_duty = {};
  std::array<double, 4> steer_vel_radps = {};
  std::uint64_t step = 0;

  // Diff conveniences for legacy tests (drive_count == 2).
  double left_radps() const { return drive_count > 0 ? drive_radps[0] : 0.0; }
  double right_radps() const { return drive_count > 1 ? drive_radps[1] : 0.0; }
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
  double wheel_base_m = 0.56;
  double max_steer_rad = 0.55;
  double max_steer_rate_radps = 2.0;
  double steer_time_constant_s = 0.08;  // servo lag to rate-implied angle
};

class World {
 public:
  void step(double dt_s);
  void set_motor_params(const MotorParams& params) { motor_ = params; }
  // Actuator-level plant input: normalized duties per the command mode.
  // Wheel speeds first-order track duty-implied targets; steers servo
  // toward duty-implied angles; the body follows by chassis kinematics.
  void apply(const robot::iface::ActuatorCommand& cmd);
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
  robot::iface::ActuatorMode last_mode_ = robot::iface::ActuatorMode::kDiffDuty;
  std::vector<StaticBox> static_boxes_;
  std::vector<Actor> actors_;
  std::vector<NamedPlace> places_;
  std::uint64_t geometry_version_ = 0;
};

}  // namespace lost_signal::sim::world
