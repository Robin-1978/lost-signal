#include "sim/world/world.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "sim/fixed_time.hpp"

namespace lost_signal::sim::world {
namespace {

void accumulate_impact(ImpactViz& aggregate, const ImpactViz& sample) {
 if (!sample.valid || !(sample.normal_impulse_Ns > 0.0)) return;
 const double previous = aggregate.normal_impulse_Ns;
 const double total = previous + sample.normal_impulse_Ns;
 aggregate.x_m =
 (aggregate.x_m * previous + sample.x_m * sample.normal_impulse_Ns) / total;
 aggregate.y_m =
 (aggregate.y_m * previous + sample.y_m * sample.normal_impulse_Ns) / total;
 aggregate.z_m =
 (aggregate.z_m * previous + sample.z_m * sample.normal_impulse_Ns) / total;
 aggregate.normal_impulse_Ns = total;
 aggregate.relative_speed_mps =
 std::max(aggregate.relative_speed_mps, sample.relative_speed_mps);
 aggregate.valid = true;
 if (sample.spark_eligible && sample.spark_normal_impulse_Ns > 0.0) {
 const double spark_previous = aggregate.spark_normal_impulse_Ns;
 const double spark_total =
 spark_previous + sample.spark_normal_impulse_Ns;
 aggregate.spark_x_m =
 (aggregate.spark_x_m * spark_previous +
 sample.spark_x_m * sample.spark_normal_impulse_Ns) / spark_total;
 aggregate.spark_y_m =
 (aggregate.spark_y_m * spark_previous +
 sample.spark_y_m * sample.spark_normal_impulse_Ns) / spark_total;
 aggregate.spark_z_m =
 (aggregate.spark_z_m * spark_previous +
 sample.spark_z_m * sample.spark_normal_impulse_Ns) / spark_total;
 aggregate.spark_normal_impulse_Ns = spark_total;
 aggregate.spark_relative_speed_mps = std::max(
 aggregate.spark_relative_speed_mps, sample.spark_relative_speed_mps);
 aggregate.spark_robot_material = sample.spark_robot_material;
 aggregate.spark_static_material = sample.spark_static_material;
 aggregate.spark_eligible = true;
 }
}

} // namespace

World::World() = default;

void World::set_motor_params(const MotorParams& params) {
  motor_ = params;
  brake_request_ = {};
}

void World::set_brake_request(int wheel, double normalized) {
  if (wheel < 0 || wheel >= 4) return;
  brake_request_[wheel] = std::clamp(normalized, 0.0, 1.0);
}

void World::set_incline(double forward_grade) {
  incline_grade_ = std::isfinite(forward_grade) ? forward_grade : 0.0;
}

void World::apply(const robot::iface::ActuatorCommand& cmd) {
  truth_.drive_count = robot::iface::actuator_drive_count(cmd.mode);
  truth_.steer_count = robot::iface::actuator_steer_count(cmd.mode);
  for (int i = 0; i < 4; ++i) {
    truth_.drive_duty[i] =
        i < truth_.drive_count ? std::clamp(cmd.drive[i], -1.0, 1.0) : 0.0;
    truth_.steer_duty[i] =
        i < truth_.steer_count ? std::clamp(cmd.steer[i], -1.0, 1.0) : 0.0;
    // Mode switch retires channels: stale slip flags must not linger.
    if (i >= truth_.drive_count) truth_.drive_slipping[i] = false;
  }
}

void World::read_engine_truth() {
 truth_.body_x_m = mujoco_.body_x();
 truth_.body_y_m = mujoco_.body_y();
 truth_.body_yaw_rad = mujoco_.body_yaw();
 truth_.forward_mps = mujoco_.forward_mps();
 truth_.lateral_mps = mujoco_.lateral_mps();
 truth_.yaw_rate_radps = mujoco_.yaw_rate();
 truth_.body_z_m = mujoco_.body_z();
 truth_.body_roll_rad = mujoco_.body_roll();
 truth_.body_pitch_rad = mujoco_.body_pitch();
 for (int i = 0; i < truth_.drive_count && i < 4; ++i) {
 truth_.drive_angle_rad[i] = mujoco_.drive_angle(i);
 truth_.drive_radps[i] = mujoco_.drive_radps(i);
 truth_.drive_slipping[i] = mujoco_.drive_slipping(i);
 }
 for (int i = 0; i < truth_.steer_count && i < 4; ++i) {
 truth_.steer_rad[i] = mujoco_.steer_angle(i);
 truth_.steer_vel_radps[i] = mujoco_.steer_vel(i);
 }
}

void World::prepare() {
 mujoco_.sync(motor_, static_boxes_, materials_, material_zones_,
 terrain_patches_, support_slabs_, geometry_version_,
 truth_.drive_count, truth_.steer_count);
 if (mujoco_.valid()) read_engine_truth();
}

void World::begin_fixed_transition() {
 aggregating_impact_ = true;
 presented_impact_ = {};
}

void World::end_fixed_transition() { aggregating_impact_ = false; }

void World::step_engine(double dt_s) {
  mujoco_.sync(motor_, static_boxes_, materials_, material_zones_,
               terrain_patches_, support_slabs_, geometry_version_,
               truth_.drive_count, truth_.steer_count);
  if (!mujoco_.valid() || !(dt_s > 0.0)) return;
  mujoco_.step(dt_s, motor_, truth_.drive_duty, truth_.steer_duty,
               brake_request_, incline_grade_, material_zones_, materials_,
 environment_, sim_time_s());
 read_engine_truth();
}

void World::step(double dt_s) {
 if (!std::isfinite(dt_s)) return;
 if (!(dt_s > 0.0)) {
 // A zero-duration call is a supported configuration rebuild/read-back.
 // It advances neither the canonical clock nor dynamic actors.
 step_engine(dt_s);
 presented_impact_ = mujoco_.impact();
 return;
 }
 const auto dt_ns = static_cast<robot::core::MonotonicNs>(
 std::llround(dt_s * static_cast<double>(kNanosecondsPerSecond)));
 if (dt_ns <= 0) return;
 if (dt_ns % kPhysicsQuantumNs != 0) {
 throw std::invalid_argument(
 "world step duration must be an integer multiple of 1 ms");
 }

 if (!aggregating_impact_) presented_impact_ = {};
 constexpr double kQuantumS =
 static_cast<double>(kPhysicsQuantumNs) /
 static_cast<double>(kNanosecondsPerSecond);
 const auto quanta = dt_ns / kPhysicsQuantumNs;
 for (robot::core::MonotonicNs quantum = 0; quantum < quanta; ++quantum) {
 sim_time_ns_ += kPhysicsQuantumNs;
 // WorldTruth.step is the public 20 ms scenario transition count, not the
 // number of internal physics/sensor partitions.
 truth_.step = static_cast<std::uint64_t>(
 sim_time_ns_ / kScenarioTransitionNs);
 step_engine(kQuantumS);
 accumulate_impact(presented_impact_, mujoco_.impact());

 // Actors stay kinematic until they become engine bodies, but their policy,
 // yielding state, and integration share the exact physics quantum. Thus a
 // 20 ms harness call and twenty 1 ms viewer calls have identical semantics.
 const double c = std::cos(truth_.body_yaw_rad);
 const double s = std::sin(truth_.body_yaw_rad);
 const double robot_vx =
 truth_.forward_mps * c - truth_.lateral_mps * s;
 const double robot_vy =
 truth_.forward_mps * s + truth_.lateral_mps * c;
 const double robot_radius = std::hypot(
 std::max(0.0, motor_.body_half_length_m),
 std::max(0.0, motor_.body_half_width_m));
 for (auto& actor : actors_) {
 step_actor(actor, actor.behavior, kQuantumS);
 apply_actor_robot_yield(actor, actor.behavior,
 truth_.body_x_m, truth_.body_y_m, robot_vx, robot_vy,
 robot_radius, kQuantumS);
 actor.x += actor.vx_mps * kQuantumS;
 actor.y += actor.vy_mps * kQuantumS;
 }
  }
}

}  // namespace lost_signal::sim::world
