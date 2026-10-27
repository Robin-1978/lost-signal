#include "sim/viewer/frame_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/autonomy/planning/prediction.hpp"
#include "robot/core/deskew.hpp"
#include "robot/core/transform.hpp"
#include "sim/world/terrain.hpp"

namespace lost_signal::sim::viewer {
namespace {

const char* safety_state_name(robot::autonomy::SafetyState state) {
 switch (state) {
 case robot::autonomy::SafetyState::kNominal: return "NOMINAL";
 case robot::autonomy::SafetyState::kDegraded: return "DEGRADED";
 case robot::autonomy::SafetyState::kStopped: return "STOPPED";
 }
 return "UNKNOWN";
}

double seconds(robot::core::MonotonicNs stamp_ns) {
 return stamp_ns == robot::core::kInvalidTime
 ? -1.0 : static_cast<double>(stamp_ns) * 1e-9;
}

const char* impact_material_name(world::ImpactMaterial material) {
 switch (material) {
 case world::ImpactMaterial::kRubber: return "rubber";
 case world::ImpactMaterial::kMetal: return "metal";
 case world::ImpactMaterial::kConcrete: return "concrete";
 case world::ImpactMaterial::kGlass: return "glass";
 case world::ImpactMaterial::kWood: return "wood";
 case world::ImpactMaterial::kPolymer: return "polymer";
 case world::ImpactMaterial::kUnknown: return "unknown";
 }
 return "unknown";
}

struct PlanarMapFromOdom {
 bool valid = false;
 double x_m = 0.0;
 double y_m = 0.0;
 double yaw_rad = 0.0;
};

PlanarMapFromOdom map_from_odom_for(const eval::EvalStep& step) {
 const auto& map_base = step.diag.estimate;
 const auto& odom_base = step.diag.odometry_estimate;
 if (!map_base.valid || !odom_base.valid) return {};
 const auto transform = robot::core::map_from_odom_delta(
 odom_base.x_m, odom_base.y_m, odom_base.yaw_rad,
 map_base.x_m, map_base.y_m, map_base.yaw_rad);
 return {true, transform.translation_m.x, transform.translation_m.y,
 2.0 * std::atan2(transform.rotation.z, transform.rotation.w)};
}

bool materially_different(const PlanarMapFromOdom& before,
 const PlanarMapFromOdom& after) {
 if (!before.valid || !after.valid) return false;
 const double scale = std::max({1.0, std::abs(before.x_m),
 std::abs(before.y_m), std::abs(after.x_m), std::abs(after.y_m)});
 const double numeric_tolerance =
 64.0 * std::numeric_limits<double>::epsilon() * scale;
 return std::hypot(after.x_m - before.x_m, after.y_m - before.y_m) >
 numeric_tolerance ||
 std::abs(robot::core::wrap_angle(
 after.yaw_rad - before.yaw_rad)) > numeric_tolerance;
}

void transform_odom_pose(const PlanarMapFromOdom& map_from_odom,
 const robot::autonomy::EstimatedState& odom_base,
 double& x_m, double& y_m, double& yaw_rad) {
 const double c = std::cos(map_from_odom.yaw_rad);
 const double s = std::sin(map_from_odom.yaw_rad);
 x_m = map_from_odom.x_m + c * odom_base.x_m - s * odom_base.y_m;
 y_m = map_from_odom.y_m + s * odom_base.x_m + c * odom_base.y_m;
 yaw_rad = robot::core::wrap_angle(
 map_from_odom.yaw_rad + odom_base.yaw_rad);
}

} // namespace

std::vector<std::pair<double, double>> lidar_endpoints(
    double body_x_m, double body_y_m, double body_yaw_rad,
 const robot::iface::ObservationFrame& obs, const LidarMount& mount,
 double measured_forward_mps, double measured_yaw_rate_radps,
 std::vector<float>* retained_time_offsets_s) {
  std::vector<std::pair<double, double>> hits;
 if (retained_time_offsets_s != nullptr) retained_time_offsets_s->clear();
  if (!obs.scan_valid || obs.ranges_m.empty()) return hits;
  const double inc = obs.scan_angle_increment_rad;
  if (!(inc > 0.0) || !std::isfinite(inc)) return hits;
  // True sensor origin: mount offset rotated by body yaw (+ mount yaw on
  // the beam fan, same composition as generation).
  const double cy = std::cos(body_yaw_rad), sy = std::sin(body_yaw_rad);
  const double ox = body_x_m + mount.xyz_m[0] * cy - mount.xyz_m[1] * sy;
  const double oy = body_y_m + mount.xyz_m[0] * sy + mount.xyz_m[1] * cy;
  const double fan =
      (std::isfinite(mount.rpy_rad[2]) ? mount.rpy_rad[2] : 0.0);
  hits.reserve(obs.ranges_m.size());
 if (retained_time_offsets_s != nullptr) {
 retained_time_offsets_s->reserve(obs.ranges_m.size());
 }
 const bool has_time_offsets = obs.time_offset_valid &&
 obs.time_offsets_s.size() == obs.ranges_m.size();
 const robot::core::DeskewTwist twist{
 measured_forward_mps, measured_yaw_rate_radps};
  for (std::size_t i = 0; i < obs.ranges_m.size(); ++i) {
    const float r = obs.ranges_m[i];
    if (!std::isfinite(r) || r <= 0.0f) continue;
 const double beam_angle = obs.scan_angle_min_rad + i * inc;
 const float time_offset_s = has_time_offsets
 ? obs.time_offsets_s[i] : 0.0f;
 const robot::core::Vec2 local = robot::core::deskew_beam(
 r, beam_angle, time_offset_s, twist);
 const double a = body_yaw_rad + fan;
 const double ca = std::cos(a), sa = std::sin(a);
 hits.emplace_back(
 ox + ca * local.x - sa * local.y,
 oy + sa * local.x + ca * local.y);
 if (retained_time_offsets_s != nullptr) {
 retained_time_offsets_s->push_back(time_offset_s);
 }
  }
  return hits;
}

Frame build_frame(const world::World& world, const eval::EvalStep& step,
                  const std::vector<std::string>& reached_ids,
                  const std::vector<std::pair<double, double>>& trail,
                  const EditorSelection& sel, const LidarMount& mount) {
  Frame f;
  f.step = step.step;
 // Scenario transitions are fixed at 20 ms. Historical frames use the
 // recorded step index rather than viewer wall-clock time.
 f.sim_time_s = static_cast<double>(step.step) * 0.02;
  const world::WorldTruth& t = step.truth_after;
  f.body.x_m = t.body_x_m;
  f.body.y_m = t.body_y_m;
  f.body.yaw_rad = t.body_yaw_rad;
 f.body.forward_mps = t.forward_mps;
 f.body.lateral_mps = t.lateral_mps;
 f.body.yaw_rate_radps = t.yaw_rate_radps;
 f.body.drive_free_speed_mps = std::max(
  0.0, world.motor_params().free_speed_mps);
 f.body.nominal_normal_load_N = std::max(
  0.0, (world.motor_params().vehicle_mass_kg +
         world.motor_params().payload_mass_kg) * 9.81);
  f.body.z_m = t.body_z_m;
  f.body.roll_rad = t.body_roll_rad;
  f.body.pitch_rad = t.body_pitch_rad;
 f.body.com_x_m = world.system_com_relative_x_m();
 f.body.com_y_m = world.system_com_relative_y_m();
 f.body.com_z_m = world.system_com_relative_z_m();
 f.truth.valid = true;
 f.truth.x_m = t.body_x_m;
 f.truth.y_m = t.body_y_m;
 f.truth.yaw_rad = t.body_yaw_rad;
 f.truth.forward_mps = t.forward_mps;
 f.truth.yaw_rate_radps = t.yaw_rate_radps;
  // Wheels from the backend wheel table (layout + live joints + slip
  // flags — the same sources physics and the encoder provider read).
  // Layouts: (2,0)+scrub = skid-steer slabs, (2,0) diff, (2,2) ackermann
  // (rears driven, fronts steered idlers with live spin), (4,4) swerve,
  // (4,0) mecanum (+ static 45-degree roller stripes: rollers are
  // analytic-wrench state, never meshed, never drawn live).
  {
    const int nd = static_cast<int>(t.drive_count);
    const int ns = static_cast<int>(t.steer_count);
    const double scrub = world.motor_params().yaw_scrub_coulomb_Nm;
    f.body.tracked = (nd == 2 && ns == 0 && scrub > 0.0);
    const bool mecanum = (nd == 4 && ns == 0);
 std::vector<world::WheelViz> live_wheels;
 if (!step.presentation.captured) {
 world::WheelViz table[6];
 const int count = world.wheels(table, 6);
 live_wheels.assign(table, table + std::max(0, count));
 }
 const auto& wheels = step.presentation.captured
 ? step.presentation.wheels : live_wheels;
 for (std::size_t i = 0; i < wheels.size(); ++i) {
 const auto& physical = wheels[i];
      FrameWheel w;
 w.x_m = physical.x_m;
 w.y_m = physical.y_m;
 w.steer_rad = physical.steer_rad;
 w.spin_rad = physical.spin_rad;
 w.spin_rate_radps = physical.spin_rate_radps;
 w.radius_m = world.motor_params().wheel_radius_m;
 w.half_width_m = world.motor_params().tire_half_width_m;
 w.slipping = physical.slipping;
 w.driven = physical.driven;
 w.duty = i < static_cast<std::size_t>(nd) ? t.drive_duty[i] : 0.0;
 w.steer_rate_radps = physical.steer_rate_radps;
      w.mecanum_rollers = mecanum;

      const double module_vx = t.forward_mps - t.yaw_rate_radps * w.y_m;
      const double module_vy = t.lateral_mps + t.yaw_rate_radps * w.x_m;
      const double wheel_c = std::cos(w.steer_rad);
      const double wheel_s = std::sin(w.steer_rad);
      w.contact_speed_mps = module_vx * wheel_c + module_vy * wheel_s;
      const double longitudinal_slip =
        w.contact_speed_mps - w.radius_m * w.spin_rate_radps;
      const double lateral_slip =
        -module_vx * wheel_s + module_vy * wheel_c;
      // Mecanum lateral motion is resolved by passive rollers. Treating that
      // intended velocity as tire slip would make the visual effect lie.
      const double kinematic_slip = mecanum ? std::abs(longitudinal_slip)
        : std::hypot(longitudinal_slip, lateral_slip);
 w.contact_valid = physical.contact_valid;
 w.contact_x_m = physical.contact_x_m;
 w.contact_y_m = physical.contact_y_m;
 w.contact_z_m = physical.contact_z_m;
 w.contact_normal_force_N = physical.contact_normal_force_N;
 w.slip_speed_mps = physical.contact_valid
 ? physical.contact_slip_speed_mps : kinematic_slip;
 w.longitudinal_slip_mps = physical.contact_valid
 ? physical.contact_longitudinal_slip_mps : longitudinal_slip;
 w.lateral_slip_mps = physical.contact_valid
 ? physical.contact_lateral_slip_mps : lateral_slip;
 w.contact_motion_x_mps = physical.contact_motion_x_mps;
 w.contact_motion_y_mps = physical.contact_motion_y_mps;
 w.contact_motion_z_mps = physical.contact_motion_z_mps;
 w.contact_slip_x_mps = physical.contact_slip_x_mps;
 w.contact_slip_y_mps = physical.contact_slip_y_mps;
 w.contact_slip_z_mps = physical.contact_slip_z_mps;
 w.friction_utilization = physical.contact_valid
 ? physical.contact_friction_utilization : 0.0;
      const double body_c = std::cos(t.body_yaw_rad);
      const double body_s = std::sin(t.body_yaw_rad);
      const double contact_x = w.contact_valid ? w.contact_x_m
        : t.body_x_m + body_c * w.x_m - body_s * w.y_m;
      const double contact_y = w.contact_valid ? w.contact_y_m
        : t.body_y_m + body_s * w.x_m + body_c * w.y_m;
      if (const auto* zone = world::material_zone_at(
        world.material_zones(), contact_x, contact_y)) {
        w.contact_material_id = zone->material_id;
 if (const auto* material = world::find_material(
 world.materials(), zone->material_id)) {
 w.contact_surface_effect = material->surface_effect;
 w.contact_fluid_depth_m = material->fluid_depth_m;
 }
      }
 w.pose = {physical.pose.valid,
 physical.pose.x_m, physical.pose.y_m, physical.pose.z_m,
 physical.pose.qw, physical.pose.qx, physical.pose.qy,
 physical.pose.qz};
 w.steering_pose = {physical.steer_joint.link_pose.valid,
 physical.steer_joint.link_pose.x_m, physical.steer_joint.link_pose.y_m,
 physical.steer_joint.link_pose.z_m, physical.steer_joint.link_pose.qw,
 physical.steer_joint.link_pose.qx, physical.steer_joint.link_pose.qy,
 physical.steer_joint.link_pose.qz};
      f.body.wheels.push_back(w);
 if (w.contact_valid) ++f.truth.wheel_contacts;
    }
  }
 // Render the unpowered supports from the passive caster joints. The
 // backend read-back is SIM-only; no support state enters the robot-facing
 // encoder or actuator channels. The fallback is used only when the plant is
 // unavailable (for example a frame assembled in a disabled-MuJoCo build).
 {
 std::vector<world::CasterViz> live_casters;
 if (!step.presentation.captured) {
 world::CasterViz table[world::MotorParams::kMaxSupports];
 const int count = world.casters(table, world::MotorParams::kMaxSupports);
 live_casters.assign(table, table + std::max(0, count));
 }
 const auto& casters = step.presentation.captured
 ? step.presentation.casters : live_casters;
 const int physical_count = static_cast<int>(casters.size());
 const double speed = std::hypot(t.forward_mps, t.lateral_mps);
 const int count = physical_count > 0 ? physical_count
 : (world.motor_params().support_count > 0
 ? world.motor_params().support_count : 2);
 for (int i = 0; i < count &&
 i < lost_signal::sim::world::MotorParams::kMaxSupports; ++i) {
 const auto& support = world.motor_params().support_count > 0
 ? world.motor_params().supports[i]
 : world::MotorParams::SupportParams{};
 FrameCaster caster;
 if (physical_count > 0) {
 const auto& physical = casters[static_cast<std::size_t>(i)];
 caster.pivot_x_m = physical.pivot_x_m;
 caster.pivot_y_m = physical.pivot_y_m;
 caster.x_m = physical.x_m;
 caster.y_m = physical.y_m;
 caster.radius_m = physical.radius_m;
      caster.trail_m = std::hypot(
 physical.pivot_x_m - physical.x_m,
 physical.pivot_y_m - physical.y_m);
 caster.yaw_rad = physical.swivel_rad;
 caster.spin_rad = physical.roll_rad;
 caster.fork_pose = {physical.fork_pose.valid,
 physical.fork_pose.x_m, physical.fork_pose.y_m,
 physical.fork_pose.z_m, physical.fork_pose.qw,
 physical.fork_pose.qx, physical.fork_pose.qy,
 physical.fork_pose.qz};
 caster.wheel_pose = {physical.wheel_pose.valid,
 physical.wheel_pose.x_m, physical.wheel_pose.y_m,
 physical.wheel_pose.z_m, physical.wheel_pose.qw,
 physical.wheel_pose.qx, physical.wheel_pose.qy,
 physical.wheel_pose.qz};
 } else {
 caster.pivot_x_m = world.motor_params().support_count > 0
 ? support.x_m : (i == 0 ? 0.9 * world.motor_params().wheel_base_m * 0.5
 : -0.9 * world.motor_params().wheel_base_m * 0.5);
      caster.pivot_y_m = world.motor_params().support_count > 0
        ? support.y_m : 0.0;
      caster.trail_m = world.motor_params().support_count > 0
        ? support.trail_m : 0.02;
 caster.radius_m = world.motor_params().support_count > 0
 ? support.radius_m : 0.03;
 caster.yaw_rad = speed > 1e-6
 ? std::atan2(t.lateral_mps, t.forward_mps) : 0.0;
      caster.x_m = caster.pivot_x_m - caster.trail_m * std::cos(caster.yaw_rad);
      caster.y_m = caster.pivot_y_m - caster.trail_m * std::sin(caster.yaw_rad);
 // No physical caster joint was recorded. Keep the wheel at its declared
 // zero pose instead of fabricating a historical angle from current time.
 caster.spin_rad = 0.0;
 }
 f.body.casters.push_back(caster);
 }
 }
 // Engineering overlays consume the same joint frames as the physics
 // read-back. They are kept in FrameBody, alongside link poses, and are
 // absent from every ROBOT-side telemetry record.
 {
 std::vector<world::JointAxisViz> live_axes;
 if (!step.presentation.captured) {
 world::JointAxisViz table[20];
 const int count = world.joint_axes(table, 20);
 live_axes.assign(table, table + std::max(0, count));
 }
 const auto& axes = step.presentation.captured
 ? step.presentation.joint_axes : live_axes;
 for (const auto& physical : axes) {
 FrameJointAxis axis;
 axis.valid = physical.valid;
 axis.link_pose = {physical.link_pose.valid,
 physical.link_pose.x_m, physical.link_pose.y_m, physical.link_pose.z_m,
 physical.link_pose.qw, physical.link_pose.qx, physical.link_pose.qy,
 physical.link_pose.qz};
 axis.axis_x = physical.axis_x;
 axis.axis_y = physical.axis_y;
 axis.axis_z = physical.axis_z;
 axis.angle_rad = physical.angle_rad;
 axis.rate_radps = physical.rate_radps;
 f.body.joint_axes.push_back(axis);
 }
 if (step.presentation.captured) {
 f.body.com_x_m = step.presentation.com_x_m;
 f.body.com_y_m = step.presentation.com_y_m;
 f.body.com_z_m = step.presentation.com_z_m;
 }
 }
 {
 std::vector<world::ContactViz> live_contacts;
 if (!step.presentation.captured) {
 world::ContactViz table[64];
 const int count = world.contacts(table, 64);
 live_contacts.assign(table, table + std::max(0, count));
 }
 const auto& contacts = step.presentation.captured
 ? step.presentation.contacts : live_contacts;
 for (const auto& physical : contacts) {
 FrameContact contact;
 contact.valid = physical.valid;
 contact.x_m = physical.x_m;
 contact.y_m = physical.y_m;
 contact.z_m = physical.z_m;
 contact.normal_x = physical.normal_x;
 contact.normal_y = physical.normal_y;
 contact.normal_z = physical.normal_z;
 contact.normal_force_N = physical.normal_force_N;
 f.body.contacts.push_back(contact);
 }
 }
 {
 std::vector<world::ArmLinkViz> live_arm_links;
 if (!step.presentation.captured) {
 world::ArmLinkViz table[4];
 const int count = world.arm_links(table, 4);
 live_arm_links.assign(table, table + std::max(0, count));
 }
 const auto& arm_links = step.presentation.captured
 ? step.presentation.arm_links : live_arm_links;
 for (const auto& physical : arm_links) {
 FrameArmLink link;
 link.valid = physical.valid;
 link.pose = {physical.pose.valid, physical.pose.x_m, physical.pose.y_m,
 physical.pose.z_m, physical.pose.qw, physical.pose.qx,
 physical.pose.qy, physical.pose.qz};
 link.joint.valid = physical.joint.valid;
 link.joint.link_pose = {physical.joint.link_pose.valid,
 physical.joint.link_pose.x_m, physical.joint.link_pose.y_m,
 physical.joint.link_pose.z_m, physical.joint.link_pose.qw,
 physical.joint.link_pose.qx, physical.joint.link_pose.qy,
 physical.joint.link_pose.qz};
 link.joint.axis_x = physical.joint.axis_x;
 link.joint.axis_y = physical.joint.axis_y;
 link.joint.axis_z = physical.joint.axis_z;
 link.joint.angle_rad = physical.joint.angle_rad;
 link.joint.rate_radps = physical.joint.rate_radps;
 f.body.arm_links.push_back(link);
 }
 }
  // True 3D sensor origin (body + attitude composed with the mount — the
  // same composition generation runs, so the viz fan starts where the
  // beams do).
  {
    const robot::core::Quaternion bq = robot::core::quaternion_from_rpy(
        t.body_roll_rad, t.body_pitch_rad, t.body_yaw_rad);
    const robot::core::Vec3 off = robot::core::rotate_vector(
        bq, {mount.xyz_m[0], mount.xyz_m[1], mount.xyz_m[2]});
    f.body.sens_x_m = t.body_x_m + off.x;
    f.body.sens_y_m = t.body_y_m + off.y;
    f.body.sens_z_m = t.body_z_m + off.z;
  }
 const double half_length = world.motor_params().body_half_length_m;
 const double half_width = world.motor_params().body_half_width_m;
 if (half_length > 0.0) f.body.half_length_m = half_length;
 if (half_width > 0.0) f.body.half_width_m = half_width;
 if (step.diag.estimate.valid) {
    f.body.estimate_valid = true;
    f.body.est_x_m = step.diag.estimate.x_m;
    f.body.est_y_m = step.diag.estimate.y_m;
    f.body.est_yaw_rad = step.diag.estimate.yaw_rad;
  }
 f.observation.accepted = step.obs_accepted;
 f.observation.scan_valid = step.obs.scan_valid;
 f.observation.odometry_valid = step.obs.odometry_valid ||
 step.obs.has_joints();
 f.observation.imu_valid = step.obs.imu_valid;
 f.observation.sequence = step.obs.sequence;
 f.observation.stamp_s = seconds(step.obs.stamp_ns);
 f.observation.lidar_beams = step.obs.ranges_m.size();
 f.observation.lidar_returns = static_cast<std::size_t>(std::count_if(
 step.obs.ranges_m.begin(), step.obs.ranges_m.end(),
 [](float range_m) { return std::isfinite(range_m) && range_m > 0.0f; }));
 const auto scan_header = robot::iface::scan_measurement_header(step.obs);
 f.observation.scan_source = scan_header.source;
 f.observation.scan_sequence = scan_header.sequence;
 f.observation.scan_stamp_s = seconds(scan_header.stamp_ns);

 f.estimate.valid = step.diag.estimate.valid;
 f.estimate.localized = step.diag.localized;
 f.estimate.stamp_s = seconds(step.diag.estimate.stamp_ns);
 f.estimate.map_x_m = step.diag.estimate.x_m;
 f.estimate.map_y_m = step.diag.estimate.y_m;
 f.estimate.map_yaw_rad = step.diag.estimate.yaw_rad;
 f.estimate.odom_x_m = step.diag.odometry_estimate.x_m;
 f.estimate.odom_y_m = step.diag.odometry_estimate.y_m;
 f.estimate.odom_yaw_rad = step.diag.odometry_estimate.yaw_rad;
 f.estimate.position_sigma_m = step.diag.estimate.position_sigma_m;
 f.estimate.heading_sigma_rad = step.diag.estimate.heading_sigma_rad;
 f.estimate.localization_action = step.diag.localization_action;
 f.estimate.slam_keyframes = step.diag.slam_keyframes;
 f.estimate.slam_loops_closed = step.diag.slam_loops_closed;
 if (step.diag.estimate.valid && step.obs.scan_valid) {
 const double estimate_yaw = step.diag.estimate.yaw_rad;
 const double cy = std::cos(estimate_yaw);
 const double sy = std::sin(estimate_yaw);
 f.observation.projection_valid = true;
 f.observation.projection_pose_stamp_s =
 seconds(step.diag.estimate.stamp_ns);
 f.observation.projected_sensor_x_m = step.diag.estimate.x_m +
 mount.xyz_m[0] * cy - mount.xyz_m[1] * sy;
 f.observation.projected_sensor_y_m = step.diag.estimate.y_m +
 mount.xyz_m[0] * sy + mount.xyz_m[1] * cy;
 // The planar ROBOT estimate has no truth elevation channel. Render the
 // sensor at its declared base-frame height instead of borrowing body_z.
 f.observation.projected_sensor_z_m = mount.xyz_m[2];
 }

 const auto& evidence = step.diag.evidence;
 f.decision.valid = evidence.sequence > 0;
 f.decision.sequence = evidence.sequence;
 f.decision.stamp_s = evidence.decision_time_s;
 f.decision.action_code = evidence.action_code;
 f.decision.reason_code = evidence.reason_code;
 f.decision.constraint_code = evidence.constraint_code;
 f.decision.safety_state = safety_state_name(step.diag.safety);
 f.decision.safety_reason = step.diag.safety_reason;
 f.decision.command_forward_mps = step.cmd.forward_mps;
 f.decision.command_yaw_rate_radps = step.cmd.yaw_rate_radps;
 f.actuation.drive_count = robot::iface::actuator_drive_count(
  step.actuator.mode);
 f.actuation.drive_duty = step.actuator.drive;
 f.actuation.service_brake = std::clamp(
  step.actuator.service_brake, 0.0, 1.0);
 f.decision.route_point_count = evidence.route_point_count;
 f.decision.moving_candidate_count = evidence.moving_candidate_count;
 if (evidence.local_costmap.valid()) {
 f.decision.local_costmap_resolution_m =
 evidence.local_costmap.resolution_m;
 f.decision.local_costmap_width = evidence.local_costmap.width;
 f.decision.local_costmap_height = evidence.local_costmap.height;
 f.decision.local_costmap_origin_x_m = evidence.local_costmap.origin_x_m;
 f.decision.local_costmap_origin_y_m = evidence.local_costmap.origin_y_m;
 f.decision.local_costmap_cells = evidence.local_costmap.cells;
 }
 f.decision.validator_swept_envelope_radius_m =
 std::max(0.0, evidence.validator_swept_envelope_radius_m);
 f.decision.validator_swept_centerline.reserve(
 evidence.validator_swept_centerline.size());
 for (const auto& point : evidence.validator_swept_centerline) {
 f.decision.validator_swept_centerline.emplace_back(point.x, point.y);
 }
 f.decision.safety_static_envelope_radius_m =
 std::max(0.0, evidence.safety_static_envelope_radius_m);
 if (step.diag.estimate.valid) {
 const double c = std::cos(step.diag.estimate.yaw_rad);
 const double s = std::sin(step.diag.estimate.yaw_rad);
 f.decision.safety_static_sweep.reserve(
 evidence.safety_static_sweep.size());
 for (const auto& point : evidence.safety_static_sweep) {
 f.decision.safety_static_sweep.emplace_back(
 step.diag.estimate.x_m + c * point.x - s * point.y,
 step.diag.estimate.y_m + s * point.x + c * point.y);
 }
 }

 // Rebuild the deployable predictor output from recorded ROBOT diagnostics.
 // This intentionally has no access to world.actors(): the overlay must show
 // what the robot believed, including missed detections and uncertainty.
 if (!step.diag.dynamic_tracks.empty() &&
 std::isfinite(evidence.decision_time_s)) {
 robot::autonomy::PerceptionResult perception;
 perception.stamp_ns = step.diag.last_obs_stamp_ns;
 perception.tracks = step.diag.dynamic_tracks;
 robot::autonomy::DynamicPredictionConfig prediction_config;
 prediction_config.horizon_s = std::max(0.0, evidence.prediction_horizon_s);
 const auto prediction = robot::autonomy::predict_perception(
 perception, evidence.decision_time_s, prediction_config);
 f.dynamic_tracks.reserve(step.diag.dynamic_tracks.size());
 for (const auto& source : step.diag.dynamic_tracks) {
 FrameDynamicTrack track;
 track.track_id = source.id;
 track.x_m = source.x_m;
 track.y_m = source.y_m;
 track.vx_mps = source.vx_mps;
 track.vy_mps = source.vy_mps;
 track.radius_m = std::max(0.0, source.radius_m);
 track.uncertainty_m = std::max(0.0, source.position_uncertainty_m);
 track.prediction_ready = source.prediction_ready;
 for (const auto& source_hypothesis : prediction.hypotheses) {
 if (source_hypothesis.track_id != source.id) continue;
 FrameDynamicHypothesis hypothesis;
 hypothesis.hypothesis_id = source_hypothesis.hypothesis_id;
 const std::size_t count = std::min(
 prediction.future_times_s.size(), source_hypothesis.states.size());
 hypothesis.points.reserve(count);
 for (std::size_t index = 0; index < count; ++index) {
 const auto& state = source_hypothesis.states[index];
 hypothesis.points.push_back({
 prediction.future_times_s[index], state.x_m, state.y_m,
 std::max(0.0, state.position_uncertainty_m)});
 }
 track.hypotheses.push_back(std::move(hypothesis));
 }
 f.dynamic_tracks.push_back(std::move(track));
 }
 }

 f.evaluation.static_collision = step.judge.static_collision;
 f.evaluation.dynamic_collision = step.judge.dynamic_collision;
 f.evaluation.terrain_collision = step.judge.terrain_collision;
 f.evaluation.static_clearance_m = step.judge.static_clearance_m;
 f.evaluation.dynamic_clearance_m = step.judge.dynamic_clearance_m;
 if (step.diag.estimate.valid) {
 f.evaluation.estimate_error_m = std::hypot(
 step.diag.estimate.x_m - t.body_x_m,
 step.diag.estimate.y_m - t.body_y_m);
 }
  f.trail = trail;
  const auto& boxes = world.static_boxes();
  f.boxes.reserve(boxes.size());
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    FrameBox b;
 b.box_id = boxes[i].box_id;
    b.x0_m = std::min(boxes[i].x0, boxes[i].x1);
    b.y0_m = std::min(boxes[i].y0, boxes[i].y1);
    b.x1_m = std::max(boxes[i].x0, boxes[i].x1);
    b.y1_m = std::max(boxes[i].y0, boxes[i].y1);
    b.height_m = boxes[i].height_m;
    b.selected = (sel.box == static_cast<int>(i));
    b.selected_corner = b.selected ? sel.box_corner : -1;
    f.boxes.push_back(b);
  }
 if (step.presentation.captured) {
 for (const auto& a : step.presentation.actors) {
    FrameActor fa;
 fa.x_m = a.x_m;
 fa.y_m = a.y_m;
    fa.radius_m = a.radius_m;
    fa.vx_mps = a.vx_mps;
    fa.vy_mps = a.vy_mps;
 if (std::hypot(a.vx_mps, a.vy_mps) > 1e-6)
 fa.heading_rad = std::atan2(a.vy_mps, a.vx_mps);
    fa.height_m = a.height_m;
 fa.pedestrian = a.pedestrian;
 fa.world_pose = {a.pose.valid, a.pose.x_m, a.pose.y_m, a.pose.z_m,
 a.pose.qw, a.pose.qx, a.pose.qy, a.pose.qz};
 for (const auto& physical : a.links) {
 FrameActor::FrameActorLink link;
 link.valid = physical.valid;
 link.kind = static_cast<std::uint8_t>(physical.kind);
 link.pose = {physical.pose.valid, physical.pose.x_m, physical.pose.y_m,
 physical.pose.z_m, physical.pose.qw, physical.pose.qx,
 physical.pose.qy, physical.pose.qz};
 fa.links.push_back(link);
 }
 if (a.pose.valid) {
 fa.heading_rad = std::atan2(2.0 * (a.pose.qw * a.pose.qz +
 a.pose.qx * a.pose.qy), 1.0 - 2.0 * (a.pose.qy * a.pose.qy +
 a.pose.qz * a.pose.qz));
 }
    f.actors.push_back(fa);
  }
 } else {
 world::ActorPoseViz actor_poses[64];
 const int actor_pose_count = world.actor_poses(actor_poses, 64);
 world::ActorLinkViz actor_links[512];
 const int actor_link_count = world.actor_links(actor_links, 512);
 std::size_t actor_index = 0;
 for (const auto& a : world.actors()) {
 FrameActor fa;
 fa.x_m = a.x;
 fa.y_m = a.y;
 fa.radius_m = a.radius_m;
 fa.vx_mps = a.vx_mps;
 fa.vy_mps = a.vy_mps;
 if (std::hypot(a.vx_mps, a.vy_mps) > 1e-6)
 fa.heading_rad = std::atan2(a.vy_mps, a.vx_mps);
 fa.height_m = a.height_m;
 fa.pedestrian = a.visual_kind == "pedestrian";
 if (actor_index < static_cast<std::size_t>(std::max(0, actor_pose_count))) {
 const auto& pose = actor_poses[actor_index];
 fa.world_pose = {pose.valid, pose.x_m, pose.y_m, pose.z_m,
 pose.qw, pose.qx, pose.qy, pose.qz};
 if (pose.valid) {
 fa.heading_rad = std::atan2(2.0 * (pose.qw * pose.qz +
 pose.qx * pose.qy), 1.0 - 2.0 * (pose.qy * pose.qy +
 pose.qz * pose.qz));
 }
 }
 for (int link_index = 0; link_index < actor_link_count; ++link_index) {
 if (actor_links[link_index].actor_index != actor_index) continue;
 FrameActor::FrameActorLink link;
 link.valid = actor_links[link_index].valid;
 link.kind = static_cast<std::uint8_t>(actor_links[link_index].kind);
 const auto& pose = actor_links[link_index].pose;
 link.pose = {pose.valid, pose.x_m, pose.y_m, pose.z_m,
 pose.qw, pose.qx, pose.qy, pose.qz};
 fa.links.push_back(link);
 }
 f.actors.push_back(fa);
 ++actor_index;
 }
 }
  for (const auto& z : world.material_zones()) {
    FrameZone fz;
    fz.x0_m = std::min(z.x0, z.x1);
    fz.y0_m = std::min(z.y0, z.y1);
    fz.x1_m = std::max(z.x0, z.x1);
    fz.y1_m = std::max(z.y0, z.y1);
    fz.material_id = z.material_id;
 if (const auto* material = world::find_material(
 world.materials(), z.material_id)) {
 fz.surface_effect = material->surface_effect;
 fz.fluid_depth_m = material->fluid_depth_m;
 }
    f.zones.push_back(fz);
  }
  for (const auto& p : world.terrain_patches()) {
    FramePatch fp;
    fp.x0_m = std::min(p.x0, p.x1);
    fp.y0_m = std::min(p.y0, p.y1);
    fp.x1_m = std::max(p.x0, p.x1);
    fp.y1_m = std::max(p.y0, p.y1);
    f.patches.push_back(fp);
  }
  const auto& places = world.places();
  f.places.reserve(places.size());
  for (std::size_t i = 0; i < places.size(); ++i) {
    FramePlace p;
    p.place_id = places[i].place_id;
    p.x_m = places[i].x_m;
    p.y_m = places[i].y_m;
    p.yaw_rad = places[i].yaw_rad;
    p.reached = std::find(reached_ids.begin(), reached_ids.end(),
                           places[i].place_id) != reached_ids.end();
    p.selected = (sel.place == static_cast<int>(i));
    f.places.push_back(p);
  }
 if (f.observation.projection_valid) {
 f.lidar_hits = lidar_endpoints(
 step.diag.estimate.x_m, step.diag.estimate.y_m,
 step.diag.estimate.yaw_rad, step.obs, mount,
 step.diag.estimate.forward_mps, step.diag.estimate.yaw_rate_radps,
 &f.lidar_time_offsets_s);
 }
  f.lidar_valid = !f.lidar_hits.empty();
 f.truth_lidar_hits = lidar_endpoints(
 t.body_x_m, t.body_y_m, t.body_yaw_rad, step.obs, mount,
 t.forward_mps, t.yaw_rate_radps,
 &f.truth_lidar_time_offsets_s);
 f.truth_lidar_valid = !f.truth_lidar_hits.empty();
  f.static_collision = step.judge.static_collision;
  f.dynamic_collision = step.judge.dynamic_collision;
  f.static_clearance_m = step.judge.static_clearance_m;
 const world::ImpactViz& impact = step.presentation.captured
 ? step.presentation.impact : world.impact();
 f.impact.valid = impact.valid;
 f.impact.x_m = impact.x_m;
 f.impact.y_m = impact.y_m;
 f.impact.z_m = impact.z_m;
 f.impact.normal_impulse_Ns = impact.normal_impulse_Ns;
 f.impact.relative_speed_mps = impact.relative_speed_mps;
 f.impact.spark_eligible = impact.spark_eligible;
 f.impact.spark_x_m = impact.spark_x_m;
 f.impact.spark_y_m = impact.spark_y_m;
 f.impact.spark_z_m = impact.spark_z_m;
 f.impact.spark_normal_impulse_Ns = impact.spark_normal_impulse_Ns;
 f.impact.spark_relative_speed_mps = impact.spark_relative_speed_mps;
 f.impact.spark_robot_material =
 impact_material_name(impact.spark_robot_material);
 f.impact.spark_static_material =
 impact_material_name(impact.spark_static_material);
  return f;
}

std::optional<Frame> build_episode_frame(
 const world::World& world, const eval::EpisodeRecorder& episode,
 std::size_t episode_index, const std::vector<std::string>& reached_ids,
 const EditorSelection& sel, const LidarMount& mount,
 std::size_t maximum_trail_points) {
 const auto& steps = episode.steps();
 if (episode_index >= steps.size()) return std::nullopt;
 const std::size_t count = maximum_trail_points == 0
 ? 0 : std::min(maximum_trail_points, episode_index + 1);
 const std::size_t begin = episode_index + 1 - count;
 std::vector<std::pair<double, double>> trail;
 trail.reserve(count);
 for (std::size_t i = begin; i <= episode_index && count > 0; ++i) {
 trail.emplace_back(steps[i].truth_after.body_x_m,
 steps[i].truth_after.body_y_m);
 }
 Frame frame = build_frame(
 world, steps[episode_index], reached_ids, trail, sel, mount);
 if (episode_index > 0) {
 const auto before = map_from_odom_for(steps[episode_index - 1]);
 const auto after = map_from_odom_for(steps[episode_index]);
 const auto& odom_base = steps[episode_index].diag.odometry_estimate;
 if (odom_base.valid && materially_different(before, after)) {
 frame.estimate.correction_transition_valid = true;
 transform_odom_pose(
 before, odom_base,
 frame.estimate.before_correction_x_m,
 frame.estimate.before_correction_y_m,
 frame.estimate.before_correction_yaw_rad);
 transform_odom_pose(
 after, odom_base,
 frame.estimate.after_correction_x_m,
 frame.estimate.after_correction_y_m,
 frame.estimate.after_correction_yaw_rad);
 }
 }
 return frame;
}

}  // namespace lost_signal::sim::viewer
