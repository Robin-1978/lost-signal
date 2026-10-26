#pragma once

// Draw model for the viewer. Pure SIM-side data (truth + provider outputs
// already recorded in the episode step), no raylib dependency, so unit
// tests build frames without a window. The frontend projects world-space
// primitives through ViewTransform and draws them; 2D and 3D frontends
// share this model.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lost_signal::sim::viewer {

// Every diagnostic field belongs to one explicit authority. Keeping these
// records separate makes the UI provenance visible and prevents a truth
// value from being mistaken for a deployable ROBOT signal.
enum class FrameAuthority {
 kPhysicsTruth,
 kRobotObservation,
 kRobotEstimate,
 kRobotDecision,
 kGameEvaluation,
};

struct FrameTruthTelemetry {
 static constexpr FrameAuthority kAuthority = FrameAuthority::kPhysicsTruth;
 bool valid = false;
 double x_m = 0.0, y_m = 0.0, yaw_rad = 0.0;
 double forward_mps = 0.0, yaw_rate_radps = 0.0;
 std::size_t wheel_contacts = 0;
};

struct FrameObservationTelemetry {
 static constexpr FrameAuthority kAuthority =
 FrameAuthority::kRobotObservation;
 bool accepted = false;
 bool scan_valid = false;
 bool odometry_valid = false;
 bool imu_valid = false;
 std::uint64_t sequence = 0;
 double stamp_s = -1.0;
 std::uint64_t scan_sequence = 0;
 double scan_stamp_s = -1.0;
 // World overlay projection uses the ROBOT estimate, never WorldTruth.
 // The raw ranges remain the recorded sensor measurement.
 bool projection_valid = false;
 double projection_pose_stamp_s = -1.0;
 double projected_sensor_x_m = 0.0;
 double projected_sensor_y_m = 0.0;
 double projected_sensor_z_m = 0.0;
 std::size_t lidar_beams = 0;
 std::size_t lidar_returns = 0;
 std::string scan_source;
};

struct FrameEstimateTelemetry {
 static constexpr FrameAuthority kAuthority = FrameAuthority::kRobotEstimate;
 bool valid = false;
 bool localized = false;
 double stamp_s = -1.0;
 double map_x_m = 0.0, map_y_m = 0.0, map_yaw_rad = 0.0;
 double odom_x_m = 0.0, odom_y_m = 0.0, odom_yaw_rad = 0.0;
 double position_sigma_m = 0.0, heading_sigma_rad = 0.0;
 std::string localization_action;
};

struct FrameDecisionTelemetry {
 static constexpr FrameAuthority kAuthority = FrameAuthority::kRobotDecision;
 bool valid = false;
 std::uint64_t sequence = 0;
 double stamp_s = -1.0;
 std::string action_code;
 std::string reason_code;
 std::string constraint_code;
 std::string safety_state;
 std::string safety_reason;
 double command_forward_mps = 0.0;
 double command_yaw_rate_radps = 0.0;
 std::size_t route_point_count = 0;
 std::size_t moving_candidate_count = 0;
 double safety_static_envelope_radius_m = 0.0;
 // Map-frame centerline of the exact raw-scan collision-monitor sweep. It is
 // transformed from ROBOT decision evidence with the ROBOT estimate only.
 std::vector<std::pair<double, double>> safety_static_sweep;
};

struct FrameEvaluationTelemetry {
 static constexpr FrameAuthority kAuthority = FrameAuthority::kGameEvaluation;
 bool static_collision = false;
 bool dynamic_collision = false;
 bool terrain_collision = false;
 double static_clearance_m = 999.0;
 double dynamic_clearance_m = 999.0;
 double estimate_error_m = 0.0;
};

struct FrameLinkPose {
 bool valid = false;
 double x_m = 0.0, y_m = 0.0, z_m = 0.0;
 double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
};

struct FrameWheel {
  bool contact_valid = false;
  double contact_x_m = 0.0, contact_y_m = 0.0, contact_z_m = 0.0;
  double contact_normal_force_N = 0.0;
  // Continuous SIM presentation evidence. Slip comes from MuJoCo contact
  // point relative velocity when contact exists; the joint/body calculation
  // is a startup fallback. Neither value is exposed to ROBOT.
  double contact_speed_mps = 0.0;
  double slip_speed_mps = 0.0;
 double longitudinal_slip_mps = 0.0;
 double lateral_slip_mps = 0.0;
 // MuJoCo-resolved world vectors on the contact tangent plane. Loose-surface
 // displacement follows module motion; tire smoke follows actual slip.
 double contact_motion_x_mps = 0.0;
 double contact_motion_y_mps = 0.0;
 double contact_motion_z_mps = 0.0;
 double contact_slip_x_mps = 0.0;
 double contact_slip_y_mps = 0.0;
 double contact_slip_z_mps = 0.0;
 double friction_utilization = 0.0;
  std::string contact_material_id;
 std::string contact_surface_effect;
 double contact_fluid_depth_m = 0.0;
  double x_m = 0.0, y_m = 0.0;  // module position in the base frame
  double steer_rad = 0.0;       // module yaw (fixed wheels read 0)
  double spin_rad = 0.0;        // live joint angle (turns visibly)
 double radius_m = 0.1;
 double half_width_m = 0.04;
  bool slipping = false;        // engine traction flag: SIM debug/effects only
  // Mecanum rollers are NOT simulated state (analytic wrench, rollers not
  // meshed): static 45-degree tread stripes, honest decoration only.
 double spin_rate_radps = 0.0;
 double steer_rate_radps = 0.0;
 double duty = 0.0;
 bool driven = false;
 bool mecanum_rollers = false;
 // Exact MuJoCo wheel-body pose in the base frame. The joint angles above
 // remain useful diagnostics; 3D rendering uses this pose when valid.
 FrameLinkPose pose;
};

struct FrameCaster {
 double pivot_x_m = 0.0, pivot_y_m = 0.0;
 double x_m = 0.0, y_m = 0.0;
 double radius_m = 0.03;
  double trail_m = 0.02;
 double yaw_rad = 0.0; // MuJoCo swivel joint angle
 double spin_rad = 0.0; // MuJoCo passive wheel roll angle
 FrameLinkPose fork_pose;
 FrameLinkPose wheel_pose;
};

struct FrameBody {
 double forward_mps = 0.0;
 double lateral_mps = 0.0;
 double yaw_rate_radps = 0.0;
  double x_m = 0.0, y_m = 0.0, yaw_rad = 0.0;
  double half_length_m = 0.28;  // configured collision/visual shell
  double half_width_m = 0.25;
  double z_m = 0.0;            // engine height (3D frontend only)
  double roll_rad = 0.0;       // live body attitude (ramps tilt the viz)
  double pitch_rad = 0.0;
  // True sensor origin in world (body pose composed with the mount — the
  // ray fan starts here, never at the body center).
  double sens_x_m = 0.0, sens_y_m = 0.0, sens_z_m = 0.0;
  // Wheels in drive order (empty for skid-steer slabs, see tracked below);
  // module positions read the backend layout table, angles read truth.
  std::vector<FrameWheel> wheels;
 // Unpowered physical caster supports. They are SIM-only presentation state,
 // never drive channels, traction evidence, or ROBOT observations.
 std::vector<FrameCaster> casters;
 // Skid-steer tracks (no rolling wheels to draw): slabs sized from the
 // chassis params. Traction flags are visible only in SIM truth debug.
  bool tracked = false;
  // ROBOT estimate at the same tick (offline truth-vs-estimate overlay).
  // estimate_valid=false hides the ghost.
  bool estimate_valid = false;
  double est_x_m = 0.0, est_y_m = 0.0, est_yaw_rad = 0.0;
};

struct FrameBox {
 std::string box_id;
  double x0_m = 0.0, y0_m = 0.0, x1_m = 0.0, y1_m = 0.0;
  double height_m = 0.0;
  bool selected = false;
  int selected_corner = -1;  // 0..3 (min/min, max/min, max/max, min/max)
};

struct FrameActor {
  double x_m = 0.0, y_m = 0.0, radius_m = 0.4;
  double vx_mps = 0.0, vy_mps = 0.0;
 double heading_rad = 0.0;
  double height_m = 1.5;  // sensor cylinder height (vehicle 1.5, pedestrian 1.8)
  bool pedestrian = false;  // explicit SIM visual kind, never inferred by size
};

// Anonymous ROBOT-side perception and prediction. These points are rebuilt
// from the same DynamicTrack contract consumed by behavior, local planning,
// validation, and safety. They never use FrameActor or any SIM actor state.
struct FrameDynamicPredictionPoint {
 double time_s = 0.0;
 double x_m = 0.0, y_m = 0.0;
 double uncertainty_m = 0.0;
};

struct FrameDynamicHypothesis {
 std::size_t hypothesis_id = 0;
 std::vector<FrameDynamicPredictionPoint> points;
};

struct FrameDynamicTrack {
 static constexpr FrameAuthority kAuthority = FrameAuthority::kRobotEstimate;
 std::uint32_t track_id = 0; // anonymous ROBOT tracker id, never a SIM actor id
 double x_m = 0.0, y_m = 0.0;
 double vx_mps = 0.0, vy_mps = 0.0;
 double radius_m = 0.0;
 double uncertainty_m = 0.0;
 bool prediction_ready = false;
 std::vector<FrameDynamicHypothesis> hypotheses;
};

// Ground-material zone overlay (SIM truth paint, never ROBOT input): the
// frontend tints the ground per zone so slick/sand reads visually.
struct FrameZone {
  double x0_m = 0.0, y0_m = 0.0, x1_m = 0.0, y1_m = 0.0;
  std::string material_id;
 std::string surface_effect;
 double fluid_depth_m = 0.0;
};

// Terrain patch footprint (frontend samples heights itself for the mesh).
struct FramePatch {
  double x0_m = 0.0, y0_m = 0.0, x1_m = 0.0, y1_m = 0.0;
};

struct FramePlace {
  std::string place_id;
  double x_m = 0.0, y_m = 0.0, yaw_rad = 0.0;
  bool reached = false;
  bool selected = false;
};

// Contact evidence produced by MuJoCo and consumed only by presentation.
// The impulse is integrated over the public plant step; the ROBOT never
// receives this structure or any field derived from it.
struct FrameImpact {
 bool valid = false;
 double x_m = 0.0, y_m = 0.0, z_m = 0.0;
 double normal_impulse_Ns = 0.0;
 double relative_speed_mps = 0.0;
 bool spark_eligible = false;
 double spark_x_m = 0.0, spark_y_m = 0.0, spark_z_m = 0.0;
 double spark_normal_impulse_Ns = 0.0;
 double spark_relative_speed_mps = 0.0;
 std::string spark_robot_material;
 std::string spark_static_material;
};

struct Frame {
 FrameTruthTelemetry truth;
 FrameObservationTelemetry observation;
 FrameEstimateTelemetry estimate;
 FrameDecisionTelemetry decision;
 FrameEvaluationTelemetry evaluation;
  FrameBody body;
  std::vector<std::pair<double, double>> trail;  // past truth (x, y), oldest first
  std::vector<FrameBox> boxes;
  std::vector<FrameActor> actors;
 // ROBOT estimates are deliberately separate from SIM actor presentation.
 std::vector<FrameDynamicTrack> dynamic_tracks;
  std::vector<FramePlace> places;
  std::vector<FrameZone> zones;
  std::vector<FramePatch> patches;
  // ROBOT observation projected with the recorded ROBOT estimate. This is the
  // only scan shown during a live run.
  std::vector<std::pair<double, double>> lidar_hits;
 // Acquisition offsets parallel to lidar_hits (seconds relative to scan-end,
 // <= 0). Presentation uses the real sensor timing to show the sweep order;
 // these values are recorded ROBOT observations, not generated animation.
 std::vector<float> lidar_time_offsets_s;
  bool lidar_valid = false;
  // SIM truth-assisted projection for explicit debrief/evaluation views only.
  std::vector<std::pair<double, double>> truth_lidar_hits;
 std::vector<float> truth_lidar_time_offsets_s;
  bool truth_lidar_valid = false;
  bool static_collision = false;
  bool dynamic_collision = false;
  double static_clearance_m = 999.0;
 FrameImpact impact;
  std::uint64_t step = 0;
};

}  // namespace lost_signal::sim::viewer
