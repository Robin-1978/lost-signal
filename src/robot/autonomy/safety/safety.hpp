#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/core/geometry.hpp"
#include "robot/core/time.hpp"
#include "robot/core/transform.hpp"
#include "robot/iface/channel.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/observation.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::autonomy {

// Independent safety authority, last gate before publication.
// It re-checks RAW evidence (scan + freshness + channel health), never
// planner state, and can only ever downgrade a command to safe stop.
// Critical faults latch into STOPPED and need deliberate recovery
// (clear_safety, or full shutdown -> configure -> activate).
enum class SafetyState {
  kNominal = 0,
  kDegraded = 1,  // transient: motion blocked, auto-recovers on fresh evidence
  kStopped = 2,   // latched: stays until deliberate recovery
};

struct SafetyVerdict {
 SafetyVerdict() = default;
 SafetyVerdict(bool allow, SafetyState safety_state, std::string code)
 : allow_motion(allow), state(safety_state), reason(std::move(code)) {}
 // ROBOT-side clearance diagnostics from the independent raw-scan gate.
 // Values remain +inf until a nonzero command is projected. They expose
 // measured geometry only; no map or simulator truth enters these fields.
 double static_hold_clearance_m = std::numeric_limits<double>::infinity();
 double static_command_min_clearance_m =
 std::numeric_limits<double>::infinity();
 double static_command_terminal_clearance_m =
 std::numeric_limits<double>::infinity();
 double dynamic_hold_clearance_m = std::numeric_limits<double>::infinity();
 double dynamic_command_clearance_m =
 std::numeric_limits<double>::infinity();
 // Exact ROBOT-side collision-monitor envelope and base-frame centerline used
 // to validate the proposed command through reaction + braking. Diagnostics
 // and teaching overlays may render this evidence; it contains no map or SIM
 // truth and never feeds back into planning.
 double static_command_envelope_radius_m = 0.0;
 std::vector<core::Vec2> static_command_sweep;
  bool allow_motion = false;
  SafetyState state = SafetyState::kStopped;
  std::string reason;  // stable code for logs/tests
};

class ISafetyMonitor {
 public:
  virtual ~ISafetyMonitor() = default;
  virtual void reset() = 0;
  virtual SafetyVerdict evaluate(const EstimatedState& est,
                                 const PerceptionResult& perception,
                                 const iface::ObservationFrame& obs,
                                 const iface::ChannelHealth& channel,
                                 core::MonotonicNs now_ns) = 0;
  virtual iface::MotionCommand gate(const iface::MotionCommand& cmd,
                                    const SafetyVerdict& verdict,
                                    core::MonotonicNs now_ns) = 0;
  virtual void clear_safety() = 0;  // deliberate recovery from a latch
  virtual SafetyVerdict last_verdict() const = 0;
 virtual void set_lidar_mount(const core::RigidTransform& mount) {
 (void)mount;
 }
 virtual void set_lidar_yaw(double yaw_rad) { (void)yaw_rad; }
};

// v1 rules (all independent of planner/validator state):
//   - stale observation (> timeout)                 -> STOPPED, latched
//   - any raw return inside the configured footprint -> STOPPED, latched
//   - forward-sector clearance below braking need    -> DEGRADED, transient
//   - a command whose braking sweep hits the raw scan -> DEGRADED, transient
//   - channel drops observed since last check       -> DEGRADED, transient
//   - missing scan evidence                         -> DEGRADED, transient
//   - otherwise                                     -> NOMINAL
// The raw-scan sweep is deliberately independent of the rolling costmap and
// planner. It uses base-frame geometry, so global localization error cannot
// move an obstacle relative to the body.
struct SafetyConfig {
  core::MonotonicNs stale_timeout_ns = 500'000'000;
  float near_field_m = 0.3f;       // braking margin ahead of motion
  // Declared chassis envelope. The coordinator replaces this default from
  // RobotDescription; SIM truth is never consulted.
  double footprint_radius_m = 0.4;
  // Padding for lidar angular/range resolution and tracking error. This is
  // local sensor geometry, not a global-localization covariance term.
 double static_margin_m = platform::kDefaultNavigationClearanceMarginM;
  double brake_decel_mps2 = 2.0;
  double forward_half_angle_rad = 0.5;
 // Once a front veto has stopped the base, keep the reverse escape active
 // until the measured front sector has cleared this release distance. This
 // hysteresis is expressed in the same near-field units as the braking
 // policy, so scan jitter cannot immediately hand control back to a route
 // that still points at the obstacle.
 float recovery_release_m = 0.6f;
 // Dynamic tracks are ROBOT-owned scan estimates. A ready track with a
 // measured velocity is checked over this short horizon so a crossing actor
 // cannot appear safe merely because it is currently outside the forward
 // ray. No SIM actor identity or truth state enters this gate.
 double dynamic_lookahead_s = 1.0;
 double dynamic_speed_gate_mps = 0.1;
 // The independent gate may pass a slow minimum-risk command only when its
 // own constant-twist check improves predicted clearance over stopping.
 double dynamic_escape_gain_m = 0.03;
 double dynamic_escape_speed_cap_mps = 0.3;
 double dynamic_check_dt_s = 0.05;
 // Independent raw-scan collision monitor. A command is projected through
 // the controller/watchdog reaction interval, then to rest under the same
 // declared braking capability used by validation.
 double command_reaction_time_s = 0.2;
 double static_check_dt_s = 0.05;
 double static_escape_speed_cap_mps = 0.3;
 // Numerical allowance for comparing two projections of the same starting
 // clearance. This is not a physical safety margin; static_margin_m is.
 double static_escape_clearance_tolerance_m = 1e-4;
 // Transient proximity releases only after the padded footprint remains
 // clear for several live scan evaluations.
 double static_release_margin_m = 0.15;
 std::size_t static_release_cycles = 3;
 // Lidar extrinsics live in the TF tree; the full rotation is used when
 // classifying beam directions. The yaw setter remains for old callers.
};

class SafetyMonitor : public ISafetyMonitor {
 public:
  explicit SafetyMonitor(SafetyConfig config = {});
 void set_lidar_mount(const core::RigidTransform& mount) override {
 base_from_lidar_ = mount;
 }
 void set_lidar_yaw(double yaw_rad) override {
 if (std::isfinite(yaw_rad)) {
 base_from_lidar_ = {{0.0, 0.0, 0.0},
 core::quaternion_from_rpy(0.0, 0.0, yaw_rad)};
 }
 }
  void reset() override;
  SafetyVerdict evaluate(const EstimatedState& est,
                         const PerceptionResult& perception,
                         const iface::ObservationFrame& obs,
                         const iface::ChannelHealth& channel,
                         core::MonotonicNs now_ns) override;
  iface::MotionCommand gate(const iface::MotionCommand& cmd,
                            const SafetyVerdict& verdict,
                            core::MonotonicNs now_ns) override;
  void clear_safety() override;
  SafetyVerdict last_verdict() const override { return last_; }

 private:
 SafetyVerdict diagnostic_verdict(bool allow, SafetyState state,
 std::string reason) const;
  SafetyConfig config_;
 core::RigidTransform base_from_lidar_{{0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 1.0}};
  bool latched_ = false;
 bool reverse_recovery_ = false;
 bool static_proximity_ = false;
 std::size_t static_release_count_ = 0;
 bool dynamic_escape_required_ = false;
 EstimatedState dynamic_estimate_{};
 std::vector<DynamicTrack> dynamic_tracks_;
 // Raw scan endpoints in the current base frame after mount transform and
 // rolling-shutter deskew. No map pose, costmap, or simulator label enters
 // this independent last gate.
 std::vector<core::Vec2> raw_scan_points_;
  std::string latch_reason_;
  std::uint64_t last_drops_ = 0;
 // Directional minima from the last raw scan. The final gate uses the rear
 // value only to admit a validator-approved reverse escape while a front
 // proximity veto is active; no simulator state enters this path.
 float rear_min_m_ = std::numeric_limits<float>::infinity();
 float front_min_m_ = std::numeric_limits<float>::infinity();
 float omni_min_m_ = std::numeric_limits<float>::infinity();
  SafetyVerdict last_{false, SafetyState::kStopped, "init"};
};

}  // namespace lost_signal::robot::autonomy
