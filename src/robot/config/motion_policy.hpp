#pragma once

#include "robot/autonomy/localization/localization_supervisor.hpp"

#include <filesystem>
#include <string>

namespace sim {

// Deployable motion policy shared by the simulated and native robots.
//
// These numbers describe the robot's acceptance thresholds and operating
// envelope, not the simulator world. They therefore live under robot_data/ and
// are parsed by ROBOT, so a hardware integrator can retune them together with
// the real sensor and actuator calibration without touching game content or
// rebuilding the policy into the binary.
//
// Rules for this file:
// - every field name carries its unit (m, rad, s, ratio, or a [0,1] quality);
// - a default equals the validated simulation baseline, so an unmodified
//   deployment behaves the same as the regression suite;
// - a value that is a hardware capability (maximum speed, braking torque,
//   sensor range) belongs in the platform profile, never here.
struct LocalizationQualityPolicy {
 // Covariance scale at which the covariance term of the published quality
 // falls to one half. The value reflects the localizer's demonstrated pose
 // spread on the reference maps.
 double position_half_quality_sigma_m{0.25};
 double heading_half_quality_sigma_rad{0.10};
 // Blend between covariance consistency and matcher evidence. Both weights
 // are dimensionless fractions that must sum to one.
 double covariance_weight{0.70};
 double evidence_weight{0.30};
 // Time constant for confidence decay since the last accepted correction.
 double confidence_decay_time_constant_s{20.0};
 // Sigma beyond which the estimate is no longer published as valid.
 double untrusted_position_sigma_m{0.45};
 double untrusted_heading_sigma_rad{0.25};
};

// Speed envelope the local planner derives from localization health. The
// planner, not the executive, owns these consequences: the executive only sees
// the resulting motion-capability status.
struct LocalizationMotionAuthorityPolicy {
 // Health below which ordinary driving is withheld and only a validated
 // braking tail may be produced.
 double hold_confidence{0.15};
 double hold_max_age_s{30.0};
 // Confidence gates and the speed fraction retained above each gate.
 double reduced_confidence{0.35};
 double reduced_speed_scale{0.40};
 double cautious_confidence{0.60};
 double cautious_speed_scale{0.70};
};

// Runtime motion limits used by the deployable planner and safety controller.
// These are policy/calibration values, not map or game content.  Keeping them
// here gives a real platform one auditable place to retune sensor age,
// deadbands and stop-latch behavior without changing the SIM executable.
struct MotionExecutionPolicy {
 double gravity_mps2{9.81};
 double physical_standoff_reserve_m{0.10};
 double prediction_step_s{0.10};
 double dynamic_rollover_risk_limit{0.85};
 double planning_rest_deadband_mps{0.005};
 double planning_pose_velocity_deadband_mps{0.03};
 double committed_progress_floor_mps{0.15};
 double controlled_stop_release_mps{0.04};
 double creep_speed_mps{0.15};
 std::size_t controlled_stop_settle_cycles{3};
 std::size_t visible_dwa_candidates{24};
};

struct MotionPolicy {
 LocalizationQualityPolicy localization_quality;
 autonomy::LocalizationSupervisorConfig localization_supervisor;
 LocalizationMotionAuthorityPolicy localization_motion;
 MotionExecutionPolicy execution;

 // Baseline used by unit tests and by explicit offline tools. Runtime
 // composition should load the deployed file so the values stay auditable.
 static MotionPolicy defaults();

 // Strict loader: unknown keys, missing keys, wrong schema versions and
 // non-finite or out-of-range values are errors. A typo must fail the robot's
 // bring-up instead of silently keeping a compiled-in default.
 static MotionPolicy load(const std::filesystem::path& path);

 // Resolves the deployed policy file. LOST_SIGNAL_ROBOT_DATA wins; otherwise
 // the sibling directory of the content root is used, which keeps the source
 // tree and an installed copy working without extra configuration.
 static std::filesystem::path resolve_path(const std::filesystem::path& content_root);

 void validate() const;
};

} // namespace sim
