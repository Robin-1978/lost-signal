#pragma once

#include "content_catalog.hpp"
#include "hw/chassis/diff_drive.hpp"
#include "algo/dynamic_obstacle_map.hpp"
#include "algo/dwa.hpp"
#include "algo/eventlog.hpp"
#include "algo/follow.hpp"
#include "hw/sensors/hal.hpp"
#include "algo/grid_map.hpp"
#include "algo/slip_detect.hpp"
#include "hw/chassis/motor_drive.hpp"
#include "algo/pid.hpp"
#include "hw/sensors/sensor.hpp"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace game {

enum class MissionStatus { Ready, Running, Succeeded, Failed };

struct MissionEvent {
 double time_s{0.0};
 std::string code;
 std::string message;
};

// Gate-3 shadow wait episode (structured): one waiting interval with the
// first/last departure assessments and the actual waited time. reasons code
// sim::DepartReason (0=GoNow..5=Invalid). Stored alongside the event log so
// integrity (open/close pairing, field ranges) and distribution (counts and
// durations by reason) are machine-checkable, not parsed from prose.
struct ShadowEpisode {
 double start_s{0.0};
 double first_pred_s{-1.0};
 int first_reason{-1};
 double end_s{0.0};
 double last_pred_s{-1.0};
 int last_reason{-1};
 double waited_s{0.0};
 // Gate-B stratification: fastest known threat and nearest clearance at wait
 // start (max |v| over active tracks, meters and m/s, -1 = uncaptured).
 double entry_threat_speed_mps{-1.0};
 double entry_nearest_m{-1.0};
};

struct ActorSnapshot {
 std::string id;
 sim::Vec2 position;
 sim::Vec2 velocity;
 double radius_m{0.0};
 bool detected{false};
 // Renderers draw vehicles and pedestrians differently (no forklifts for
 // people); set from the agent kind, never inferred from size.
 bool pedestrian{false};
};

struct DynamicObstacleSnapshot {
 std::string id;
 sim::Vec2 estimated_position;
 sim::Vec2 estimated_velocity;
 double radius_m{0.0};
 double last_observed_s{0.0};
};

struct LocalTrajectorySnapshot {
 std::vector<sim::Vec2> points;
 bool collision{false};
 double score{0.0};
};

struct ControlSnapshot {
 bool dwa_active{false};
 sim::Vec2 local_target;
 std::vector<LocalTrajectorySnapshot> dwa_candidates;
 std::vector<sim::Vec2> dwa_selected;
 // Shadow departure assessment (gate 3): observed only, never acted on.
 // shadow_reason codes sim::DepartReason for the departure verdict
 // (assess_departure never emits kUnassessed/kClear), -1 = unassessed.
 bool shadow_assessed{false};
 double shadow_depart_s{-1.0};
 int shadow_reason{-1};
 // Spatial pocket shadow (Gate B): last sampling while waiting. verdict codes
 // sim::SearchVerdict (0=FoundFeasible/1=AllRefused/2=Incomplete), -1 = unsampled.
 int shadow_pocket_verdict{-1};
 int shadow_pocket_feasible{0};
 int shadow_pocket_total{0};
 // Gate-B reason distribution: counts of the three leg reasons per candidate
 // (transit+hold+rejoin), bins code sim::DepartReason 0..8. Sum == 3*total.
 int shadow_pocket_reasons[10]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 double heading_error_rad{0.0};
 double heading_pid_p{0.0};
 double heading_pid_i{0.0};
 double heading_pid_d{0.0};
 double target_v_mps{0.0};
 double target_w_radps{0.0};
 double left_target_mps{0.0};
 double right_target_mps{0.0};
 double left_measured_mps{0.0};
 double right_measured_mps{0.0};
 double left_target_rpm{0.0};
 double right_target_rpm{0.0};
 double left_measured_rpm{0.0};
 double right_measured_rpm{0.0};
 double left_pwm{0.0};
 double right_pwm{0.0};
 double left_speed_pid_p{0.0};
 double left_speed_pid_i{0.0};
 double left_speed_pid_d{0.0};
 double right_speed_pid_p{0.0};
 double right_speed_pid_i{0.0};
 double right_speed_pid_d{0.0};
 // Ackermann only: drive-unit values mirror into the left_* fields above and
 // the servo state lands here. Differential leaves these zero (never fiction).
 double steer_target_rad{0.0};
 double steer_angle_rad{0.0};
 double steer_pwm{0.0};
 double steer_left_rad{0.0};
 double steer_right_rad{0.0};
};

struct MissionSnapshot {
 double time_s{0.0};
 sim::DiffState robot;
 sim::Vec2 estimated_position;
 double estimated_heading{0.0};
 std::vector<ActorSnapshot> actors;
 std::vector<DynamicObstacleSnapshot> dynamic_obstacles;
 std::vector<sim::Vec2> lidar_hits;
 ControlSnapshot control;
 bool static_obstacle_detected{false};
 bool dynamic_object_detected{false};
 bool waiting_for_clearance{false};
 bool via_reached{false};
 // Ordered-goals progress: index of the next unreached goal in
 // objective.goals. via_reached stays synced for legacy UI/headless readers.
 std::size_t goals_reached{0};
 MissionStatus status{MissionStatus::Ready};
};

struct MissionResult {
 MissionStatus status{MissionStatus::Ready};
 std::string failure_reason;
 double time_s{0.0};
 double path_length_m{0.0};
 double energy_wh{0.0};
 double min_clearance_m{999.0};
 double min_static_clearance_m{999.0};
 double min_static_clearance_time_s{0.0};
 sim::Vec2 min_static_robot_position;
 double min_dynamic_clearance_m{999.0};
 double min_dynamic_clearance_time_s{0.0};
 sim::Vec2 min_dynamic_robot_position;
 sim::Vec2 min_dynamic_actor_position;
 double odometry_error_m{0.0};
 int collisions{0};
 int pedestrian_contacts{0};
 int slip_events{0};
 int astar_expanded{0};
 int local_decisions{0};
 bool via_reached{false};
 bool safe_score{false};
 bool fast_score{false};
 bool efficient_score{false};
};

class MissionSimulation {
 public:
 static constexpr double fixed_step_s = 0.02;

 MissionSimulation(const ContentCatalog& catalog, std::string level_id, BuildSelection build,
                   std::optional<uint64_t> seed_override = std::nullopt,
                   const std::vector<std::string>& owned_abilities = {});

 void reset();
 void start();
 void step();
 void run_to_completion();
 void set_manual_command(double forward, double turn);

 const LevelDefinition& level() const { return level_; }
 const MapDefinition& map_definition() const { return map_definition_; }
 const BuildSelection& build() const { return build_; }
 const RobotDefinition& robot() const { return robot_definition_; }
 static const std::string& level_robot_id(const ContentCatalog& catalog, const std::string& level_id,
                                            const BuildSelection& build);
 uint64_t effective_seed() const { return seed_override_.value_or(level_.seed); }
 const sim::GridMap& occupancy_map() const { return *truth_map_; }
 const std::vector<sim::Vec2>& planned_path() const { return planned_path_; }
 const MissionSnapshot& snapshot() const { return snapshot_; }
 const MissionResult& result() const { return result_; }
 const std::vector<MissionEvent>& events() const { return events_; }
 const std::vector<MissionSnapshot>& replay() const { return replay_; }
 const std::vector<ShadowEpisode>& shadow_episodes() const { return shadow_episodes_; }
 bool shadow_episode_open() const { return shadow_episode_open_; }
 int slip_episodes() const { return slip_detector_.episodes(); }
 const std::vector<double>& slip_episode_times() const { return slip_episode_times_; }

 private:
 void build_world();
 void plan_route();
 void update_actors();
 void scan_environment();
 void refresh_dynamic_obstacles();
 void decide_control();
 void apply_motion();
 void finish_success();
 void finish_failure(std::string reason, std::string message);
 void record_event(std::string code, std::string message, double throttle_s = 0.0);
 void update_shadow_episode();
 void record_replay();
 double surface_traction(sim::Vec2 position) const;
 double static_clearance(sim::Vec2 position) const;
 sim::Vec2 current_navigation_target();
 bool has(const std::string& slot) const;
 const ComponentDefinition& req(const std::string& slot) const;
 double spec(const std::string& slot, const std::string& key) const;
 double decision_hz() const;
 double corner_speed_cap(double nav_x, double nav_y) const;
 double track_max_age_s() const;
 sim::GroundContact wheel_ground(sim::Vec2 position) const;

 const ContentCatalog& catalog_;
 std::string level_id_;
 BuildSelection build_;
 // Seed override for multi-seed studies. Empty = level JSON seed (the only
 // seed official baselines ever use). Effective seed fans out to all three
 // RNG streams with the same xor constants as before.
 std::optional<uint64_t> seed_override_;
 const LevelDefinition& level_;
 const MapDefinition& map_definition_;
 const RobotDefinition& robot_definition_;
 // Resolved equipment, keyed by slot name. Anything slot-specific beyond
 // this point is a layering bug: use has()/req()/spec() instead of members.
 std::map<std::string, const ComponentDefinition*> parts_;
 double gyro_bias_{0.0};
 bool was_saturated_{false};
 // Continuous driving style, piecewise-interpolated across the policy anchors.
 Properties behavior_;

 std::unique_ptr<sim::GridMap> truth_map_;
 // Sensing goes through the HAL even in simulation: the same call path a
 // hardware backend will serve. Rebuilt with the world in build_world().
 std::unique_ptr<sim::LidarBackend> lidar_;
 std::unique_ptr<sim::GridMap> collision_map_;
 std::unique_ptr<sim::GridMap> planning_map_;
 std::unique_ptr<sim::DiffDrive> drive_;
 std::unique_ptr<sim::ChassisModel> chassis_;
 std::unique_ptr<sim::PID> heading_pid_;
 double heading_kp_{0.0};
 // Yaw-rate capability used by DWA/manual/clamps. Diff: DiffDrive w_max.
 // Ackermann: v*tan(steer_max)/base at top drive speed. Hardware, not tuned.
 double yaw_cap_rads_{2.0};
 sim::DeadReckon odometry_;
 sim::Rng rng_;
 sim::Rng perception_rng_;
 sim::Rng imu_rng_;
 sim::GuideState guide_state_;
 std::vector<sim::Vec2> planned_path_;
 std::vector<sim::Patrol> patrols_;
 std::vector<std::string> patrol_ids_;
 // Per-patrol clock offset: active-from-start patrols run on wall-clock (0);
 // summoned patrols run on (now - t0) so the encounter reproduces from the
 // summon moment. Same index as patrols_.
 std::vector<double> patrol_t0_;
 // Pedestrian wanders: same pattern as patrols (clock offset + dormant
 // summoning). Active agents read patrols-then-wanders; actual_pedestrian_
 // parallels actual_actors_ (true = social agent: eats clearance, never fails).
 std::vector<sim::Wander> wanders_;
 std::vector<std::string> wander_ids_;
 std::vector<double> wander_t0_;
 std::vector<sim::Wander> dormant_wanders_;
 std::vector<std::string> dormant_wander_ids_;
 std::vector<bool> actual_pedestrian_;
 // Dormant until a "summon" goal names them: invisible, non-collidable,
 // unplannable. Parallel to patrols_ once summoned.
 std::vector<sim::Patrol> dormant_patrols_;
 std::vector<std::string> dormant_ids_;
 std::vector<sim::DynState> actual_actors_;
 std::vector<sim::Vec2> dynamic_centroid_;
 std::vector<int> confirm_count_;
 sim::DynamicObstacleMap dynamic_obstacle_map_;

 MissionSnapshot snapshot_;
 MissionResult result_;
 std::vector<MissionEvent> events_;
 std::vector<ShadowEpisode> shadow_episodes_;
 std::vector<MissionSnapshot> replay_;
 // Shadow slip instrument (tracked only, see apply_motion): detector state
 // plus previous-step speeds. Observation only; no consumer yet (consumer
 // design is a behavior gate, not smuggled in here).
 sim::SlipDetector slip_detector_;
 std::vector<double> slip_episode_times_;
 double slip_prev_tread_v_{0.0};
 double slip_prev_body_v_{0.0};
 bool slip_have_prev_{false};
 sim::DiffCmd held_command_;
 // Shadow wait episode bookkeeping (gate 3): first predicted departure when
 // a wait opens, vs the last assessment and the actual waited time when it
 // closes. Observation only; no control path reads these.
 bool shadow_episode_open_{false};
 double shadow_episode_start_s_{0.0};
 double shadow_episode_first_s_{-1.0};
 int shadow_episode_first_reason_{-1};
 double shadow_episode_entry_speed_{-1.0};
 double shadow_episode_entry_nearest_{-1.0};
 // Staging for the next episode open (filled in the DWA branch where tracks
 // are visible, consumed by update_shadow_episode on the rising edge).
 double shadow_open_speed_{-1.0};
 double shadow_open_nearest_{-1.0};
 double next_scan_s_{0.0};
 double next_decision_s_{0.0};
 double next_replay_s_{0.0};
 double last_event_time_s_{-1000.0};
 std::string last_event_code_;
 bool manual_control_active_{false};
 sim::DiffCmd manual_command_;
};

const char* mission_status_name(MissionStatus status);

} // namespace game
