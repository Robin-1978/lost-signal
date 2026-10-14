#pragma once

#include "content_catalog.hpp"
#include "sim/diff_drive.hpp"
#include "sim/dynamic_obstacle_map.hpp"
#include "sim/dwa.hpp"
#include "sim/eventlog.hpp"
#include "sim/follow.hpp"
#include "sim/grid_map.hpp"
#include "sim/motor_drive.hpp"
#include "sim/pid.hpp"
#include "sim/sensor.hpp"

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

struct ActorSnapshot {
 std::string id;
 sim::Vec2 position;
 sim::Vec2 velocity;
 double radius_m{0.0};
 bool detected{false};
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

 MissionSimulation(const ContentCatalog& catalog, std::string level_id, BuildSelection build);

 void reset();
 void start();
 void step();
 void run_to_completion();
 void set_manual_command(double forward, double turn);

 const LevelDefinition& level() const { return level_; }
 const MapDefinition& map_definition() const { return map_definition_; }
 const BuildSelection& build() const { return build_; }
 const sim::GridMap& occupancy_map() const { return *truth_map_; }
 const std::vector<sim::Vec2>& planned_path() const { return planned_path_; }
 const MissionSnapshot& snapshot() const { return snapshot_; }
 const MissionResult& result() const { return result_; }
 const std::vector<MissionEvent>& events() const { return events_; }
 const std::vector<MissionSnapshot>& replay() const { return replay_; }

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
 void record_replay();
 double surface_traction(sim::Vec2 position) const;
 double static_clearance(sim::Vec2 position) const;
 sim::Vec2 current_navigation_target();
 double decision_hz() const;
 double track_max_age_s() const;
 sim::GroundContact wheel_ground(sim::Vec2 position) const;

 const ContentCatalog& catalog_;
 std::string level_id_;
 BuildSelection build_;
 const LevelDefinition& level_;
 const MapDefinition& map_definition_;
 const RobotDefinition& robot_definition_;
 const ComponentDefinition& tires_;
 const ComponentDefinition* sensor_{nullptr};
 const ComponentDefinition* computer_{nullptr};
 const ComponentDefinition* motor_{nullptr};
 const ComponentDefinition* imu_{nullptr};
 double gyro_bias_{0.0};
 // Continuous driving style, piecewise-interpolated across the policy anchors.
 Properties behavior_;

 std::unique_ptr<sim::GridMap> truth_map_;
 std::unique_ptr<sim::GridMap> collision_map_;
 std::unique_ptr<sim::GridMap> planning_map_;
 std::unique_ptr<sim::DiffDrive> drive_;
 std::unique_ptr<sim::ChassisModel> chassis_;
 std::unique_ptr<sim::PID> heading_pid_;
 double heading_kp_{0.0};
 sim::DeadReckon odometry_;
 sim::Rng rng_;
 sim::Rng perception_rng_;
 sim::Rng imu_rng_;
 sim::GuideState guide_state_;
 std::vector<sim::Vec2> planned_path_;
 std::vector<sim::Patrol> patrols_;
 std::vector<std::string> patrol_ids_;
 std::vector<sim::DynState> actual_actors_;
 std::vector<sim::Vec2> dynamic_centroid_;
 std::vector<int> confirm_count_;
 sim::DynamicObstacleMap dynamic_obstacle_map_;

 MissionSnapshot snapshot_;
 MissionResult result_;
 std::vector<MissionEvent> events_;
 std::vector<MissionSnapshot> replay_;
 sim::DiffCmd held_command_;
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
