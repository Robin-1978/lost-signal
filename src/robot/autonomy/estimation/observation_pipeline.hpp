#pragma once

#include <cstddef>
#include <string>

#include "robot/autonomy/control/grip_adapter.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/estimation/measurement_synchronizer.hpp"
#include "robot/autonomy/localization/localization_system.hpp"
#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/mapping/traversal.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/core/transform.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Owns the complete ROBOT-side asynchronous observation flow. Transport
// envelopes are normalized into acquisition-time events, estimation consumes
// each new motion/IMU sample once, and perception/mapping run only on a new
// scan. Tick-time consumers receive one-shot localization and adaptation
// views. No simulator type or truth-side identity enters this subsystem.
class ObservationPipeline {
 public:
 void reset();

 void set_lidar_mount(const core::RigidTransform& base_from_lidar) {
 localization_.set_lidar_mount(base_from_lidar);
 }
 void set_startup_mode(StartupMode mode) {
 localization_.set_startup_mode(mode);
 }
 void mark_localized() { localization_.mark_localized(); }

 // Accepts one transport envelope and dispatches only newly acquired sensor
 // sections. Returns false for an invalid or fully stale envelope.
 bool ingest(const iface::ObservationFrame& observation,
 const core::FrameTransformTree& transforms, IEstimator& estimator,
 IPerception& perception, IMapper& mapper);

 struct TickResult {
 LocalizationSystem::TickResult localization;
 };

 // Runs the tick-time consumers of the cached asynchronous evidence. The map
 // is a ROBOT prior/SLAM product: unknown cells remain unknown to perception.
 TickResult update(const iface::MapSnapshot& map, core::MonotonicNs now_ns,
 IEstimator& estimator, IPerception& perception, IMapper& mapper,
 GripAdapter& grip, TraversalModel& traversal);

 bool has_observation() const { return measurements_.has_observation(); }
 bool has_scan() const { return measurements_.has_scan(); }
 bool has_motion() const { return measurements_.has_motion(); }
 bool has_imu() const { return measurements_.has_imu(); }
 const iface::ObservationFrame& latest() const {
 return measurements_.latest();
 }
 const PerceptionResult& perception() const { return perception_; }

 bool localized() const { return localization_.localized(); }
 const std::string& last_localization_action() const {
 return last_localization_action_;
 }
 std::size_t slam_keyframes() const { return localization_.slam_keyframes(); }
 std::size_t slam_loops_closed() const {
 return localization_.slam_loops_closed();
 }
 std::size_t slam_tracks() const { return localization_.slam_tracks(); }
 const SlamTrackResult& last_track_result() const {
 return localization_.last_track_result();
 }
 const SlamLoopResult& last_slam_result() const {
 return localization_.last_slam_result();
 }

 private:
 MeasurementSynchronizer measurements_;
 LocalizationSystem localization_;
 PerceptionResult perception_{};
 core::MonotonicNs last_traverse_ns_ = core::kInvalidTime;
 std::string last_localization_action_ = "init";
};

} // namespace lost_signal::robot::autonomy
