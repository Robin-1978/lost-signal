#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/autonomy/coordinator.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"
#include "robot/platform/description.hpp"
#include "robot/platform/transform_tree.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;

lost_signal::robot::platform::RobotDescription make_desc() {
  using namespace lost_signal::robot;
  platform::RobotDescription desc;
  desc.links = {{"base_link", "world", {0, 0, 0}, {0, 0, 0}},
                {"lidar_link", "base_link", {0.3, 0.0, 0.5}, {0, 0, 0}},
                {"imu_link", "base_link", {0.1, 0.0, 0.2}, {M_PI_2, 0, 0}}};
  platform::SensorMount lidar;
  lidar.sensor_id = "ro1.lidar";
  lidar.link = "lidar_link";
  platform::SensorMount imu;
  imu.sensor_id = "ro1.imu";
  imu.link = "imu_link";
  desc.sensors = {lidar, imu};
  return desc;
}

lost_signal::robot::iface::ObservationFrame make_imu_obs(
    int seq, lost_signal::robot::core::MonotonicNs stamp_ns, double gyro_y) {
  using namespace lost_signal::robot;
  iface::ObservationFrame obs;
  obs.sequence = static_cast<std::uint64_t>(seq);
  obs.stamp_ns = stamp_ns;
  obs.source = "t.imu";
  obs.frame = core::Frame::kImuLink;
  obs.imu_valid = true;
  obs.imu_gyro_y_radps = gyro_y;
  return obs;
}

lost_signal::robot::iface::ObservationFrame make_motion_obs(
 int seq, lost_signal::robot::core::MonotonicNs stamp_ns,
 double forward_mps) {
 using namespace lost_signal::robot;
 iface::ObservationFrame obs;
 obs.sequence = static_cast<std::uint64_t>(seq);
 obs.stamp_ns = stamp_ns;
 obs.source = "t.encoder";
 obs.frame = core::Frame::kBaseLink;
 obs.odometry_valid = true;
 obs.forward_mps = forward_mps;
 return obs;
}
}  // namespace

int main() {
  using namespace lost_signal::robot;

  // Static edges resolve sensor mounts from the description.
  {
    autonomy::AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    coord.set_static_frames(platform::static_transform_tree(make_desc()));
    const auto mount = coord.tf().lookup("base_link", "lidar_link");
    assert(mount.has_value());
    const core::Vec3 o =
        core::transform_point(*mount, {0.0, 0.0, 0.0});
    assert(std::abs(o.x - 0.3) < 1e-12 && std::abs(o.z - 0.5) < 1e-12);
  }

  // IMU rotation through the tree: a roll-90-mounted gyro_y is body yaw.
  // Without the tree the same frame's gyro_y is ignored and yaw never moves.
  {
    autonomy::AutonomyCoordinator with_tree;
    assert(with_tree.configure().ok && with_tree.activate().ok);
    with_tree.set_static_frames(platform::static_transform_tree(make_desc()));
    with_tree.ingest_observation(make_imu_obs(1, 1'000'000, 0.5));
    with_tree.ingest_observation(make_imu_obs(2, 101'000'000, 0.5));
    with_tree.update(iface::MapSnapshot{}, iface::ChannelHealth{},
                     101'000'000);
    const double yaw_with = with_tree.estimate().yaw_rad;
    assert(yaw_with > 0.01);  // 0.5 rad/s * 0.1 s rotated into yaw

    autonomy::AutonomyCoordinator bare;
    assert(bare.configure().ok && bare.activate().ok);
    bare.ingest_observation(make_imu_obs(1, 1'000'000, 0.5));
    bare.ingest_observation(make_imu_obs(2, 101'000'000, 0.5));
    bare.update(iface::MapSnapshot{}, iface::ChannelHealth{}, 101'000'000);
    assert(std::abs(bare.estimate().yaw_rad) < 1e-12);  // gyro_y unrotated
  }

  // Dynamic edges: mapless ticks keep map->odom identity and odom->base on
  // the estimate. The coordinator is the only odom publisher.
  {
    autonomy::AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    coord.set_static_frames(platform::static_transform_tree(make_desc()));
    coord.ingest_observation(make_imu_obs(1, 1'000'000, 0.5));
    coord.ingest_observation(make_imu_obs(2, 101'000'000, 0.5));
    coord.update(iface::MapSnapshot{}, iface::ChannelHealth{}, 101'000'000);
    const auto drift = coord.tf().lookup("map", "odom");
    assert(drift.has_value());
    assert(std::abs(drift->translation_m.x) < 1e-9 &&
           std::abs(drift->translation_m.y) < 1e-9 &&
           std::abs(drift->rotation.w - 1.0) < 1e-9);  // no correction
    const auto odom_base = coord.tf().lookup("odom", "base_link");
    assert(odom_base.has_value());
    const core::Vec3 b =
        core::transform_point(*odom_base, {0.0, 0.0, 0.0});
 assert(std::abs(b.x - coord.odometry_estimate().x_m) < 1e-9 &&
 std::abs(b.y - coord.odometry_estimate().y_m) < 1e-9);
 }

 // REP-105: an operator/relocalization map jump changes map->odom while
 // odom->base remains continuous. Their composition still equals map->base.
 {
 autonomy::AutonomyCoordinator coord;
 assert(coord.configure().ok && coord.activate().ok);
 coord.seed_pose(5.0, -2.0, 0.3);
 coord.ingest_observation(make_motion_obs(1, 1'000'000, 1.0));
 coord.ingest_observation(make_motion_obs(2, 101'000'000, 1.0));
 coord.update(iface::MapSnapshot{}, iface::ChannelHealth{}, 101'000'000);
 const auto odom_before = coord.tf().lookup("odom", "base_link");
 const auto map_odom_before = coord.tf().lookup("map", "odom");
 assert(odom_before.has_value() && map_odom_before.has_value());

 coord.seed_pose(-4.0, 3.0, -0.5);
 coord.update(iface::MapSnapshot{}, iface::ChannelHealth{}, 101'000'000);
 const auto odom_after = coord.tf().lookup("odom", "base_link");
 const auto map_odom_after = coord.tf().lookup("map", "odom");
 const auto map_base = coord.tf().lookup("map", "base_link");
 assert(odom_after.has_value() && map_odom_after.has_value() &&
 map_base.has_value());
 assert(std::abs(odom_after->translation_m.x -
 odom_before->translation_m.x) < 1e-12);
 assert(std::abs(odom_after->translation_m.y -
 odom_before->translation_m.y) < 1e-12);
 assert(std::abs(map_odom_after->translation_m.x -
 map_odom_before->translation_m.x) > 1.0);
 const core::Vec3 map_origin =
 core::transform_point(*map_base, {0.0, 0.0, 0.0});
 assert(std::abs(map_origin.x - coord.estimate().x_m) < 1e-9);
 assert(std::abs(map_origin.y - coord.estimate().y_m) < 1e-9);
  }

  std::printf("coordinator_tf ok\n");
  return 0;
}
