#include <cassert>
#include <cstdio>

#include "robot/iface/channel.hpp"
#include "robot/iface/endpoint.hpp"
#include "robot/platform/description.hpp"

int main() {
  using namespace lost_signal::robot;

  // Registry: one owner per (endpoint, role).
  iface::EndpointRegistry reg;
  assert(reg.register_endpoint({"a.lidar", iface::EndpointRole::kLidar,
                                iface::ProviderBackend::kSim, "lidar_link"}));
  assert(!reg.register_endpoint({"a.lidar", iface::EndpointRole::kLidar,
                                 iface::ProviderBackend::kNative,
                                 "lidar_link"}));
  assert(reg.owns("a.lidar", iface::EndpointRole::kLidar));
  assert(!reg.owns("a.lidar", iface::EndpointRole::kImu));

  // Channel: overflow drops oldest, keeps newest.
  iface::RuntimeChannel ch({.obs_capacity = 2, .cmd_capacity = 2});
  ch.activate();
  auto mk = [](std::uint64_t seq, core::MonotonicNs t) {
    iface::ObservationFrame o;
    o.sequence = seq;
    o.stamp_ns = t;
    o.source = "a.lidar";
    o.frame = core::Frame::kLidarLink;
    o.scan_valid = true;
    o.ranges_m = {1.0f};
    return o;
  };
  assert(ch.publish_observation(mk(1, 1)));
  assert(ch.publish_observation(mk(2, 2)));
  assert(ch.publish_observation(mk(3, 3)));  // evicts seq 1
  auto drained = ch.drain_observations();
  assert(drained.size() == 2);
  assert(drained[0].sequence == 2 && drained[1].sequence == 3);
  assert(ch.health().obs_dropped == 1);

  // Channel: stale sequence rejected.
  assert(!ch.publish_observation(mk(3, 4)));

  // Channel: command expiry -> safe stop.
  iface::MotionCommand cmd;
  cmd.space = iface::CommandSpace::kYawRate;
  cmd.stamp_ns = 100;
  cmd.expiry_ns = 200;
  cmd.forward_mps = 1.0;
  assert(ch.publish_command(cmd));
  iface::MotionCommand got = ch.take_command(150);
  assert(got.forward_mps == 1.0);
  assert(ch.publish_command(cmd));
  iface::MotionCommand late = ch.take_command(10'000);
  assert(late.is_safe_stop());
  assert(ch.health().cmd_expired == 1);

  // Description: versioned, hashable, no physics fields.
  platform::RobotDescription desc;
  desc.robot_id = "ro1";
  desc.version = 2;
  desc.links = {{"base_link", "world"}, {"lidar_link", "base_link"}};
  desc.sensors = {{"a.lidar", "lidar_link"}};
  desc.limits = {.max_forward_mps = 2.0,
                 .max_yaw_rate_radps = 2.0,
                 .footprint_radius_m = 0.4};
  assert(platform::valid_robot_description(desc));
  assert(platform::hash_robot_description(desc) != 0);
  platform::RobotDescription bad = desc;
  bad.robot_id = "";
  assert(!platform::valid_robot_description(bad));

  std::printf("transport ok\n");
  return 0;
}
