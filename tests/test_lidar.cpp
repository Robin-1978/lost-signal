#include <cassert>
#include <cstdio>

#include "robot/iface/message.hpp"
#include "robot/providers/real_lidar2d.hpp"
#include "sim/providers/lidar16.hpp"
#include "sim/providers/lidar2d.hpp"

int main() {
  using namespace lost_signal;

  // Sim2D: typed message, deterministic, valid from first poll.
  sim::providers::Sim2DLidarProvider sim2d("sim.lidar2d", 72, 10.0f);
  sim2d.set_step(3, 60'000'000);
  robot::iface::Lidar2DScan scan;
  assert(sim2d.poll(scan));
  assert(scan.valid());
  assert(scan.ranges_m.size() == 72);
  assert(sim2d.descriptor().backend == robot::iface::ProviderBackend::kSim);

  // Sim16: 16 rings x N azimuths, one rotation per poll.
  sim::providers::Sim16LidarProvider sim16("sim.lidar16", 8);
  sim16.set_step(1, 100'000'000);
  robot::iface::Lidar3DCloud cloud;
  assert(sim16.poll(cloud));
  assert(cloud.valid());
  assert(cloud.ring_count == 16);
  assert(cloud.points.size() == 16 * 8);

  // Real2D: same ISensorProvider seam, native backend, replay-fed.
  robot::providers::Real2DLidarProvider real("ro1.lidar",
                                             std::vector<float>(36, 4.0f),
                                             0.1f);
  assert(real.descriptor().backend == robot::iface::ProviderBackend::kNative);
  robot::iface::Lidar2DScan rscan;
  assert(!real.poll(rscan));  // nothing fed: no fabricated scan
  real.feed(200'000'000);
  assert(real.poll(rscan));
  assert(rscan.valid());
  assert(rscan.ranges_m.size() == 36);

  // Polymorphism: all three behind ISensorProvider<IMessage>-compatible use.
  robot::iface::IMessage* m = &scan;
  assert(m->valid() && m->sequence() == 1);

  std::printf("lidar ok\n");
  return 0;
}
