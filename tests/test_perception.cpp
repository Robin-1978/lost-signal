#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/mapping/mapper.hpp"

namespace {
lost_signal::robot::iface::ObservationFrame make_scan(
    lost_signal::robot::core::MonotonicNs stamp, float bg, int fg_lo,
    int fg_hi, float fg_range) {
  using namespace lost_signal::robot;
  iface::ObservationFrame o;
  o.sequence = static_cast<std::uint64_t>(stamp);
  o.stamp_ns = stamp;
  o.source = "t.lidar";
  o.frame = core::Frame::kLidarLink;
  o.odometry_valid = true;
  o.scan_valid = true;
  o.ranges_m.assign(72, bg);
  o.static_ranges_m.assign(72, bg);
  o.scan_static_valid = true;
  for (int i = fg_lo; i <= fg_hi; ++i) o.ranges_m[i] = fg_range;
  return o;
}

lost_signal::robot::autonomy::EstimatedState origin() {
  lost_signal::robot::autonomy::EstimatedState e;
  e.valid = true;
  e.stamp_ns = 1;
  return e;
}
}  // namespace

int main() {
  using namespace lost_signal::robot::autonomy;

  // Foreground bump vs empty reference -> one track near the bump.
  {
    LidarPerception perc;
    // Beams 34..38 of 72 span angles ~ -9deg..+9deg -> +x direction.
    auto obs = make_scan(1'000'000, 10.0f, 34, 38, 3.0f);
    PerceptionResult r = perc.update(obs, origin());
    assert(r.scan_ok);
    assert(r.tracks.size() == 1);
    assert(std::abs(r.tracks[0].x_m - 3.0) < 0.4);
    assert(std::abs(r.tracks[0].y_m) < 0.4);
    assert(!r.tracks[0].prediction_ready);  // single sighting: not ready
  }

  // Association: same bump next frame keeps identity, then goes ready.
  {
    LidarPerception perc;
    std::uint32_t id = 0;
    for (int k = 1; k <= 4; ++k) {
      auto obs = make_scan(k * 1'000'000, 10.0f, 34, 38, 3.0f);
      PerceptionResult r = perc.update(obs, origin());
      assert(r.tracks.size() == 1);
      if (id == 0) {
        id = r.tracks[0].id;
      } else {
        assert(r.tracks[0].id == id);
      }
    }
    auto obs = make_scan(5'000'000, 10.0f, 34, 38, 3.0f);
    PerceptionResult r = perc.update(obs, origin());
    assert(r.tracks[0].prediction_ready);
  }

  // Departed object: beams back to reference -> track ages out.
  {
    LidarPerception perc;
    for (int k = 1; k <= 2; ++k) {
      auto obs = make_scan(k * 1'000'000, 10.0f, 34, 38, 3.0f);
      perc.update(obs, origin());
    }
    for (int k = 3; k <= 6; ++k) {
      auto obs = make_scan(k * 1'000'000, 10.0f, 0, -1, 0.0f);  // no bump
      PerceptionResult r = perc.update(obs, origin());
      if (k <= 5) {
        (void)r;
      } else {
        assert(r.tracks.empty());  // 4 misses > max_misses=3
      }
    }
  }

  // TF mount: forward-shifted lidar moves the track +x by the mount.
  {
    LidarPerception perc;
    perc.set_lidar_mount(1.0, 0.0, 0.0);
    auto obs = make_scan(1'000'000, 10.0f, 34, 38, 3.0f);
    PerceptionResult r = perc.update(obs, origin());
    assert(r.tracks.size() == 1);
    assert(std::abs(r.tracks[0].x_m - 4.0) < 0.4);  // 3.0 range + 1.0 mount
  }

  // No static reference: classification unavailable, coverage retained.
  {
    LidarPerception perc;
    auto obs = make_scan(1'000'000, 10.0f, 34, 38, 3.0f);
    obs.scan_static_valid = false;
    obs.static_ranges_m.clear();
    PerceptionResult r = perc.update(obs, origin());
    assert(r.scan_ok);
    assert(r.tracks.empty());
  }

  // Mapper: background paints occupied, foreground clears but never paints.
  {
    RayClearingMapper mapper;
    auto obs = make_scan(1'000'000, 4.0f, 34, 38, 3.0f);
    EstimatedState est = origin();
    est.x_m = 0.0;
    est.y_m = 0.0;
    mapper.update(obs, est);
    LocalMap m = mapper.local_map();
    assert(m.valid);
    // Background beam straight back (-x is beam 0): endpoint occupied.
    // (Interior query point: exact cell corners are FP-fragile.)
    assert(m.is_blocked(-3.95, -0.05));
    // Mid-ray of that beam is cleared free.
    assert(m.is_free(-2.0, 0.0));
    // Foreground bump at ~3 m: ray cleared, endpoint NOT painted.
    assert(!m.is_blocked(3.0, 0.0));
    assert(m.is_free(2.0, 0.0));
    // Recenter on teleport: window follows, old content dropped.
    est.x_m = 100.0;
    mapper.update(obs, est);
    LocalMap m2 = mapper.local_map();
    assert(m2.valid);
    assert(!m2.is_blocked(4.0, 0.0));
  }

  std::printf("perception ok\n");
  return 0;
}
