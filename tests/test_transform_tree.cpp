#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "robot/core/transform.hpp"
#include "robot/platform/description.hpp"
#include "robot/platform/transform_tree.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
bool near(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) <= tol;
}
}  // namespace

int main() {
  using namespace lost_signal::robot;

  // Quaternion identities: yaw-only rotation moves +x to +y.
  {
    const core::Quaternion q = core::quaternion_from_rpy(0.0, 0.0, kPi / 2);
    const core::Vec3 v = core::rotate_vector(q, {1.0, 0.0, 0.0});
    assert(near(v.x, 0.0) && near(v.y, 1.0) && near(v.z, 0.0));
    // Compose then invert round-trips to identity.
    const core::RigidTransform t{{1.0, 2.0, 3.0}, q};
    const core::RigidTransform back =
        core::compose(t, core::inverse(t));
    assert(near(back.translation_m.x, 0.0) && near(back.translation_m.y, 0.0) &&
           near(back.translation_m.z, 0.0));
    assert(near(back.rotation.w, 1.0));
    const core::Vec3 p = core::transform_point(t, {0.0, 0.0, 0.0});
    assert(near(p.x, 1.0) && near(p.y, 2.0) && near(p.z, 3.0));
  }

  // Chain lookup: map -> odom -> base resolves across two edges, and the
  // reverse lookup inverts it.
  {
    core::FrameTransformTree tree;
    tree.set_transform("map", "odom", {{1.0, 0.0, 0.0}, {0, 0, 0, 1}});
    tree.set_transform("odom", "base_link",
                       {{0.0, 2.0, 0.0},
                        core::quaternion_from_rpy(0.0, 0.0, kPi / 2)});
    const auto fwd = tree.lookup("map", "base_link");
    assert(fwd.has_value());
    const core::Vec3 p = core::transform_point(*fwd, {1.0, 0.0, 0.0});
    // base point (1,0,0): yaw 90 -> (0,1,0) in odom, + (0,2,0) offset,
    // + (1,0,0) map offset = (1,3,0).
    assert(near(p.x, 1.0) && near(p.y, 3.0) && near(p.z, 0.0));
    const auto rev = tree.lookup("base_link", "map");
    assert(rev.has_value());
    const core::Vec3 q = core::transform_point(*rev, p);
    assert(near(q.x, 1.0) && near(q.y, 0.0) && near(q.z, 0.0));
    // Unknown / empty frames fail closed; self lookup is identity.
    assert(!tree.lookup("map", "lidar_link").has_value());
    assert(!tree.lookup("", "map").has_value());
    const auto self = tree.lookup("odom", "odom");
    assert(self.has_value() && near(self->translation_m.x, 0.0) &&
           near(self->rotation.w, 1.0));
    // Re-setting an edge replaces it (latest wins, no stale duplicate).
    tree.set_transform("map", "odom", {{5.0, 0.0, 0.0}, {0, 0, 0, 1}});
    const auto upd = tree.lookup("map", "base_link");
    assert(upd.has_value());
    const core::Vec3 p2 = core::transform_point(*upd, {1.0, 0.0, 0.0});
    assert(near(p2.x, 5.0) && near(p2.y, 3.0));
    // Degenerate edges throw, never half-insert.
    bool threw = false;
    try {
      tree.set_transform("", "x", core::RigidTransform{});
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    assert(tree.frame_ids().size() == 3);
  }

  // SE(2) correction delta: identity when poses agree, exact hand values.
  {
    using lost_signal::robot::core::map_from_odom_delta;
    using lost_signal::robot::core::transform_point;
    auto id = map_from_odom_delta(1.0, 2.0, 0.3, 1.0, 2.0, 0.3);
    assert(near(id.translation_m.x, 0.0) && near(id.translation_m.y, 0.0) &&
           near(id.rotation.w, 1.0));
    // Pure translation, no rotation: delta is the difference.
    auto tr = map_from_odom_delta(1.0, 0.0, 0.0, 4.0, 0.0, 0.0);
    const core::Vec3 pt = transform_point(tr, {1.0, 0.0, 0.0});
    assert(near(pt.x, 4.0) && near(pt.y, 0.0));
    // Pure yaw about the origin.
    auto yw = map_from_odom_delta(0.0, 0.0, 0.0, 0.0, 0.0, kPi / 2);
    const core::Vec3 pr = transform_point(yw, {1.0, 0.0, 0.0});
    assert(near(pr.x, 0.0) && near(pr.y, 1.0));
    // Combined: pre (1,0,0) corrected to (1,0,pi/2) rotates the point
    // around the corrected origin: t = post - R(d)*pre = (1,-1), and the
    // pre origin lands exactly on post.
    auto cb = map_from_odom_delta(1.0, 0.0, 0.0, 1.0, 0.0, kPi / 2);
    const core::Vec3 pc = transform_point(cb, {1.0, 0.0, 0.0});
    assert(near(pc.x, 1.0) && near(pc.y, 0.0));
    const core::Vec3 qc = transform_point(cb, {0.0, 0.0, 0.0});
    assert(near(qc.x, 1.0) && near(qc.y, -1.0));
  }

  // Static tree from the robot description: links compose, bad links skip.
  {
    platform::RobotDescription desc;
    desc.links = {{"base_link", "world", {0, 0, 0}, {0, 0, 0}},
                  {"lidar_link", "base_link", {0.3, 0.0, 0.5}, {0, 0, 0}},
                  {"", "base_link", {0, 0, 0}, {0, 0, 0}}};  // skipped
    const core::FrameTransformTree tree =
        platform::static_transform_tree(desc);
    const auto base_from_lidar = tree.lookup("base_link", "lidar_link");
    assert(base_from_lidar.has_value());
    // Lidar origin in base frame: the mount translation itself.
    const core::Vec3 o =
        core::transform_point(*base_from_lidar, {0.0, 0.0, 0.0});
    assert(near(o.x, 0.3) && near(o.y, 0.0) && near(o.z, 0.5));
 // The description's world->base root marker is deliberately absent from
 // ROBOT static TF. Runtime map->odom->base owns the moving root.
 assert(!tree.lookup("world", "lidar_link").has_value());
  }

  std::printf("transform_tree ok\n");
  return 0;
}
