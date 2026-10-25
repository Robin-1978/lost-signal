#include "robot/platform/transform_tree.hpp"

#include <cmath>

namespace lost_signal::robot::platform {

core::FrameTransformTree static_transform_tree(const RobotDescription& desc) {
  core::FrameTransformTree tree;
  for (const auto& link : desc.links) {
    if (link.link.empty() || link.parent.empty() || link.link == link.parent) {
      continue;
    }
 // `world -> base_link` is a description root marker, not a calibrated
 // static transform. Runtime localization publishes map->odom and the
 // estimator publishes odom->base_link; inserting the root marker would
 // create a conflicting cycle and silently anchor a moving robot to world.
 if (link.link == "base_link" && link.parent == "world") continue;
    bool finite = true;
    for (int i = 0; i < 3; ++i) {
      finite = finite && std::isfinite(link.xyz_m[i]) &&
               std::isfinite(link.rpy_rad[i]);
    }
    if (!finite) continue;
    core::RigidTransform parent_from_link;
    parent_from_link.translation_m = {link.xyz_m[0], link.xyz_m[1],
                                     link.xyz_m[2]};
    parent_from_link.rotation = core::quaternion_from_rpy(
        link.rpy_rad[0], link.rpy_rad[1], link.rpy_rad[2]);
    tree.set_transform(link.parent, link.link, parent_from_link);
  }
  return tree;
}

}  // namespace lost_signal::robot::platform
