#pragma once

// Static TF edges from the robot description. Composes every LinkMount
// (parent -> link, xyz + RPY) into a FrameTransformTree, so extrinsics
// resolve from one source instead of being hand-carried per provider.
// Links with empty names or non-finite numbers are skipped, never
// half-inserted; an empty description yields an empty tree.

#include "robot/core/transform.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::platform {

core::FrameTransformTree static_transform_tree(const RobotDescription& desc);

}  // namespace lost_signal::robot::platform
