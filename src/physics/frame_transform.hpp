#pragma once

#include "physics_world.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace sim::physics {

// Rigid transforms use target_T_source semantics: translation and rotation
// express the source frame in the target frame.
struct RigidTransform {
 Vec3 translation_m;
 Quaternion rotation;
};

Quaternion quaternion_multiply(Quaternion lhs, Quaternion rhs);
Vec3 rotate_vector(Quaternion rotation, Vec3 vector);
Vec3 transform_point(const RigidTransform& target_from_source, Vec3 point_in_source);
RigidTransform compose(const RigidTransform& target_from_middle,
 const RigidTransform& middle_from_source);
RigidTransform inverse(const RigidTransform& target_from_source);

// Small deterministic TF-style graph used by the standalone simulator. It
// carries the same frame semantics as ROS TF without coupling core simulation
// to ROS or a wall clock.
class FrameTransformTree {
 public:
 void clear();
 void set_transform(std::string parent_frame, std::string child_frame,
 const RigidTransform& parent_from_child);
 std::optional<RigidTransform> lookup(const std::string& target_frame,
 const std::string& source_frame) const;
 std::vector<std::string> frame_ids() const;

 private:
 struct Edge {
 std::string frame;
 RigidTransform current_from_frame;
 };

 std::map<std::string, std::vector<Edge>> edges_;
};

} // namespace sim::physics
