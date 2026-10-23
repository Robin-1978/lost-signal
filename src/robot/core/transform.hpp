#pragma once

// Rigid transform math + a small deterministic TF-style graph. Ported from
// main's sim/hardware/physics/frame_transform (same frame semantics as ROS
// TF, no ROS, no wall clock), adapted to lost_signal::robot::core so both
// ROBOT (static mounts from RobotDescription) and SIM (dynamic edges) can
// share one implementation. Transform convention throughout is
// target_T_source: translation + rotation express the source frame in the
// target frame. RPY follows REP-103 (R = Rz(yaw) * Ry(pitch) * Rx(roll)).

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace lost_signal::robot::core {

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

struct Quaternion {
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
};

struct RigidTransform {
  Vec3 translation_m;
  Quaternion rotation;
};

Quaternion quaternion_multiply(Quaternion lhs, Quaternion rhs);
Vec3 rotate_vector(Quaternion rotation, Vec3 vector);
Vec3 transform_point(const RigidTransform& target_from_source,
                     Vec3 point_in_source);
RigidTransform compose(const RigidTransform& target_from_middle,
                       const RigidTransform& middle_from_source);
RigidTransform inverse(const RigidTransform& target_from_source);
// RPY (rad) to unit quaternion. Throws on non-finite input.
Quaternion quaternion_from_rpy(double roll_rad, double pitch_rad,
                               double yaw_rad);

class FrameTransformTree {
 public:
  void clear();
  // Records parent_T_child (and the inverse). Throws when either name is
  // empty or both names are equal: fail closed, never a degenerate edge.
  void set_transform(std::string parent_frame, std::string child_frame,
                     const RigidTransform& parent_from_child);
  // target_T_source across the chain, or nullopt when either frame is
  // unknown/disconnected. Identity when both names are equal.
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

}  // namespace lost_signal::robot::core
