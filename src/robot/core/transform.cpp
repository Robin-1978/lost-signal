#include "robot/core/transform.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::core {

namespace {

Quaternion normalized(Quaternion value) {
  const double norm = std::sqrt(value.x * value.x + value.y * value.y +
                                value.z * value.z + value.w * value.w);
  if (!(norm >= 1e-12) || !std::isfinite(norm)) {
    throw std::invalid_argument("frame transform quaternion must be non-zero");
  }
  return {value.x / norm, value.y / norm, value.z / norm, value.w / norm};
}

}  // namespace

Quaternion quaternion_multiply(Quaternion lhs, Quaternion rhs) {
  lhs = normalized(lhs);
  rhs = normalized(rhs);
  return normalized({
      lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
      lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
      lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
  });
}

Vec3 rotate_vector(Quaternion rotation, Vec3 vector) {
  rotation = normalized(rotation);
  const Vec3 q{rotation.x, rotation.y, rotation.z};
  const Vec3 twice_cross{
      2.0 * (q.y * vector.z - q.z * vector.y),
      2.0 * (q.z * vector.x - q.x * vector.z),
      2.0 * (q.x * vector.y - q.y * vector.x),
  };
  return {
      vector.x + rotation.w * twice_cross.x + q.y * twice_cross.z -
          q.z * twice_cross.y,
      vector.y + rotation.w * twice_cross.y + q.z * twice_cross.x -
          q.x * twice_cross.z,
      vector.z + rotation.w * twice_cross.z + q.x * twice_cross.y -
          q.y * twice_cross.x,
  };
}

Vec3 transform_point(const RigidTransform& target_from_source,
                     Vec3 point_in_source) {
  const Vec3 rotated =
      rotate_vector(target_from_source.rotation, point_in_source);
  return {target_from_source.translation_m.x + rotated.x,
          target_from_source.translation_m.y + rotated.y,
          target_from_source.translation_m.z + rotated.z};
}

RigidTransform compose(const RigidTransform& target_from_middle,
                       const RigidTransform& middle_from_source) {
  return {
      transform_point(target_from_middle, middle_from_source.translation_m),
      quaternion_multiply(target_from_middle.rotation,
                          middle_from_source.rotation),
  };
}

RigidTransform inverse(const RigidTransform& target_from_source) {
  const Quaternion rotation = normalized(target_from_source.rotation);
  const Quaternion inverse_rotation{-rotation.x, -rotation.y, -rotation.z,
                                    rotation.w};
  const Vec3 negative{-target_from_source.translation_m.x,
                      -target_from_source.translation_m.y,
                      -target_from_source.translation_m.z};
  return {rotate_vector(inverse_rotation, negative), inverse_rotation};
}

Quaternion quaternion_from_rpy(double roll_rad, double pitch_rad,
                               double yaw_rad) {
  if (!std::isfinite(roll_rad) || !std::isfinite(pitch_rad) ||
      !std::isfinite(yaw_rad)) {
    throw std::invalid_argument("rpy must be finite");
  }
  const double cr = std::cos(roll_rad * 0.5), sr = std::sin(roll_rad * 0.5);
  const double cp = std::cos(pitch_rad * 0.5), sp = std::sin(pitch_rad * 0.5);
  const double cy = std::cos(yaw_rad * 0.5), sy = std::sin(yaw_rad * 0.5);
  // R = Rz(yaw) * Ry(pitch) * Rx(roll), Hamilton convention.
  return normalized({
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy,
  });
}

RigidTransform map_from_odom_delta(double pre_x_m, double pre_y_m,
                                   double pre_yaw_rad, double post_x_m,
                                   double post_y_m, double post_yaw_rad) {
  const double d_yaw = wrap_angle(post_yaw_rad - pre_yaw_rad);
  const Quaternion rot = quaternion_from_rpy(0.0, 0.0, d_yaw);
  const Vec3 rotated = rotate_vector(rot, {pre_x_m, pre_y_m, 0.0});
  return {{post_x_m - rotated.x, post_y_m - rotated.y, 0.0}, rot};
}

void FrameTransformTree::clear() {
  edges_.clear();
}

void FrameTransformTree::set_transform(std::string parent_frame,
                                       std::string child_frame,
                                       const RigidTransform& parent_from_child) {
  if (parent_frame.empty() || child_frame.empty() ||
      parent_frame == child_frame) {
    throw std::invalid_argument(
        "frame transform needs two distinct named frames");
  }
  auto remove_edge = [](std::vector<Edge>& edges, const std::string& frame) {
    edges.erase(std::remove_if(edges.begin(), edges.end(),
                               [&](const Edge& edge) {
                                 return edge.frame == frame;
                               }),
                edges.end());
  };
  remove_edge(edges_[parent_frame], child_frame);
  remove_edge(edges_[child_frame], parent_frame);
  edges_[parent_frame].push_back({child_frame, parent_from_child});
  edges_[child_frame].push_back({parent_frame, inverse(parent_from_child)});
}

std::optional<RigidTransform> FrameTransformTree::lookup(
    const std::string& target_frame, const std::string& source_frame) const {
  if (target_frame.empty() || source_frame.empty()) {
    return std::nullopt;
  }
  if (target_frame == source_frame) {
    return RigidTransform{};
  }
  struct Visit {
    std::string frame;
    RigidTransform target_from_frame;
  };
  std::queue<Visit> pending;
  std::set<std::string> visited;
  pending.push({target_frame, {}});
  visited.insert(target_frame);
  while (!pending.empty()) {
    const Visit current = pending.front();
    pending.pop();
    const auto found = edges_.find(current.frame);
    if (found == edges_.end()) {
      continue;
    }
    for (const auto& edge : found->second) {
      if (!visited.insert(edge.frame).second) {
        continue;
      }
      const RigidTransform target_from_next =
          compose(current.target_from_frame, edge.current_from_frame);
      if (edge.frame == source_frame) {
        return target_from_next;
      }
      pending.push({edge.frame, target_from_next});
    }
  }
  return std::nullopt;
}

std::vector<std::string> FrameTransformTree::frame_ids() const {
  std::vector<std::string> output;
  output.reserve(edges_.size());
  for (const auto& [frame, unused] : edges_) {
    (void)unused;
    output.push_back(frame);
  }
  return output;
}

}  // namespace lost_signal::robot::core
