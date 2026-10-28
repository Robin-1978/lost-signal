#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robot/core/geometry.hpp"
#include "robot/core/time.hpp"
#include "robot/core/transform.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

// Shared implementation capacity for bounded scan-hypothesis generation.
// Public search policies still decide how many hypotheses are accepted; this
// only keeps the local matcher, SLAM and global search from drifting into
// unrelated hard-coded storage limits.
inline constexpr std::size_t kMaximumScanMatchHypotheses = 24;

// Deterministic known-map scan matcher: planar LiDAR scan vs static raster,
// searched around an odometric prior. No truth, no actor identities, no map
// mutation. Unknown cells score 0 (never false certainty); outside-map is
// evidence against the pose.
struct ScanMatcherConfig {
  double search_translation_m = 0.75;
  double search_heading_rad = 0.35;
  double translation_step_m = 0.05;
  double heading_step_rad = 0.05;
  double minimum_overlap = 0.30;
  // Best match must beat the next separated hypothesis by this margin.
  // Quantized endpoints tie exactly, so only separated hypotheses compete;
  // a small margin in repetitive aisles would drag the correction away.
  double minimum_score_margin = 0.05;
  std::size_t minimum_returns = 4;
  std::size_t maximum_returns = 96;
 // Survey rasters support expected-range verification. Synthetic one-scan
 // SLAM submaps contain splatted endpoints rather than complete surfaces and
 // disable this stage explicitly.
 bool expected_range_validation = true;
 // Local odometry-centred tracking may require a shifted winner to improve
 // on the prior by minimum_score_margin. Startup/global hypothesis trackers
 // leave this off because their centres are competing hypotheses, not an
 // already established continuous odometry pose.
 bool require_prior_improvement = false;
  // Two candidates closer than this are one physical hypothesis.
  double ambiguity_distance_m = 0.25;
  double ambiguity_heading_rad = 0.15;
  // Mount lives in the TF tree; set via set_lidar_mount(). No copy here.
  double max_range_m = 10.0;
};

struct ScanMatchResult {
  bool accepted = false;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double position_sigma_m = 0.05;
  double heading_sigma_rad = 0.02;
 // Observed pose information in map-frame [x,y,yaw] coordinates. Local
 // scan matching can be rank-deficient (for example, motion along a straight
 // corridor); zero eigenvalues deliberately leave those prior covariance
 // directions unchanged. Global/startup results may use the scalar fallback.
 std::array<double, 9> pose_information{};
 bool pose_information_valid = false;
 // A locally ambiguous likelihood ridge can still constrain the directions
 // shared by all tied hypotheses (for example wall-normal position and yaw
 // in a long corridor). `accepted` remains false so callers cannot treat it
 // as a full pose fix; this flag authorizes information-form fusion only.
 bool partial_observation_valid = false;
  double score = 0.0;
  double overlap = 0.0;
  double score_margin = 0.0;
  std::size_t usable_returns = 0;
  std::string rejection_reason = "unassessed";
};

// One prepared beam: validated range in sensor frame. Shared by the bounded
// local matcher and the whole-map global search so both score identically.
struct ScanBeam {
  double range = 0.0;  // clamped to max_range_m
  double angle = 0.0;  // sensor-frame angle, rad
  bool hit = false;    // true return (range < max_range_m)
};

struct PoseScore {
  double score = 0.0;
  double overlap = 0.0;
  std::size_t usable_returns = 0;
  // False when every beam scored exactly 0 (all unknown): no evidence either
  // way. Callers skip such hypotheses instead of ranking them at 0.
  bool has_evidence = false;
};

class ScanMatcher {
 public:
  explicit ScanMatcher(ScanMatcherConfig config = ScanMatcherConfig{});

  // Full lidar mount from the TF tree (coordinator fans out).
  void set_lidar_mount(const core::RigidTransform& base_from_lidar) {
    base_from_lidar_ = base_from_lidar;
  }

  // ranges: planar returns in sensor frame (angle_min + i*increment).
  // roll/pitch/z carry the slope attitude (estimate values at the call
  // site); hypotheses stay SE(2) — the raster is 2D. Defaults keep
  // flat-ground tests exact.
  ScanMatchResult localize(const std::vector<float>& ranges, double angle_min_rad,
                           double angle_increment_rad,
                           double odom_x_m, double odom_y_m,
                           double odom_heading_rad,
                           const iface::MapSnapshot& map,
                           core::MonotonicNs stamp_ns,
                           double body_roll_rad = 0.0,
                           double body_pitch_rad = 0.0,
                           double body_z_m = 0.0) const;

  // Filter + deterministic angular downsample to at most max_beams.
  // Empty output means no usable geometry (caller rejects).
  std::vector<ScanBeam> prepare_beams(const std::vector<float>& ranges,
                                      double angle_min_rad,
                                      double angle_increment_rad,
                                      std::size_t max_beams) const;
  // Score one SE(2) hypothesis with prepared beams; roll/pitch/z are the
  // slope attitude at the call site (estimate values). Pure function of
  // (pose, beams, map); the global search calls this per coarse candidate.
  PoseScore score_pose(double x_m, double y_m, double yaw_rad,
                       const std::vector<ScanBeam>& beams,
                       const iface::MapSnapshot& map,
                       double body_roll_rad = 0.0,
                       double body_pitch_rad = 0.0,
                       double body_z_m = 0.0) const;
 // Range-consistency score for a bounded endpoint shortlist. Survey maps
 // and accumulated SLAM submaps can distinguish two poses whose endpoints
 // land in the same thick occupied band by comparing the first expected hit
 // distance along each measured ray.
 PoseScore score_expected_ranges(
 double x_m, double y_m, double yaw_rad,
 const std::vector<ScanBeam>& beams,
 const iface::MapSnapshot& map,
 double body_roll_rad = 0.0,
 double body_pitch_rad = 0.0) const;

  // Batched scorer: same math as score_pose, pose-invariant work hoisted
  // (beam trig per batch, rotations per heading slot). Scores are
  // bitwise identical to score_pose — only redundant libm calls are gone.
  // Slots name heading cells (caller passes a stable slot per heading);
  // a slot re-fills when its yaw differs, so misuse degrades to today's
  // cost, never to wrong rotations.
  class Batch {
   public:
    Batch(const ScanMatcher& matcher, const std::vector<ScanBeam>& beams,
          double body_roll_rad, double body_pitch_rad);
    std::size_t usable_returns() const { return usable_returns_; }
    PoseScore score(std::size_t slot, double x_m, double y_m, double yaw_rad,
                    const iface::MapSnapshot& map, double body_z_m);

   private:
    struct BeamT {
      double range = 0.0, cb = 1.0, sb = 0.0;
      bool hit = false;
    };
    struct RotSet {
      bool ready = false;
      double yaw_rad = 0.0;
      core::Quaternion sensor_q{0.0, 0.0, 0.0, 1.0};
      core::Vec3 mount_off{0.0, 0.0, 0.0};
      std::vector<double> dx, dy, free_c, free_s;
    };
    RotSet& rot_for(std::size_t slot, double yaw_rad);
    const core::RigidTransform& mount() const { return *mount_; }
    const core::RigidTransform* mount_ = nullptr;
    double roll_rad_ = 0.0, pitch_rad_ = 0.0;
    std::vector<BeamT> beams_;
    std::size_t usable_returns_ = 0;
    std::vector<RotSet> rots_;
  };
  // Batch over prepared beams for one search call (roll/pitch fixed for
  // the call, as in every current caller).
  Batch begin_batch(const std::vector<ScanBeam>& beams, double body_roll_rad,
                    double body_pitch_rad) const {
    return Batch(*this, beams, body_roll_rad, body_pitch_rad);
  }

 private:
  ScanMatcherConfig config_;
  core::RigidTransform base_from_lidar_{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
};

}  // namespace lost_signal::robot::autonomy
