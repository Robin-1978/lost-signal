#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robot/autonomy/localization/scan_matcher.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

// Deterministic coarse-to-fine whole-map search: kidnap recovery and cold
// start. The cold-start contract is bounded work over the complete map with
// no silent truncation:
//
//   1. coarse pass: every free raster cell on a coarse grid x headings,
//      scored with a downsampled beam subset (same scoring as the local
//      matcher, via ScanMatcher::score_pose);
//   2. refine: the top-K coarse survivors each get a bounded local match
//      around their pose;
//   3. accept the best refined hypothesis only with overlap and a margin
//      over the next accepted refine.
//
// If the evaluation budget cannot cover the coarse pass, the result is an
// explicit rejection, never a partial search reported as a match. Unknown
// cells never become false certainty (they score 0, as in the local path).
struct GlobalSearchConfig {
  double coarse_position_step_m = 1.0;
  double coarse_heading_step_rad = 0.35;
  std::size_t coarse_beams = 24;       // downsampled subset per coarse pose
  std::size_t coarse_hypotheses = 16;  // survivors sent to refinement
  // Upper bound on coarse pose evaluations. Exists to bound a pathological
  // map, not to make ordinary large maps unusable; exceeding it rejects
  // explicitly instead of reporting a partial search as a match.
  std::size_t maximum_evaluations = 8000000;
  double minimum_overlap = 0.30;
  double minimum_score_margin = 0.03;
  // Equivalent-support gate: a runner-up that explains essentially the same
  // fraction of occupied returns is unobservable even when discrete raster
  // scoring gives the winner a small numerical edge (e.g. map-edge gradients
  // along a uniform wall shift which border beams count as outside, while
  // genuine features move whole groups of endpoints into free space).
  // Calibrated: edge artifacts move <7% of explained returns; the
  // T-junction's stub/pillar/clutter move >14%. The 0.10 line sits between.
  double equivalent_support_overlap_tolerance = 0.10;
  // Mount lives in the TF tree; set via set_lidar_mount(). No copy here.
  double max_range_m = 10.0;
  std::size_t minimum_returns = 4;
  // Bounded refinement window around each coarse survivor.
  double refine_translation_m = 1.0;
  double refine_translation_step_m = 0.10;
  double refine_heading_rad = 0.35;
  double refine_heading_step_rad = 0.05;
  // Loose snap margin inside one refine window: the window's job is to snap
  // to the local optimum, not to prove uniqueness. Twin protection lives one
  // level up, in the cross-survivor comparison with minimum_score_margin:
  // survivors that refine to the same physical pose are merged, survivors
  // that refine to distinct poses must clear the strict margin.
  double refine_score_margin = 0.01;
  double refine_ambiguity_distance_m = 0.25;
  double refine_ambiguity_heading_rad = 0.15;
};

struct GlobalSearchResult {
  bool accepted = false;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double position_sigma_m = 0.05;
  double heading_sigma_rad = 0.02;
  double score = 0.0;
  double overlap = 0.0;
  double score_margin = 0.0;
  std::size_t usable_returns = 0;
  std::size_t coarse_evaluations = 0;
  std::string rejection_reason = "unassessed";
};

class GlobalSearcher {
 public:
  explicit GlobalSearcher(GlobalSearchConfig config = GlobalSearchConfig{});

  void set_lidar_mount(double x_m, double y_m, double yaw_rad) {
    mount_x_m_ = x_m;
    mount_y_m_ = y_m;
    mount_yaw_rad_ = yaw_rad;
  }

  // Whole-map search. The zero prior is only a finite API placeholder and
  // never constrains candidates; every free cell is searched.
  GlobalSearchResult search(const std::vector<float>& ranges,
                            double angle_min_rad,
                            double angle_increment_rad,
                            const iface::MapSnapshot& map,
                            core::MonotonicNs stamp_ns) const;

 private:
  GlobalSearchConfig config_;
  double mount_x_m_ = 0.0;
  double mount_y_m_ = 0.0;
  double mount_yaw_rad_ = 0.0;
};

}  // namespace lost_signal::robot::autonomy
