#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
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
//   1. coarse pass: every non-occupied raster cell on a coarse grid x
//      headings, scored with a downsampled beam subset (same scoring as the
//      local matcher, via ScanMatcher::score_pose). Unknown cells seed too:
//      live surveys mark everything outside authored boxes unknown (never
//      free), so free-only seeding would be vacuous there. Occupied cells
//      are the only excluded prior; unknown scores 0, never certainty.
//   2. refine: the top-K coarse survivors each get a bounded local match
//      around their pose;
//   3. validate: each refined survivor is re-checked in a tight window with
//      expected-range raycast scoring (would the map produce this scan
//      from this pose?), then the best validated hypothesis is accepted
//      only with overlap and a margin over the next accepted one.
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
  // Sparse-scan guard (master: 12). A handful of hits can match an
  // unrelated aisle by chance and refine alone; below this count the
  // search reports insufficient evidence instead of a confident fix.
  // Unknown-background surveys make this load-bearing: without it the
  // raycast stage never sees enough rays to disagree.
  std::size_t minimum_returns = 12;
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
  // Refine beam budget (bounded local match per survivor): the window's job
  // is to snap to the local optimum, not to recount every return — coarse
  // already ranked with a subset. 24 keeps the ambiguity gates green at a
  // third of the default cost (measured, not tuned).
 std::size_t refine_max_returns = 96;
  // Final raycast validation window around each refined survivor (master's
  // third pass). Endpoint scoring degrades gracefully on unknown cells
  // (unknown = 0, neutral), so a wrong aisle can outscore the true pose on
  // sparse evidence; the raycast stage re-asks every surviving hypothesis
  // the stronger question — would the map actually produce this scan from
  // this pose? — with expected-range marching that treats unknown as free
  // space to see through (occupied ends the ray). Tight window: refine
  // already snapped; this stage validates, it does not search.
  double final_translation_m = 0.25;
  double final_translation_step_m = 0.05;
  double final_heading_rad = 0.06;
  double final_heading_step_rad = 0.02;
  // Final beam budget (hits only, stride-downsampled like the coarse set):
  // the final stage recounts with evidence, so it may spend more beams
  // than the coarse ranking pass.
  std::size_t final_max_returns = 96;
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
  // Runner-up refined hypotheses (best-first after [0] = the published
  // pose above, at most 3): the startup tracker disambiguates these over
  // later scans instead of forcing a guess now. Empty unless refined
  // runner-ups exist (accepted or ambiguous alike).
  struct RunnerUp {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
    double score = 0.0;
  };
  std::vector<RunnerUp> runner_ups;
  std::size_t usable_returns = 0;
  std::size_t coarse_evaluations = 0;
  // Refine calls actually executed (post-dedupe survivors). Observability
  // only: bounded by coarse_hypotheses, never a decision input.
  std::size_t refined_poses = 0;
  std::string rejection_reason = "unassessed";
};

class GlobalSearcher {
 public:
  explicit GlobalSearcher(GlobalSearchConfig config = GlobalSearchConfig{});
 ~GlobalSearcher();

 GlobalSearcher(const GlobalSearcher&) = delete;
 GlobalSearcher& operator=(const GlobalSearcher&) = delete;
 GlobalSearcher(GlobalSearcher&&) noexcept;
 GlobalSearcher& operator=(GlobalSearcher&&) noexcept;

  void set_lidar_mount(const core::RigidTransform& base_from_lidar) {
    base_from_lidar_ = base_from_lidar;
  }

  // Whole-map search. roll/pitch/z carry the slope attitude (estimate
  // values); hypotheses stay SE(2). Defaults keep flat tests exact.
  GlobalSearchResult search(const std::vector<float>& ranges,
                            double angle_min_rad,
                            double angle_increment_rad,
                            const iface::MapSnapshot& map,
                            core::MonotonicNs stamp_ns,
                            double body_roll_rad = 0.0,
                            double body_pitch_rad = 0.0,
                            double body_z_m = 0.0) const;

 // Cooperative whole-map search for real-time callers. begin() freezes one
 // scan and map revision; advance() evaluates a deterministic bounded slice
 // and returns a result only when every coarse/refine/final candidate has
 // been considered. Splitting work changes latency, never ranking or gates.
 void begin(const std::vector<float>& ranges, double angle_min_rad,
 double angle_increment_rad, const iface::MapSnapshot& map,
 core::MonotonicNs stamp_ns, double body_roll_rad = 0.0,
 double body_pitch_rad = 0.0, double body_z_m = 0.0);
 std::optional<GlobalSearchResult> advance(
 std::size_t work_budget = 2048);
 bool active() const;
 void cancel();

 private:
 struct WorkState;
  GlobalSearchConfig config_;
  core::RigidTransform base_from_lidar_{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
 std::unique_ptr<WorkState> work_;
};

}  // namespace lost_signal::robot::autonomy
