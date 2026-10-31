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

// Deterministic coarse-to-fine whole-map search for cold start and kidnap
// recovery. Every observable coarse position and heading is ranked with a
// likelihood field and a downsampled hit subset. The field tolerates coarse
// sampling error; bounded exact refinement and full expected-range raycasts
// validate the survivors before publication. Unknown cells remain eligible
// base-pose seeds because a sparse survey marks unauthored space unknown;
// only occupied base cells are excluded.
//
// Total search work scales with the observable map. Real-time callers bound
// work per tick through advance(); no fixed lifetime budget is allowed to
// turn an otherwise observable large map into a localization failure.
// Unknown cells never become false certainty (they score 0, as in the local
// path).
struct GlobalSearchConfig {
 // Deterministic raster cells prepared per localization tick. The
 // observability index excludes poses too far from mapped geometry before
 // the expensive heading/beam scoring pass.
 std::size_t preparation_cells_per_cycle = 262144;
  double coarse_position_step_m = 1.0;
 double coarse_heading_step_rad = 0.35;
 // Likelihood-field endpoint score used only to rank whole-map seeds. The
 // exact endpoint and raycast gates still validate the refined result.
 double coarse_score_sigma_m = 0.45;
  std::size_t coarse_beams = 24;       // downsampled subset per coarse pose
  std::size_t coarse_hypotheses = 16;  // survivors sent to refinement
  // A large bounded recovery ellipse needs a wider spatial beam than a
  // normal whole-map/startup search. Small bounded unit tests and tight
  // local recoveries retain the ordinary budget.
  std::size_t bounded_recovery_hypotheses = 48;
 // Non-maximum suppression keeps the bounded survivor set distributed over
 // distinct places instead of filling it with adjacent samples of one peak.
 double coarse_hypothesis_distance_m = 1.5;
 double coarse_hypothesis_heading_rad = 0.45;
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
 double equivalent_support_score_margin = 0.05;
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
 // Refinement clusters adjacent samples of the same local peak. This stage
 // only snaps each coarse seed; separated-place ambiguity is decided after
 // full raycast validation.
  double refine_ambiguity_distance_m = 0.25;
  double refine_ambiguity_heading_rad = 0.15;
 // Final candidates inside these bounds represent the same physical pose.
 // Ambiguity is measured only between separated place hypotheses.
 double global_ambiguity_distance_m = 0.75;
 double global_ambiguity_heading_rad = 0.70;
 // Refine beam budget (bounded local match per survivor): the window's job
  // is to snap to the local optimum, not to recount every return — coarse
 // already ranked with a subset. The default uses up to 96 returns so the
 // local peak is stable on sparse and asymmetric geometry.
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
 // Acquisition time of the frozen scan. A caller that advances the pose by
 // intervening odometry must replace this with the propagated state's time
 // before publishing the correction.
 core::MonotonicNs stamp_ns = core::kInvalidTime;
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
  // Effective geometric support normalized by minimum_returns.  It is a
  // diagnostic for telemetry and temporal candidate comparison; admission
  // still uses the matcher score/overlap/margin and the supervisor's
  // covariance innovation gate, so sparse but informative scans are not
  // rejected by an independent threshold.
  double geometric_support = 0.0;
  std::size_t coarse_evaluations = 0;
 // Cells touched while preparing the immutable map revision's occupied-area
 // prefix and likelihood distance field. The first search on a revision
 // collectively perform two raster passes; a completed cache reports zero
 // on later searches. A request that resumes interrupted preparation reports
 // only the cells it completed. This is diagnostic only.
 std::size_t map_preparation_cells = 0;
 // Coarse position diagnostics count raster seeds once, before headings.
 // They make exact observability pruning measurable without feeding it back
 // into localization policy.
 std::size_t coarse_positions_considered = 0;
 std::size_t coarse_positions_skipped_unobservable = 0;
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
 // scan and retains a non-owning view of one immutable, provider-owned map
 // revision; the caller must keep that revision alive until completion or
 // cancel(). advance() evaluates a deterministic bounded slice
 // and returns a result only when every coarse/refine/final candidate has
 // been considered. Splitting work changes latency, never ranking or gates.
 void begin(const std::vector<float>& ranges, double angle_min_rad,
 double angle_increment_rad, const iface::MapSnapshot& map,
 core::MonotonicNs stamp_ns, double body_roll_rad = 0.0,
 double body_pitch_rad = 0.0, double body_z_m = 0.0);
 // Recovery with an established map frame searches only the covariance
 // supported neighbourhood. This keeps a strong repeated structure on the
 // far side of a large map from displacing a still informative ROBOT prior.
 void begin_near(const std::vector<float>& ranges, double angle_min_rad,
 double angle_increment_rad, const iface::MapSnapshot& map,
 core::MonotonicNs stamp_ns, double center_x_m, double center_y_m,
 double radius_m, double center_heading_rad, double heading_radius_rad,
 double body_roll_rad = 0.0,
 double body_pitch_rad = 0.0, double body_z_m = 0.0);
 std::optional<GlobalSearchResult> advance(
 std::size_t work_budget = 8192);
 bool active() const;
 // Cooperative work keeps a non-owning view of one immutable map revision.
 // A caller must cancel/restart when the provider publishes another map.
 bool active_map_matches(const iface::MapSnapshot& map) const;
 void cancel();

 private:
 struct PreparedMap;
 struct WorkState;
  GlobalSearchConfig config_;
  core::RigidTransform base_from_lidar_{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
 mutable std::shared_ptr<PreparedMap> prepared_map_;
 std::unique_ptr<WorkState> work_;
};

}  // namespace lost_signal::robot::autonomy
