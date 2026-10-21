#pragma once

#include "robot/autonomy/localization/localization_provider.hpp"

#include <cstddef>

namespace sim {

// Deterministic coarse-to-fine global scan matcher for a surveyed map. This is
// a startup and recovery provider, not a replacement for continuous odometry
// or a claim of full AMCL probabilistic filtering.
//
// The cold-start contract is bounded work over the complete map. A sweep of a
// 200,000 m² map at full raster resolution would evaluate hundreds of millions
// of poses, so the first pass scores sampled endpoints against a distance field
// derived from the map, the second refines the retained place hypotheses on
// that field, and the third matches the survivors against the full raster. No
// stage silently truncates the map: if the configured evaluation budget cannot
// cover the first pass, the result is an explicit rejection.
struct GlobalMapLocalizationConfig {
 // Distance-field raster. It is deliberately coarser than the navigation map;
 // the final stage returns to full resolution.
 double coarse_field_resolution_m{0.20};
 double coarse_position_step_m{1.00};
 double coarse_heading_step_rad{0.35};
 double coarse_score_sigma_m{0.45};
 std::size_t coarse_returns{24};
 std::size_t coarse_hypotheses{16};
 // Middle refinement around each coarse place hypothesis.
 double refine_translation_m{1.00};
 double refine_translation_step_m{0.10};
 double refine_heading_step_rad{0.05};
 // Final refinement against the full-resolution occupancy raster.
 double final_translation_m{0.25};
 double final_translation_step_m{0.05};
 double final_heading_rad{0.06};
 double final_heading_step_rad{0.02};
 double minimum_overlap{0.30};
 double minimum_score_margin{0.03};
 // A second place hypothesis that explains essentially the same fraction of
 // occupied returns is unobservable even when discrete raster scoring gives
 // it a small numerical edge. Both values are explicit so a symmetric hall
 // cannot be accepted by an accidental tie-break.
 double equivalent_support_overlap_tolerance{0.05};
 double equivalent_support_margin{0.03};
 double ambiguity_distance_m{0.75};
 double ambiguity_heading_rad{0.70};
 std::size_t minimum_returns{12};
 std::size_t maximum_returns{96};
 // Upper bound on coarse place evaluations. It exists to bound a pathological
 // map, not to make ordinary large maps unusable; exceeding it rejects
 // explicitly instead of reporting a partial search as a match.
 std::size_t maximum_evaluations{8000000};
 double sensor_x_m{0.0};
 double sensor_y_m{0.0};
 double sensor_yaw_rad{0.0};
};

class GlobalMapLocalizer final : public autonomy::ILocalizationProvider {
 public:
 explicit GlobalMapLocalizer(GlobalMapLocalizationConfig config = {})
 : config_(config) {}

 autonomy::LocalizationResult localize(
 const Scan& scan, const Vec2& odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const override;

 // Explicit cold-start entry point. The map search is global; the zero prior
 // is only a finite API placeholder and is never used to constrain candidates.
 autonomy::LocalizationResult localize_unknown_start(
 const Scan& scan, const runtime::MapSnapshot& map,
 double timestamp_s) const;

 private:
 GlobalMapLocalizationConfig config_;
};

} // namespace sim
