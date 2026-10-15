#include "algo/entry_shadow.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

} // namespace

EntryWindowAssessment assess_entry_window(double ego_dist_m, double ego_speed_mps,
                                          double conflict_half_len_m, const WindowThreat& threat,
                                          double stale_budget_s) {
  EntryWindowAssessment out;
  // Degenerate ego inputs: no assessment, not a silent go.
  if (!(ego_speed_mps > 0.0) || !(ego_dist_m >= 0.0) || !(conflict_half_len_m >= 0.0)) {
    out.ego_invalid = true;
    out.verdict = EntryVerdict::kObservedUncertain;
    return out;
  }
  out.ego_enter_s = ego_dist_m / ego_speed_mps;
  out.ego_exit_s = (ego_dist_m + 2.0 * conflict_half_len_m) / ego_speed_mps;
  // No threat data, or data too old to project: Uncertain, never Clear.
  if (!threat.valid || !(threat.age_s >= 0.0) || threat.age_s > stale_budget_s) {
    out.threat_enter_s = kInf;
    out.threat_exit_s = kInf;
    out.margin_s = 0.0;
    out.verdict = EntryVerdict::kObservedUncertain;
    return out;
  }
  const double zone_len = 2.0 * conflict_half_len_m;
  if (threat.speed_mps > 0.0) {
    // Approaching (or inside and still moving): entered at max(0, dist/u).
    out.threat_enter_s = std::max(0.0, threat.dist_to_zone_m / threat.speed_mps);
    out.threat_exit_s = (threat.dist_to_zone_m + zone_len) / threat.speed_mps;
    if (out.threat_exit_s < 0.0) {
      // Already fully past the zone: never comes back (constant-speed model).
      out.threat_enter_s = kInf;
      out.threat_exit_s = kInf;
    }
  } else if (threat.dist_to_zone_m < 0.0 &&
             threat.dist_to_zone_m + zone_len > 0.0) {
    // Stationary inside the zone: occupies it for the whole horizon.
    out.threat_enter_s = 0.0;
    out.threat_exit_s = kInf;
  } else {
    // Outside and not approaching: cannot arrive under this model.
    out.threat_enter_s = kInf;
    out.threat_exit_s = kInf;
  }
  const bool disjoint = (out.threat_exit_s <= out.ego_enter_s) || (out.threat_enter_s >= out.ego_exit_s);
  if (disjoint) {
    // Margin = closest approach of the two intervals (both orders handled).
    const double gap_after = out.threat_enter_s - out.ego_exit_s;
    const double gap_before = out.ego_enter_s - out.threat_exit_s;
    out.margin_s = std::max(gap_after, gap_before);
    out.verdict = EntryVerdict::kObservedClear;
  } else {
    out.margin_s = 0.0;
    out.verdict = EntryVerdict::kObservedOccupied;
  }
  return out;
}

const char* entry_verdict_name(EntryVerdict verdict) {
  switch (verdict) {
    case EntryVerdict::kObservedClear: return "ObservedClear";
    case EntryVerdict::kObservedOccupied: return "ObservedOccupied";
    case EntryVerdict::kObservedUncertain: return "ObservedUncertain";
  }
  return "ObservedUncertain";
}

} // namespace sim
