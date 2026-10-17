#pragma once

// Shared track->window translation for field replay tests (test-only).
// Layering: assess_entry_window takes neutral numbers; the ray-ray conflict
// geometry (ego forward ray vs threat velocity ray) is the caller's
// translation, kept here so every replay test uses the same one.
#include "robot/autonomy/planning/validation/entry_shadow.hpp"
#include "sim/game/scenario/scenario_session.hpp"

#include <algorithm>
#include <cmath>

namespace field {

inline sim::Vec2 conflict_center(sim::Vec2 ego, sim::Vec2 ego_dir, sim::Vec2 threat,
                                 sim::Vec2 threat_vel) {
  const double tv = std::hypot(threat_vel.x, threat_vel.y);
  sim::Vec2 center{(ego.x + threat.x) * 0.5, (ego.y + threat.y) * 0.5};
  if (tv < 1e-6) return center;
  const sim::Vec2 v{threat_vel.x / tv, threat_vel.y / tv};
  const double denom = ego_dir.x * v.y - ego_dir.y * v.x;
  if (std::abs(denom) < 1e-6) return center;
  const double dx = threat.x - ego.x;
  const double dy = threat.y - ego.y;
  const double t = (dx * v.y - dy * v.x) / denom;
  const double s = (dx * ego_dir.y - dy * ego_dir.x) / denom;
  const double tc = std::max(0.0, std::min(t, 10.0));
  const double sc = std::max(0.0, std::min(s, 10.0));
  center = {(ego.x + ego_dir.x * tc + threat.x + v.x * sc) * 0.5,
            (ego.y + ego_dir.y * tc + threat.y + v.y * sc) * 0.5};
  return center;
}

struct StepAssessment {
  bool assessed{false};
  sim::EntryVerdict verdict{sim::EntryVerdict::kObservedUncertain};
  double margin_s{0.0};
};

// One threat track against one hypothetical ego pose. Freshness-gated by the
// caller-supplied age (stale -> unassessed, never a silent Clear).
inline StepAssessment assess_pose_vs_track(sim::Vec2 ego, double ego_heading, double ego_speed,
                                           sim::Vec2 track_pos, sim::Vec2 track_vel,
                                           double track_radius_m, double age_s) {
  StepAssessment out;
  if (age_s < 0.0 || age_s > sim::kEntryStaleBudgetS) return out;
  const sim::Vec2 dir{std::cos(ego_heading), std::sin(ego_heading)};
  const auto center = conflict_center(ego, dir, track_pos, track_vel);
  if (std::hypot(center.x - ego.x, center.y - ego.y) > 10.0) return out;
  const double half = 0.45 + track_radius_m + 0.2;
  sim::WindowThreat threat;
  threat.valid = true;
  threat.dist_to_zone_m = std::hypot(track_pos.x - center.x, track_pos.y - center.y) - half;
  const double to_center = std::max(1e-6, std::hypot(center.x - track_pos.x, center.y - track_pos.y));
  const double toward =
    (track_vel.x * (center.x - track_pos.x) + track_vel.y * (center.y - track_pos.y)) / to_center;
  threat.speed_mps = (toward > 0.0) ? std::hypot(track_vel.x, track_vel.y) : 0.0;
  threat.age_s = age_s;
  const double ego_dist = std::max(0.0, std::hypot(center.x - ego.x, center.y - ego.y) - half);
  const auto a = sim::assess_entry_window(ego_dist, ego_speed, half, threat, sim::kEntryStaleBudgetS);
  out.assessed = true;
  out.verdict = a.verdict;
  out.margin_s = a.margin_s;
  return out;
}

} // namespace field
