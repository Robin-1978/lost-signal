#include "robot/autonomy/behavior/entry_governor.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

GovernorOutput govern_entry(GovernorState& state, const GovernorInput& input) {
  const bool fresh = input.occupied_fresh || input.clear_fresh;
  if (input.occupied_fresh) {
    state.hold_active = true;
    state.clear_streak = 0;
    state.silent_streak = 0;
 if (input.stop_valid) {
 state.stop_point = input.stop_point;
 state.stop_valid = true;
 }
  } else if (input.clear_fresh) {
    state.silent_streak = 0;
    if (state.hold_active) {
      ++state.clear_streak;
      if (state.clear_streak >= kReleaseClears) {
        state.hold_active = false;
        state.clear_streak = 0;
 state.stop_valid = false;
      }
    }
  } else {
    // Stale/uncertain: keep state (never invent a verdict), but do not hold
    // the brake on silence forever — tracks gone means DWA native owns it.
    ++state.silent_streak;
    state.clear_streak = 0;
    if (state.silent_streak >= kReleaseSilent) {
      state.hold_active = false;
 state.stop_valid = false;
    }
  }
  GovernorOutput out;
  out.hold_active = state.hold_active;
 // A hold without a valid stop point is still reported (DWA clamps to zero),
 // but once a valid virtual wall exists it persists for the hold lifetime.
 out.stop_point = state.stop_point;
 out.stop_valid = state.hold_active && state.stop_valid;
  (void)fresh;
  return out;
}

Vec2 governor_conflict_center(Vec2 ego, Vec2 ego_dir, Vec2 threat, Vec2 threat_vel) {
  const double tv = std::hypot(threat_vel.x, threat_vel.y);
  Vec2 center{(ego.x + threat.x) * 0.5, (ego.y + threat.y) * 0.5};
  if (tv < 1e-6) return center;
  const Vec2 v{threat_vel.x / tv, threat_vel.y / tv};
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

} // namespace sim
