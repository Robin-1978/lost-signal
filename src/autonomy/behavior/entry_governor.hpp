#pragma once

#include "core/math.hpp"

namespace sim {

// Entry governor (Phase-2, Behavior layer output stage). Pure state machine:
// consumes per-step window verdicts, emits a one-sided HOLD_AT constraint.
// It NEVER emits a go-signal: absent hold means "DWA decides", never
// "proceed". One-sided brakes cannot command motion, only withhold it.
//
// Rules (documented behavior params, not tuned to any table):
// - Fresh Occupied -> hold ON immediately, streaks reset.
// - Fresh Clear -> clear streak++; hold OFF after kReleaseClears (0.5 s).
// - Stale/Uncertain (no fresh verdict) -> keep state, silent streak++;
//   release after kReleaseSilent (0.3 s, the estimator dropout budget):
//   tracks gone means nothing known to hold for; DWA native still guards.
// - The stop point is carried through while held; consumers clamp speed by
//   v <= sqrt(2*a*d) to the point (virtual wall, no score tampering).
//
// The caller translates tracks to verdicts (see assess_entry_window) and
// the stop point from route geometry; hysteresis lives here and only here.
struct GovernorInput {
  bool occupied_fresh{false};
  bool clear_fresh{false};
  Vec2 stop_point{0, 0};
  bool stop_valid{false};
};

struct GovernorState {
  bool hold_active{false};
  int clear_streak{0};
  int silent_streak{0};
 // The virtual wall belongs to the active hold, not to one observation
 // frame. A tracker gap must not erase the point while hysteresis still
 // says HOLD; doing so leaves a stopped robot inside the swept corridor.
 Vec2 stop_point{0, 0};
 bool stop_valid{false};
};

struct GovernorOutput {
  bool hold_active{false};
  Vec2 stop_point{0, 0};
  bool stop_valid{false};
};

constexpr int kReleaseClears = 25; // 25 x 0.02 s = 0.5 s sustained Clear.
constexpr int kReleaseSilent = 15; // 15 x 0.02 s = 0.3 s without fresh data.

GovernorOutput govern_entry(GovernorState& state, const GovernorInput& input);

// Shared consumer translation (neutral Vec2 math, no game types): conflict
// center of the ego forward ray vs the threat velocity ray, clamped to
// lookaheads; parallel/degenerate falls back to the midpoint.
Vec2 governor_conflict_center(Vec2 ego, Vec2 ego_dir, Vec2 threat, Vec2 threat_vel);

} // namespace sim
