#pragma once

#include "robot/core/dynamic_object_state.hpp"
#include "robot/core/math.hpp"

#include <limits>
#include <vector>

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
 Vec2 stop_direction{1, 0};
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
 Vec2 stop_direction{1, 0};
 bool stop_valid{false};
};

struct GovernorOutput {
  bool hold_active{false};
  Vec2 stop_point{0, 0};
 // Unit tangent of the route at stop_point. Crossing behavior supplies it
 // so the virtual-wall consumer can distinguish along-route distance from
 // lateral localization error. Legacy geometric entry holds leave +x here
 // and continue to use Euclidean distance.
 Vec2 stop_direction{1, 0};
  bool stop_valid{false};
};

constexpr int kReleaseClears = 25; // 25 x 0.02 s = 0.5 s sustained Clear.
constexpr int kReleaseSilent = 15; // 15 x 0.02 s = 0.3 s without fresh data.

GovernorOutput govern_entry(GovernorState& state, const GovernorInput& input);

// Map-independent dynamic conflict assessment. The route starts at the
// measured ego position. All target states come from ROBOT perception and
// prediction; map conflict semantics and simulator actor identity are absent.
// The result is one-sided: it may request a hold, but never commands motion.
struct EntryWindowRequest {
 std::vector<Vec2> route;
 double cruise_speed_mps{0.0};
 std::vector<std::vector<DynamicObjectState>> predicted_frames;
 double frame_dt_s{0.0};
 double ego_radius_m{0.0};
 double clearance_m{0.0};
 double braking_deceleration_mps2{0.0};
};

struct DynamicEntryAssessment {
 GovernorInput governor;
 bool prediction_evidence_available{false};
 bool conflict{false};
 // True only when brake-and-hold at the derived point remains outside every
 // retained target occupancy over the prediction horizon. A conflict with no
 // safe hold is not converted into a zero-speed command; the local planner
 // must compare fully validated motion candidates against brake-and-hold.
 bool safe_hold_available{false};
 double minimum_gap_m{std::numeric_limits<double>::infinity()};
 double first_conflict_s{std::numeric_limits<double>::infinity()};
 Vec2 first_conflict_position;
 double stop_distance_m{std::numeric_limits<double>::infinity()};
};

DynamicEntryAssessment assess_dynamic_entry_window(
 const EntryWindowRequest& request);

// Intersects two one-sided behavior constraints before a no-return boundary.
// A mapped rule and an online, map-independent conflict detector are both
// allowed to withhold motion; neither can grant permission over the other.
// When both provide valid virtual walls, the first wall along the current
// route wins. An invalid held wall means "stop now" and is therefore the
// stricter result. The caller must not introduce a new stop after entering a
// separately validated no-return maneuver.
GovernorOutput merge_entry_constraints(
 const GovernorOutput& first, const GovernorOutput& second,
 const std::vector<Vec2>& route);

struct EntryWindowDecision {
 DynamicEntryAssessment assessment;
 GovernorOutput control;
};

// Stateful autonomy owner for runtime use. The pure transition above remains
// available for exhaustive contract tests; composition layers do not retain
// or mutate behavior state themselves.
class EntryGovernorSystem {
 public:
 void reset() { state_ = {}; }
 GovernorOutput update(const GovernorInput& input) {
 return govern_entry(state_, input);
 }
 EntryWindowDecision assess(const EntryWindowRequest& request) {
 EntryWindowDecision decision;
 decision.assessment = assess_dynamic_entry_window(request);
 decision.control = govern_entry(state_, decision.assessment.governor);
 return decision;
 }
 bool holding() const { return state_.hold_active; }

 private:
 GovernorState state_;
};

// Shared consumer translation (neutral Vec2 math, no game types): conflict
// center of the ego forward ray vs the threat velocity ray, clamped to
// lookaheads; parallel/degenerate falls back to the midpoint.
Vec2 governor_conflict_center(Vec2 ego, Vec2 ego_dir, Vec2 threat, Vec2 threat_vel);

} // namespace sim
