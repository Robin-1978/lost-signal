#pragma once

#include "dynobs.hpp"

#include <limits>
#include <vector>

namespace sim {

// Predictor input: one tracked object plus what the tracker learned about its
// patrol corridor. The tracker owns observation/velocity/bounds estimation;
// the predictor owns only the future: constant-velocity extrapolation with
// reflection inside the observed corridor. Pure function of its arguments,
// no hidden state, so DWA and tests can call it without a tracker.
struct PredictInput {
 DynState state;
 double last_observed_s{0.0};
 Vec2 min_observed{0.0, 0.0};
 Vec2 max_observed{0.0, 0.0};
 bool bounded_x{false};
 bool bounded_y{false};
};

// Fixed-step future frames: frame k holds the predicted state at
// time_s + age + k * frame_dt_s for every input younger than max_age_s.
// Stale inputs (age > max_age_s) and future-dated inputs are skipped.
std::vector<std::vector<DynState>> predict_frames(const std::vector<PredictInput>& inputs, double time_s,
                                                  double horizon_s, double frame_dt_s, double max_age_s);

// Phase-aware gap acceptance: the minimum predicted separation between the
// ego vehicle (advancing along its path at speed_mps, holding at the end)
// and every predicted threat frame, minus radii.
// PARKED (not wired into arbitration): gates (conjunction, maturity, veto)
// and scored-waiting all failed the pinned finale — in a two-attractor
// encounter any wait-timing change flips seeds, and stop-in-place waiting
// cannot fix graze passes (east measured 8mm center-overlap). Next touchpoint
// is wait-aware planning (plan §4 item 16), not another bolt-on.
// Unit tests pin the math meanwhile.
// Returns +infinity when there is nothing predicted.
// Timed ego trajectory sample: position at absolute seconds since now, same
// time base as predict_frames (frame k stands at k*frame_dt_s).
struct TimedPose {
 double t_s{0.0};
 Vec2 p{0.0, 0.0};
};

// Swept same-time gap between an arbitrary timed ego motion and the threat
// frames. Generalizes predicted_gap (constant cruise + depart delay) to
// multi-phase executions: drive, hold at a pocket, rejoin. Ego positions are
// resampled at frame times, then swept exactly per interval — identical
// arithmetic to predicted_gap for constant-speed inputs (pinned equal).
// Coverage is the caller's duty: ego times outside [0, (N-1)*dt] clamp to the
// ends, so a hold past the last frame would hide behind it; assess-level
// wrappers must gate horizon coverage first (as assess_departure does).
double timed_gap(const std::vector<TimedPose>& ego,
                 const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                 double ego_radius_m);

double predicted_gap(const std::vector<Vec2>& path, std::size_t start_idx, double speed_mps,
                     const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                     double ego_radius_m, double depart_delay_s = 0.0);

// Earliest safe departure: smallest wait d >= 0 such that departing at now+d
// keeps the whole prediction horizon clear (via predicted_gap with delay).
// Returns +infinity when no departure within the horizon works. Phase-1a
// (observe only): the mission records it during waits without acting on it;
// action waits until the observed error (predicted vs actual release) proves
// small. Pure function, no behavior by itself.
double earliest_departure(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                           const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                           double ego_radius_m, double margin_m);

// Infinity is not one verdict: callers (and shadow logs) must distinguish
// why no departure is provable, because each cause belongs to a different
// owner — window sizing, Phase-2 waiting spots, or a genuinely blocked
// corridor. kHorizonShort/kWaitUnsafe/kBlockedInWindow are undecidable here,
// never "unsafe forever".
enum class DepartReason {
 kGoNow,          // wait_s == 0: depart now, whole trajectory proven.
 kWait,           // wait_s > 0: earliest provably safe departure.
 kBlockedInWindow,// horizon covers the traversal and the waiting spot stays
                  // safe, yet no wait clears the corridor.
 kWaitUnsafe,     // the waiting spot itself is swept by a threat within the
                  // horizon: staying is what kills. Phase-2 territory.
 kHorizonShort,   // the horizon cannot even cover the traversal: unknown,
                  // not unsafe (SOTIF Area 3: never file unknown as safe).
 kInvalid,        // degenerate input (empty path/frames, non-positive cruise).
 kUnassessed,     // capability does not exist yet (Phase-2 pocket/rejoin/detour):
                  // honestly Unknown, never borrowed from another cause.
};

struct DepartureAssessment {
 double wait_s{std::numeric_limits<double>::infinity()};
 DepartReason reason{DepartReason::kInvalid};
};

const char* depart_reason_name(DepartReason reason);

DepartureAssessment assess_departure(const std::vector<Vec2>& path, std::size_t start_idx,
                                     double cruise_mps,
                                     const std::vector<std::vector<DynState>>& frames,
                                     double frame_dt_s, double ego_radius_m, double margin_m);

// Phase-2 local-behavior candidates (plan item 16b). The kinds are frozen;
// only time feasibility is assessable today. Pocket/rejoin/detour need
// spatial planning that does not exist yet and report Unknown honesty.
enum class LocalCandidate {
 kProceed,     // depart now along the remaining path, no waiting.
 kWaitInPlace, // hold the current spot until the assessed departure, then proceed.
 kWaitAtPocket,// move aside, wait safe, rejoin. Phase-2 (unassessed).
 kRejoin,      // rejoin evaluation after a pocket wait. Phase-2 (unassessed).
 kDetour,      // existing local绕行. Phase-2 (DWA owns execution today).
};

struct Feasibility {
 enum class Verdict { kFeasible, kInfeasible, kUnknown };
 Verdict verdict{Verdict::kUnknown};
 DepartReason reason{DepartReason::kUnassessed};
 // Earliest departure when the time assessment produced one, else infinity.
 double wait_s{std::numeric_limits<double>::infinity()};
 // Evidence: swept minimum gap of the assessed execution (at wait_s when
 // known, at departure-now otherwise); +infinity when nothing was assessed.
 double min_gap_m{std::numeric_limits<double>::infinity()};
};

// One feasibility gate for every candidate kind: hard constraints first
// (full-trajectory swept gap, waiting-spot safety, horizon coverage), cost
// comparison only among feasible candidates (done by the caller, Phase-2).
// Unknown and Infeasible both refuse release; they differ in owner.
Feasibility check_candidate(LocalCandidate type, const std::vector<Vec2>& path, std::size_t start_idx,
                            double cruise_mps, const std::vector<std::vector<DynState>>& frames,
                            double frame_dt_s, double ego_radius_m, double margin_m);

} // namespace sim
