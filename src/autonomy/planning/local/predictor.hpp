#pragma once

#include "autonomy/perception/dynamic_object.hpp"
#include "autonomy/mapping/grid_map.hpp"

#include <limits>
#include <vector>

namespace sim {

// Predictor input: one prediction seed plus what the tracker learned about
// its motion corridor. The seed may preserve a learned periodic-motion
// velocity while the current public track is stopped at a corridor endpoint.
// The predictor owns only the future: constant-velocity extrapolation with
// reflection inside the observed corridor. Pure function of its arguments,
// no hidden state, so DWA and tests can call it without a tracker.
struct PredictInput {
 DynamicObjectState state;
 // Optional second motion mode. Trackers use it when a previously moving
 // target pauses: continuing and reversing are both plausible until fresh
 // observations resolve intent.
 Vec2 alternate_velocity{0.0, 0.0};
 bool alternate_velocity_valid{false};
 double last_observed_s{0.0};
 Vec2 min_observed{0.0, 0.0};
 Vec2 max_observed{0.0, 0.0};
 bool bounded_x{false};
 bool bounded_y{false};
};

// Fixed-step future frames: frame k holds the predicted state at
// time_s + age + k * frame_dt_s for every input younger than max_age_s.
// Stale inputs (age > max_age_s) and future-dated inputs are skipped.
std::vector<std::vector<DynamicObjectState>> predict_frames(const std::vector<PredictInput>& inputs,
                                                            double time_s, double horizon_s,
                                                            double frame_dt_s, double max_age_s);

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
                 const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                 double ego_radius_m);

double predicted_gap(const std::vector<Vec2>& path, std::size_t start_idx, double speed_mps,
                     const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                     double ego_radius_m, double depart_delay_s = 0.0);

// Earliest safe departure: smallest wait d >= 0 such that departing at now+d
// keeps the whole prediction horizon clear (via predicted_gap with delay).
// Returns +infinity when no departure within the horizon works. Phase-1a
// (observe only): the mission records it during waits without acting on it;
// action waits until the observed error (predicted vs actual release) proves
// small. Pure function, no behavior by itself.
double earliest_departure(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                          const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                          double ego_radius_m, double margin_m);

// Infinity is not one verdict: callers (and shadow logs) must distinguish
// why no departure is provable, because each cause belongs to a different
// owner — window sizing, Phase-2 waiting spots, or a genuinely blocked
// corridor. kHorizonShort/kWaitUnsafe/kBlockedInWindow are undecidable here,
// never "unsafe forever".
enum class DepartReason {
 kGoNow,           // wait_s == 0: depart now, whole trajectory proven.
 kWait,            // wait_s > 0: earliest provably safe departure.
 kBlockedInWindow, // horizon covers the traversal and the waiting spot stays
                   // safe, yet no wait clears the corridor.
 kWaitUnsafe,      // the waiting spot itself is swept by a threat within the
                   // horizon: staying is what kills. Phase-2 territory.
 kHorizonShort,    // the horizon cannot even cover the traversal: unknown,
                   // not unsafe (SOTIF Area 3: never file unknown as safe).
 kInvalid,         // degenerate input (empty path/frames, non-positive cruise).
 kUnassessed,      // capability does not exist yet (Phase-2 pocket/rejoin/detour):
                   // honestly Unknown, never borrowed from another cause.
 kClear,           // a leg checked over its full interval with no violation.
 kStaticBlocked,   // no static path exists (A* on the inflated grid failed):
                   // geometric infeasibility, proven — not traffic, not unknown.
 kCapsViolated,    // the geometric path exists but the chassis cannot execute
                   // it: fillet radius below Rmin, smoothed path clipping an
                   // obstacle, or steer demand beyond spec. Proven against the
                   // caller-supplied execution caps — not traffic, not unknown.
};

struct DepartureAssessment {
 double wait_s{std::numeric_limits<double>::infinity()};
 DepartReason reason{DepartReason::kInvalid};
};

const char* depart_reason_name(DepartReason reason);

DepartureAssessment assess_departure(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                                     const std::vector<std::vector<DynamicObjectState>>& frames,
                                     double frame_dt_s, double ego_radius_m, double margin_m);

// Phase-2 local-behavior candidates (plan item 16b). The kinds are frozen;
// only time feasibility is assessable today. Pocket/rejoin/detour need
// spatial planning that does not exist yet and report Unknown honesty.
enum class LocalCandidate {
 kProceed,      // depart now along the remaining path, no waiting.
 kWaitInPlace,  // hold the current spot until the assessed departure, then proceed.
 kWaitAtPocket, // move aside, wait safe, rejoin. Phase-2 (unassessed).
 kRejoin,       // rejoin evaluation after a pocket wait. Phase-2 (unassessed).
 kDetour,       // Existing local detour; DWA owns execution today.
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
                            double cruise_mps, const std::vector<std::vector<DynamicObjectState>>& frames,
                            double frame_dt_s, double ego_radius_m, double margin_m);

// One assessed phase of a WaitAtPocket candidate. end_s chains the schedule:
// the next leg starts at this leg's nominal (not proven) completion — legs
// never restart at t = 0. A joint instant belongs to both adjacent legs
// (closed intervals; min over a shared endpoint is idempotent, so no gap and
// no double jeopardy in the verdict).
struct LegAssessment {
 Feasibility::Verdict verdict{Feasibility::Verdict::kUnknown};
 DepartReason reason{DepartReason::kUnassessed};
 double end_s{std::numeric_limits<double>::infinity()};
 double min_gap_m{std::numeric_limits<double>::infinity()};
 // The timed trajectory that was actually swept (transit only; empty when
 // the leg was decided without executing anything).
 std::vector<TimedPose> exec_traj;
};

// Leg 1 — Transit assessment with its own planning (map-mediated overload
// below); the plain-path stub is retired — every transit verdict now names
// the executed trajectory or exactly why it cannot.

// Execution capability for transit assessment. Spin branch uses can_spin +
// yaw_rate only; Ackermann branch uses wheel_base/max_steer/max_steer_rate/
// servo_response (RO-2 JSON values flow here from the mission layer — never
// defaulted inside the assessment, so an unspecified model stays Unknown
// instead of silently borrowing another chassis' numbers).
struct ChassisExecCaps {
 bool can_spin{true};
 double yaw_rate_rads{2.0};
 double wheel_base_m{0.0};
 double max_steer_rad{0.0};
 double max_steer_rate_radps{0.0};
 double servo_response_s{0.0};
};

// Transit with its own planning: A* on the caller-inflated grid produces the
// geometric path (static clearance is the generator's contract); the verdict
// then covers the EXECUTED trajectory, never the grid polyline by proxy:
// spin-capable chassis execute turn-drive-turn (rotate in place at yaw_rate,
// drive legs at cruise — the body stays inside its collision ring while
// spinning), which is swept exactly via timed_gap. Ackermann chassis execute
// arc-fillet smoothing (radius >= L/tan(max_steer)) followed through a
// servo-lag bicycle sim at cruise; the SIMULATED trajectory (not the fillet
// reference) is static-rechecked and swept, so smoothing deviations and lag
// are evidence, not assumptions. Unspecified models stay Unknown/kUnassessed.
// A* failure is proven static infeasibility (kStaticBlocked), not traffic,
// not unknown.
// Evidence: exec_traj holds the checked timed trajectory (empty when nothing
// was executed-checked).
LegAssessment assess_transit(const GridMap& inflated_map, Vec2 start, double start_theta, Vec2 pocket,
                             double cruise_mps, const ChassisExecCaps& exec,
                             const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                             double ego_radius_m, double margin_m);

// Leg 2 — Hold at a spot over [hold_from_s, hold_until_s]: the whole interval
// is swept, never just the endpoints. Zero-duration (from == until) reduces
// to a point check. hold_until past the horizon is Unknown (unprovable), a
// swept spot is Infeasible/WaitUnsafe (staying is what kills).
LegAssessment assess_hold(Vec2 spot, double hold_from_s, double hold_until_s,
                          const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                          double ego_radius_m, double margin_m);

// Leg 3 — Rejoin from the pocket after depart_delay_s: hold-then-go over the
// given path via predicted_gap. Coverage gated first (traversal must finish
// inside the horizon), then the violation is classified: swept hold position
// means WaitUnsafe, otherwise BlockedInWindow.
LegAssessment assess_rejoin(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                            double depart_delay_s, const std::vector<std::vector<DynamicObjectState>>& frames,
                            double frame_dt_s, double ego_radius_m, double margin_m);

struct PocketAssessment {
 LegAssessment transit;
 LegAssessment hold;
 LegAssessment rejoin;
 Feasibility::Verdict overall{Feasibility::Verdict::kUnknown};
 // Chain minimum over legs that produced evidence (nominal model only: same
 // inputs, same arithmetic — not a release license, and never inherited by
 // perturbed scenarios). +infinity when no leg produced evidence.
 double chain_min_gap_m{std::numeric_limits<double>::infinity()};
};

// Pocket candidate sampling (generation layer): lateral offsets off route
// vertices, snapped to free inflated cells, each paired with a downstream
// merge vertex and the route suffix from it. Sampling NEVER upgrades a
// safety verdict: every candidate still goes through the full three-leg
// assessment, and a missed feasible spot outside the sampled set proves
// nothing about the space. All budgets are explicit caller inputs (search
// effort, not physics — nothing to derive, everything to state).
struct PocketSampleParams {
 // Lateral offsets to try per route vertex, in order (meters, signed).
 std::vector<double> lateral_offsets_m{1.0, -1.0};
 // Only vertices within this arc distance from route start are used (m).
 double window_m{10.0};
 // Minimum arc spacing between sampled vertices (m, 0 = every vertex):
 // observation-budget diversification, not a safety parameter.
 double vertex_stride_m{0.0};
 // Merge = first vertex at least this far downstream of the pocket vertex (m).
 double min_downstream_m{3.0};
 // Pockets closer than this to the start are skipped (m): that is WaitInPlace.
 double min_pocket_dist_m{0.3};
 // Hard cap on assessed candidates (deterministic order: route, then offset).
 int max_candidates{8};
};

struct PocketCandidate {
 Vec2 pocket{0.0, 0.0};
 Vec2 merge{0.0, 0.0};
 PocketAssessment assessment;
};

enum class SearchVerdict {
 kFoundFeasible, // at least one candidate fully proven (evidence attached).
 kAllRefused,    // every assessed candidate proven Infeasible (reasons kept).
                 // Says nothing about unsampled space.
 kIncomplete,    // no proof either way: Unknown legs, truncation, or zero
                 // candidates sampled. Never file as safe or as infeasible.
};

struct PocketSearch {
 std::vector<PocketCandidate> candidates;
 SearchVerdict verdict{SearchVerdict::kIncomplete};
 bool truncated{false};
};

// Generate (bounded, deterministic) + assess (three legs each) + keep every
// verdict with its reasons. hold_s is the scheduling policy input, identical
// for all candidates; scheduling optimization is not this slice.
PocketSearch sample_pockets(const GridMap& inflated_map, const std::vector<Vec2>& route, Vec2 start,
                            double start_theta, double cruise_mps, const ChassisExecCaps& exec, double hold_s,
                            const PocketSampleParams& params,
                            const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                            double ego_radius_m, double margin_m);

// Pocket candidate with both transits planned: A* from start to pocket and
// from pocket to the route merge point (on the caller-inflated grid), then
// the route suffix appended for the rejoin leg. Spot selection lives in
// sample_pockets; this composes assessment for given spots.
PocketAssessment assess_pocket_wait(const GridMap& inflated_map, Vec2 start, double start_theta, Vec2 pocket,
                                    Vec2 merge, const std::vector<Vec2>& route_suffix, double cruise_mps,
                                    const ChassisExecCaps& exec, double hold_s,
                                    const std::vector<std::vector<DynamicObjectState>>& frames,
                                    double frame_dt_s, double ego_radius_m, double margin_m);

} // namespace sim
