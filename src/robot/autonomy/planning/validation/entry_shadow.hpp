#pragma once

#include <limits>

namespace sim {

// Entry-window shadow (Phase-1, observation only, never wired to behavior).
// Answers the flagship-death question: over the WHOLE ego transit window,
// is the conflict zone free of the threat — not just at the current instant.
// The myopic check (current distance) says "go" for both fast and slow ego
// on the same geometry; the window check separates them: slow ego transits
// longer, so the same threat phase that a fast ego outruns catches the slow
// one mid-zone. That separation is the unit under test.
//
// Layering: pure function on neutral numbers (distances, speeds, stamps).
// No map/game types, no chassis stepping, no filter fusion. The caller
// (future Phase-2, or the test) translates tracks once; phasing math lives
// here. Mission/DWA consume nothing — this file has no consumer yet.
//
// Vocabulary discipline: verdicts are OBSERVED window relations
// (Clear/Occupied/Uncertain), never Feasible/Allowed. Uncertain is sticky:
// invalid or stale threat input yields Uncertain, NEVER a default-go Clear.
// Margins are reported evidence, not gates — nothing releases on them,
// This remains an observational shadow model. Any authority path requires
// separate safety, confidence, and parity evidence.

enum class EntryVerdict {
  kObservedClear,    // threat window and ego window strictly disjoint.
  kObservedOccupied, // windows overlap or touch: touching is sharing.
  kObservedUncertain // no assessment possible (invalid/stale threat input).
};

// One threat projected at constant speed onto the conflict zone.
// dist_to_zone_m: threat distance to the zone NEAR edge along its path.
// Negative means already inside/past (handled explicitly, not clamped).
struct WindowThreat {
  bool valid{false};          // false = no threat data (sensor gap, not "free").
  double dist_to_zone_m{0.0}; // to near edge; <0 means inside or past.
  double speed_mps{0.0};      // toward the zone; <=0 means not approaching.
  double age_s{0.0};          // observation age; over budget -> Uncertain.
};

struct EntryWindowAssessment {
  double ego_enter_s{0.0};    // ego reaches zone near edge.
  double ego_exit_s{0.0};     // ego clears zone far edge.
  double threat_enter_s{0.0}; // threat reaches near edge (inf = never).
  double threat_exit_s{0.0};  // threat clears far edge (inf = never/stays).
  double margin_s{0.0};       // min separation between windows (>0 = disjoint).
  EntryVerdict verdict{EntryVerdict::kObservedUncertain};
  bool ego_invalid{false};    // ego inputs degenerate (no speed/distance).
};

constexpr double kEntryStaleBudgetS = 0.30; // same dropout budget as estimator replay.

// Constant-speed projection of both windows, exact interval comparison.
// Ego: [dist/v, (dist + 2*half)/v]. Threat approaching: [dist/u,
// (dist + 2*half)/u]; threat inside (dist<0, moving): entered at 0, exits at
// (dist + 2*half)/u; threat not approaching (u<=0) and outside: never
// (inf/inf -> Clear iff valid, threat cannot arrive); threat stationary
// inside: [0, inf] -> Occupied whenever ego transits. Invalid ego (v<=0 or
// dist<0) or invalid/stale threat -> Uncertain.
EntryWindowAssessment assess_entry_window(double ego_dist_m, double ego_speed_mps,
                                          double conflict_half_len_m, const WindowThreat& threat,
                                          double stale_budget_s = kEntryStaleBudgetS);

const char* entry_verdict_name(EntryVerdict verdict);

} // namespace sim
