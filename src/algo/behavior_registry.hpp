#pragma once

#include "algo/dynobs.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

// Actor behavior registry (extension seam for motion patterns).
// A behavior is a pure function: (neutral params, time) -> state. Adding one
// = implement the evaluator + one register_builtin_behaviors() line + the
// type string in map JSON. No dispatch switch to extend, no mission edits.
// Unknown types throw loudly at construction (same contract as retired
// if/else chains, now data-driven).
//
// Layering: evaluators take neutral numbers only (ActorBehaviorParams),
// never game types — algo stays independent of game. The caller (mission)
// translates its ActorDefinition once; per-behavior math (ping-pong,
// loop, dwells) lives with its implementation.
//
// Explicit builtin registration (one call line per behavior): static
// self-registration was tried for chassis and reverted (archive linkers drop
// unreferenced objects). Explicit beats magic.
namespace sim {

// Neutral behavior inputs: everything an evaluator needs, nothing it doesn't.
struct ActorBehaviorParams {
 std::vector<Vec2> points;
 std::vector<double> waits;
 double speed_mps{0.5};
 double radius_m{0.25};
};

using BehaviorEvaluator = std::function<DynState(const ActorBehaviorParams&, double)>;

class BehaviorRegistry {
 public:
  static void register_evaluator(const std::string& type, BehaviorEvaluator evaluator);
  // Throws std::runtime_error on unknown type (loud, never fallback).
  static DynState evaluate(const std::string& type, const ActorBehaviorParams& params, double time_s);
  // All builtin type strings (for tests and content tooling).
  static std::vector<std::string> builtin_types();
  // Explicit builtin registration (one line per behavior below).
  static void register_builtin_behaviors();

 private:
  static std::map<std::string, BehaviorEvaluator>& registry();
};

} // namespace sim
