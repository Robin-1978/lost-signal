#pragma once

#include "sim/game/mission/world/actor_motion.hpp"

#include <cstdint>
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
namespace mission {

// Neutral behavior inputs: everything an evaluator needs, nothing it doesn't.
struct ActorBehaviorParams {
 std::vector<sim::Vec2> points;
 std::vector<double> waits;
 double speed_mps{0.5};
 double radius_m{0.25};
 // Seeded-wander phase (FNV-1a of the actor id, set by the caller):
 // wander ignores it, seeded-wander rotates/reverses the loop with it.
 std::uint64_t seed{0};
};

// Deterministic id hash (content addressing without content edits:
// distinct walkers desynchronize for free, same id replays bit-identical).
inline std::uint64_t hash_actor_id(const std::string& id) {
 std::uint64_t hash = 1469598103934665603ull;
 for (const char c : id) {
  hash ^= static_cast<unsigned char>(c);
  hash *= 1099511628211ull;
 }
 return hash;
}

using ActorBehaviorEvaluator = std::function<sim::DynamicObjectState(const ActorBehaviorParams&, double)>;

class ActorBehaviorRegistry {
 public:
  static void register_evaluator(const std::string& type, ActorBehaviorEvaluator evaluator);
  // Throws std::runtime_error on unknown type (loud, never fallback).
  static sim::DynamicObjectState evaluate(const std::string& type, const ActorBehaviorParams& params, double time_s);
  // All builtin type strings (for tests and content tooling).
  static std::vector<std::string> builtin_types();
  // Explicit builtin registration (one line per behavior below).
  static void register_builtin_behaviors();

 private:
  static std::map<std::string, ActorBehaviorEvaluator>& registry();
};

} // namespace mission
