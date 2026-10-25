#pragma once

// SIM-owned actor motion policies (World internals, never ROBOT input).
// Actors used to fly straight lines; behaviors decide their velocity per
// step: constant (authored), waypoint follow, or seeded wander. Determinism
// is structural: policies are pure functions of (actor, behavior, dt) with
// an explicit splitmix64 stream per actor — same seed + same dt sequence
// gives bit-identical trajectories, so replay re-runs them exactly.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lost_signal::sim::world {

struct Actor;  // defined in world.hpp; behavior mutates velocity only.

struct ActorWaypoint {
  double x_m = 0.0;
  double y_m = 0.0;
};

enum class ActorBehaviorKind {
  kConstantVelocity,  // authored vx/vy untouched (today's straight lines)
  kWaypoints,         // head to each point at speed_mps, loop or stop
  kWander,            // seeded random-walk heading at speed_mps
};

struct ActorBehavior {
  ActorBehaviorKind kind = ActorBehaviorKind::kConstantVelocity;
  // Waypoints: empty list fails stopped (velocity zero, never invented).
  std::vector<ActorWaypoint> waypoints;
  double speed_mps = 1.0;  // clamped >= 0
  bool loop = true;        // false: stop at the final point
  double arrive_radius_m = 0.3;
  // Dwell: pause this long at each waypoint arrival (master's timed-stop
  // doctrine, adapted to the incremental stepper: velocity zero while the
  // latch counts down). 0 = today's behavior exactly (no pause).
  double dwell_s = 0.0;
  // Per-point waits (master's waits[] doctrine): waits_s[i] overrides
  // dwell_s at waypoints[i]; shorter lists fall back to dwell_s per
  // index, longer tails are ignored. Empty = uniform dwell_s (legacy).
  // Negative/non-finite entries read 0 (never time travel).
  std::vector<double> waits_s;
  // Wander: explicit stream, seeded on first step.
  std::uint64_t seed = 1;
  double turn_rate_radps = 0.6;  // max heading rate, clamped >= 0
  // Runtime, mutated by stepping (per-actor stream state included).
  std::size_t waypoint_index = 0;
  double heading_rad = 0.0;
  bool initialized = false;
  std::uint64_t rng_state = 0;
  // Dwell latch: true while serving a timed stop.
  bool dwelling_ = false;
 double dwell_remaining_s = 0.0;
 // Optional SIM actor-agent response. Scripted motion remains the desired
 // path, while continuous relative-motion prediction brakes the actor before
 // it reaches the physical robot envelope. These are actor capabilities,
 // never ROBOT inputs; false preserves an unaware/hostile obstacle.
 bool yield_to_robot = false;
 double reaction_time_s = 0.25;
 double acceleration_mps2 = 1.0;
 double brake_deceleration_mps2 = 1.5;
 double clearance_m = 0.20;
 bool yield_initialized_ = false;
 double yield_speed_mps_ = 0.0;
};

// One deterministic step: sets the actor velocity from its behavior.
// dt <= 0 is a no-op (no time passes, no stream consumed).
void step_actor(Actor& actor, ActorBehavior& behavior, double dt);

// Apply the actor's declared collision-response model to the nominal
// velocity produced by step_actor(). The robot pose/velocity are SIM physics
// state used only to evolve the other simulated agent; no result crosses the
// ROBOT observation seam except through ordinary sensors.
void apply_actor_robot_yield(Actor& actor, ActorBehavior& behavior,
 double robot_x_m, double robot_y_m,
 double robot_vx_mps, double robot_vy_mps,
 double robot_radius_m, double dt);

// String registry for scenario authors (S5 files name behaviors; the World
// fills parameters afterwards). Builtins self-register below; first
// registration wins, duplicates rejected, unknown fails closed.
class ActorBehaviorRegistry {
 public:
  static ActorBehaviorRegistry& global();
  using Factory = ActorBehavior (*)();
  bool register_kind(const std::string& name, Factory factory);
  bool create(const std::string& name, ActorBehavior& out) const;
  bool contains(const std::string& name) const;

 private:
  struct Entry {
    std::string name;
    Factory factory = nullptr;
  };
  std::vector<Entry> kinds_;
};

}  // namespace lost_signal::sim::world
