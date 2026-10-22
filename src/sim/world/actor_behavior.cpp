#include "sim/world/actor_behavior.hpp"

#include <algorithm>
#include <cmath>

#include "sim/world/world.hpp"

namespace lost_signal::sim::world {

namespace {

// Deterministic stream (splitmix64): explicit per-actor state, no global
// RNG, no platform variance. Same seed + same call sequence = identical.
std::uint64_t splitmix64(std::uint64_t& state) {
  std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

double rand01(std::uint64_t& state) {
  return static_cast<double>(splitmix64(state) >> 11) *
         (1.0 / 9007199254740992.0);
}

ActorBehavior make_constant_velocity() { return ActorBehavior{}; }

ActorBehavior make_waypoints() {
  ActorBehavior b;
  b.kind = ActorBehaviorKind::kWaypoints;
  return b;
}

ActorBehavior make_wander() {
  ActorBehavior b;
  b.kind = ActorBehaviorKind::kWander;
  return b;
}

struct RegisterActorKind {
  RegisterActorKind(const std::string& name,
                    ActorBehaviorRegistry::Factory factory) {
    (void)ActorBehaviorRegistry::global().register_kind(name, factory);
  }
};

const RegisterActorKind kRegisteredConstant("constant-velocity",
                                             make_constant_velocity);
const RegisterActorKind kRegisteredWaypoints("waypoints", make_waypoints);
const RegisterActorKind kRegisteredWander("wander", make_wander);

}  // namespace

void step_actor(Actor& actor, ActorBehavior& behavior, double dt) {
  if (!(dt > 0.0)) return;
  switch (behavior.kind) {
    case ActorBehaviorKind::kConstantVelocity:
      return;  // authored velocity untouched
    case ActorBehaviorKind::kWaypoints: {
      if (behavior.waypoints.empty()) {
        actor.vx_mps = 0.0;  // misconfigured: stop, never invent motion
        actor.vy_mps = 0.0;
        return;
      }
      const double speed = std::max(0.0, behavior.speed_mps);
      const double arrive = std::max(1e-3, behavior.arrive_radius_m);
      const std::size_t n = behavior.waypoints.size();
      behavior.waypoint_index %= n;
      // Advance past reached points (capped: clustered points can't spin).
      for (std::size_t k = 0; k <= n; ++k) {
        const ActorWaypoint& wp = behavior.waypoints[behavior.waypoint_index];
        const double dx = wp.x_m - actor.x;
        const double dy = wp.y_m - actor.y;
        if (std::hypot(dx, dy) > arrive) break;
        if (!behavior.loop &&
            behavior.waypoint_index + 1 >= n) {
          actor.vx_mps = 0.0;  // final point reached: latched stop
          actor.vy_mps = 0.0;
          return;
        }
        behavior.waypoint_index =
            behavior.loop ? (behavior.waypoint_index + 1) % n
                          : behavior.waypoint_index + 1;
      }
      const ActorWaypoint& wp = behavior.waypoints[behavior.waypoint_index];
      const double dx = wp.x_m - actor.x;
      const double dy = wp.y_m - actor.y;
      const double dist = std::hypot(dx, dy);
      if (dist < 1e-9 || speed == 0.0) {
        actor.vx_mps = 0.0;
        actor.vy_mps = 0.0;
        return;
      }
      actor.vx_mps = speed * dx / dist;
      actor.vy_mps = speed * dy / dist;
      return;
    }
    case ActorBehaviorKind::kWander: {
      if (!behavior.initialized) {
        // Continue the authored heading when there is one; else face +x.
        if (std::hypot(actor.vx_mps, actor.vy_mps) > 1e-9) {
          behavior.heading_rad =
              std::atan2(actor.vy_mps, actor.vx_mps);
        } else {
          behavior.heading_rad = 0.0;
        }
        behavior.rng_state = behavior.seed ^ 0x9E3779B97F4A7C15ULL;
        behavior.initialized = true;
      }
      const double speed = std::max(0.0, behavior.speed_mps);
      const double turn = std::max(0.0, behavior.turn_rate_radps);
      behavior.heading_rad +=
          (rand01(behavior.rng_state) - 0.5) * 2.0 * turn * dt;
      actor.vx_mps = speed * std::cos(behavior.heading_rad);
      actor.vy_mps = speed * std::sin(behavior.heading_rad);
      return;
    }
  }
  actor.vx_mps = 0.0;  // unreachable kind: stop rather than drift
  actor.vy_mps = 0.0;
}

ActorBehaviorRegistry& ActorBehaviorRegistry::global() {
  static ActorBehaviorRegistry registry;
  return registry;
}

bool ActorBehaviorRegistry::register_kind(const std::string& name,
                                          Factory factory) {
  if (name.empty() || factory == nullptr || contains(name)) return false;
  kinds_.push_back({name, factory});
  return true;
}

bool ActorBehaviorRegistry::create(const std::string& name,
                                   ActorBehavior& out) const {
  for (const auto& e : kinds_) {
    if (e.name == name) {
      out = e.factory();
      return true;
    }
  }
  return false;
}

bool ActorBehaviorRegistry::contains(const std::string& name) const {
  for (const auto& e : kinds_) {
    if (e.name == name) return true;
  }
  return false;
}

}  // namespace lost_signal::sim::world
