#include "sim/world/actor_behavior.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
      // Serving a timed stop: velocity zero, latch counts down. Expiry
      // falls through to the advance logic (the dwelled point is already
      // reached, so it moves past it — or latches the final stop), but
      // must not re-latch on the same step (eternal dwell): the skip
      // below covers exactly the countdown-expiry step.
      bool skip_latch = false;
      if (behavior.dwelling_) {
        behavior.dwell_remaining_s -= dt;
        actor.vx_mps = 0.0;
        actor.vy_mps = 0.0;
        if (behavior.dwell_remaining_s > 0.0) return;
        behavior.dwelling_ = false;
        behavior.dwell_remaining_s = 0.0;
        skip_latch = true;
      }
      const double speed = std::max(0.0, behavior.speed_mps);
      const double arrive = std::max(1e-3, behavior.arrive_radius_m);
      const std::size_t n = behavior.waypoints.size();
      behavior.waypoint_index %= n;
      // Advance past reached points (capped: clustered points can't spin).
      // The final point of a non-loop latches stopped here (stop is
      // infinite dwell: no timed stop served at the terminus).
      std::size_t arrivals = 0;
      std::size_t last_reached = n;  // sentinel: none yet
      for (std::size_t k = 0; k <= n; ++k) {
        const ActorWaypoint& wp = behavior.waypoints[behavior.waypoint_index];
        const double dx = wp.x_m - actor.x;
        const double dy = wp.y_m - actor.y;
        if (std::hypot(dx, dy) > arrive) break;
        ++arrivals;
        last_reached = behavior.waypoint_index;
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
      // Fresh arrival + configured dwell: latch the timed stop now (the
      // post-countdown fall-through above moves past afterwards). Already
      // dwelling is impossible here (early return above), so sitting
      // inside the radius never re-triggers — no eternal dwell. Duration
      // is the reached point's wait (per-point override, else uniform).
      if (arrivals > 0 && !skip_latch && last_reached < n) {
        double dur = std::max(0.0, behavior.dwell_s);
        if (!std::isfinite(dur)) dur = 0.0;
        if (last_reached < behavior.waits_s.size()) {
          const double w = behavior.waits_s[last_reached];
          dur = (std::isfinite(w) ? std::max(0.0, w) : 0.0);
        }
        if (dur > 0.0) {
          behavior.dwelling_ = true;
          behavior.dwell_remaining_s = dur;
          actor.vx_mps = 0.0;
          actor.vy_mps = 0.0;
          return;
        }
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

void apply_actor_robot_yield(Actor& actor, ActorBehavior& behavior,
 double robot_x_m, double robot_y_m,
 double robot_vx_mps, double robot_vy_mps,
 double robot_radius_m, double dt) {
 if (!(dt > 0.0) || !behavior.yield_to_robot) return;
 const double desired_speed = std::hypot(actor.vx_mps, actor.vy_mps);
 if (!behavior.yield_initialized_) {
 behavior.yield_speed_mps_ = desired_speed;
 behavior.yield_initialized_ = true;
 }

 // Solve |relative_position + relative_velocity * t| = combined_radius.
 // The first non-negative root is the continuous time to envelope entry;
 // comparing it with reaction + braking time prevents discrete-step
 // tunnelling without peeking into any ROBOT estimate or plan.
 const double px = robot_x_m - actor.x;
 const double py = robot_y_m - actor.y;
 const double rvx = robot_vx_mps - actor.vx_mps;
 const double rvy = robot_vy_mps - actor.vy_mps;
 const double combined_radius = std::max(0.0, actor.radius_m) +
 std::max(0.0, robot_radius_m) +
 std::max(0.0, behavior.clearance_m);
 const double a = rvx * rvx + rvy * rvy;
 const double b = 2.0 * (px * rvx + py * rvy);
 const double c = px * px + py * py - combined_radius * combined_radius;
 bool conflict = c <= 0.0;
 double first_contact_s = conflict
 ? 0.0 : std::numeric_limits<double>::infinity();
 if (!conflict && a > 1e-12) {
 const double discriminant = b * b - 4.0 * a * c;
 if (discriminant >= 0.0) {
 const double root = std::sqrt(discriminant);
 const double enter = (-b - root) / (2.0 * a);
 const double leave = (-b + root) / (2.0 * a);
 if (leave >= 0.0) first_contact_s = std::max(0.0, enter);
 }
 }
 const double brake = std::max(1e-6, behavior.brake_deceleration_mps2);
 const double stopping_horizon_s =
 std::max(0.0, behavior.reaction_time_s) +
 behavior.yield_speed_mps_ / brake;
 conflict = conflict || first_contact_s <= stopping_horizon_s;

 if (conflict) {
 behavior.yield_speed_mps_ = std::max(
 0.0, behavior.yield_speed_mps_ - brake * dt);
 } else {
 behavior.yield_speed_mps_ = std::min(
 desired_speed,
 behavior.yield_speed_mps_ +
 std::max(0.0, behavior.acceleration_mps2) * dt);
 }
 if (desired_speed <= 1e-12 || behavior.yield_speed_mps_ <= 0.0) {
 actor.vx_mps = 0.0;
 actor.vy_mps = 0.0;
 return;
 }
 const double scale = std::min(1.0, behavior.yield_speed_mps_ / desired_speed);
 actor.vx_mps *= scale;
 actor.vy_mps *= scale;
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
