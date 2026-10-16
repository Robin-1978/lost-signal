#pragma once
#include <vector>
#include <cstdint>
// Headless event log: the evidence layer for failure diagnosis.
// Runners and the client share it; UI panels only read it. Equal codes are rate-limited.
namespace sim {
enum EvCode : uint8_t {
 EV_COLLISION = 0, // Static collision, including blind motion.
 EV_DYN_HIT,       // Dynamic-actor collision.
 EV_DYN_NEAR,      // Clearance to a dynamic actor fell below 0.8 m.
 EV_DRIFT_HIGH,    // Odometry drift exceeded 1 m.
 EV_DWA_STOP,      // No feasible DWA command; stop in place.
 EV_VIA,           // Passed a waypoint.
 EV_REACHED,       // Reached the goal.
 EV_NO_PATH,       // A* found no path.
 EV_LOST,          // Route ended without arrival, for example after drift.
 EV_COUNT
};
inline const char* ev_name(uint8_t c) {
 static const char* n[EV_COUNT] = {"COLLISION", "DYN_HIT", "DYN_NEAR", "DRIFT_HIGH", "DWA_STOP",
                                   "VIA",       "REACHED", "NO_PATH",  "LOST"};
 return c < EV_COUNT ? n[c] : "?";
}
struct Event {
 double t{0};
 uint8_t code{0};
 float aux{0};
};
class EventLog {
 public:
 void clear() {
  ev_.clear();
  for (auto& x : last_)
   x = -1e9;
 }
 // Record a code at most once per min_gap seconds. Return whether it was kept.
 bool log(double t, uint8_t code, float aux = 0, double min_gap = 2.0) {
  if (code >= EV_COUNT)
   return false;
  if (t - last_[code] < min_gap)
   return false;
  last_[code] = t;
  ev_.push_back({t, code, aux});
  return true;
 }
 const std::vector<Event>& events() const { return ev_; }
 uint32_t mask() const {
  uint32_t m = 0;
  for (auto& e : ev_)
   m |= (1u << e.code);
  return m;
 }

 private:
 std::vector<Event> ev_;
 double last_[EV_COUNT];
};
} // namespace sim
