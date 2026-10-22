#pragma once

namespace lost_signal::robot::autonomy {

// Deployable stop latch shared by the simulated and native plants.
//
// A rejected trajectory publishes a staged braking command, but the command
// reaching zero is not evidence that the wheels have settled. Handing authority
// back on the first nominally feasible cycle turns a short safety rejection on
// a grade into a brake/accelerate limit cycle. The latch therefore stays closed
// until the measured tread speed has stayed below the release threshold for a
// fixed number of decision cycles.
//
// The latch is deliberately independent of the planner's requested yaw: a
// request is evidence of intent, never of motion, and gating release on it
// would deadlock the in-place turn the planner needs in order to leave the stop.
class ControlledStopLatch {
public:
 struct Config {
 // Peak measured tread speed at or below which the plant counts as settled.
 double release_speed_mps{0.04};
 // Consecutive settled decision cycles required before releasing authority.
 int settle_cycles{3};
 };

 // Advance one decision cycle and report whether motion must stay withheld.
 //
 // `carrying_stop` is whether the previously published command was itself a
 // controlled stop; `peak_wheel_speed_mps` is the larger of the two measured
 // tread speeds. A negative or non-finite speed is treated as unsettled so a
 // feedback dropout cannot release the latch.
 bool update(bool carrying_stop, double peak_wheel_speed_mps, const Config& config);

 // Forget accumulated settle evidence (session reset, plant reconfiguration).
 void reset() { settle_cycles_ = 0; }

 int settle_cycles() const { return settle_cycles_; }

private:
 int settle_cycles_{0};
};

} // namespace lost_signal::robot::autonomy
