#pragma once

namespace sim {

// Honest slip detection: compares what the wheels claim (encoder-derived
// longitudinal acceleration) against what the body feels (IMU acceleration).
// Both inputs are sensed quantities; model-side saturation truth never enters,
// so a detection is something the robot could actually know onboard.
//
// Why filtering is needed: encoders quantize (CPR=2048, r=0.1 m, 20 ms step:
// ~0.015 m/s per quantum, i.e. ~0.77 m/s^2 per-step spikes in the raw
// derivative). The residual therefore passes a first-order low-pass with
// tau = 0.12 s, which cuts white per-step noise by sqrt(dt/2*tau) ~= 1/3.5
// while a real slip episode (sustained 0.2-1 s wheel/body divergence, several
// m/s^2) passes nearly untouched.
//
// INPUT CONTRACT (learned the hard way): both accelerations must be
// post-step quantities from independent states — tread speed for the encoder
// path, body speed for the IMU path. Feeding a body-derived encoder signal
// (pre-step twist, as DifferentialMotorDrive telemetry was) makes the
// residual measure intra-step integration jerk, not slip: it then latches on
// every slam-reversal while model saturation stays silent. Wiring this
// detector to the game therefore waits on the quarter-car wheel/body split
// that gives encoders a tread state of their own.
// 6-sigma over the filtered floor keeps clean driving silent; the 0.6 m/s^2
// floor stops a near-perfect IMU from tripping on quantization residue.
// Hysteresis (off at half threshold) plus persistence (4 steps to latch,
// 8 steps to release) rejects single-step spikes. All of tau/floor/counts
// are method constants; only sigma comes from content.
// Threshold scales with the equipped IMU spec: on = max(0.6, 6 * sigma).
class SlipDetector {
 public:
 explicit SlipDetector(double accel_sigma = 0.15)
     : on_threshold_(0.6 > 6.0 * accel_sigma ? 0.6 : 6.0 * accel_sigma),
       off_threshold_((0.6 > 6.0 * accel_sigma ? 0.6 : 6.0 * accel_sigma) * 0.5) {}
 void reset() {
  filtered_ = 0.0;
  above_count_ = 0;
  below_count_ = 0;
  detected_ = false;
  episodes_ = 0;
 }
 // Returns the current latched detection state. Rising edges increment
 // the episode counter (use episodes() for debrief recall).
 bool step(double encoder_accel, double imu_accel, double dt) {
  const double raw = encoder_accel >= imu_accel ? encoder_accel - imu_accel : imu_accel - encoder_accel;
  const double alpha = dt / (kTauS + dt);
  filtered_ += alpha * (raw - filtered_);
  if (!detected_) {
   above_count_ = filtered_ >= on_threshold_ ? above_count_ + 1 : 0;
   if (above_count_ >= kConfirmSteps) {
    detected_ = true;
    above_count_ = 0;
    ++episodes_;
   }
  } else {
   below_count_ = filtered_ <= off_threshold_ ? below_count_ + 1 : 0;
   if (below_count_ >= kReleaseSteps) {
    detected_ = false;
    below_count_ = 0;
   }
  }
  return detected_;
 }
 bool detected() const { return detected_; }
 int episodes() const { return episodes_; }

 private:
 static constexpr double kTauS = 0.12;
 static constexpr int kConfirmSteps = 4;
 static constexpr int kReleaseSteps = 8;
 double on_threshold_{0.9};
 double off_threshold_{0.45};
 double filtered_{0.0};
 int above_count_{0};
 int below_count_{0};
 bool detected_{false};
 int episodes_{0};
};

} // namespace sim
