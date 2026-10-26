#pragma once

#include <cstddef>
#include <deque>
#include <vector>

namespace lost_signal::sim::viewer {

// Converts wall time into a number of fixed simulation steps.  It never
// changes simulation dt: rate and turbo only control how many 20 ms steps the
// caller executes between renders.
class PlaybackController {
 public:
  struct Config {
    double step_s = 0.02;
    double requested_rate = 1.0;
 // Maximum retained backlog expressed in wall seconds. The corresponding
 // simulation-time debt scales with the requested rate, so a 64x request
 // is not silently capped to the same 0.5 simulated seconds as 1x.
 double max_backlog_wall_s = 0.5;
    double frame_work_budget_ms = 12.0;
 // Turbo may spend more time between input polls while remaining bounded.
 // Physics dt, sensor cadence, and ROBOT cadence are unchanged.
 double turbo_frame_work_budget_ms = 24.0;
    int max_steps_per_frame = 256;
    double statistics_window_s = 2.0;
  };

  struct Statistics {
    double requested_rate = 1.0;
    double actual_rtf = 0.0;
    double steps_per_s = 0.0;
    double tick_p99_ms = 0.0;
 double tick_max_ms = 0.0;
    bool turbo = false;
    bool throttled = false;
  };

  PlaybackController();
  explicit PlaybackController(Config config);

  void set_requested_rate(double rate);
  double requested_rate() const { return config_.requested_rate; }
  void set_turbo(bool enabled);
  bool turbo() const { return turbo_; }

  // Call once per rendered frame, then repeatedly call step_due()/on_step(),
  // and finally end_frame(). elapsed_work_ms is time spent stepping in the
  // current frame. Paused frames should call discard_debt() instead.
  void begin_frame(double wall_dt_s);
  bool step_due(double elapsed_work_ms);
  void on_step(double tick_ms);
  void end_frame();
  void discard_debt();

  Statistics statistics() const;
  int steps_this_frame() const { return frame_steps_; }

 private:
  struct FrameSample {
    double end_wall_s = 0.0;
    double wall_s = 0.0;
    int steps = 0;
    bool throttled = false;
  };
  struct TickSample {
    double wall_s = 0.0;
    double tick_ms = 0.0;
  };

  void trim_window();

  Config config_;
  bool turbo_ = false;
  double debt_s_ = 0.0;
  double wall_s_ = 0.0;
 double frame_wall_s_ = 0.0;
  int frame_steps_ = 0;
  bool frame_throttled_ = false;
  std::deque<FrameSample> frames_;
  std::deque<TickSample> ticks_;
};

} // namespace lost_signal::sim::viewer
