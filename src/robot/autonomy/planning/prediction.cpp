#include "robot/autonomy/planning/prediction.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lost_signal::robot::autonomy {

namespace {

struct ReflectedAxisState {
  double position{0.0};
  double velocity{0.0};
};

ReflectedAxisState reflected_axis(double position, double velocity,
                                  double time_s, double minimum,
                                  double maximum, bool bounded,
                                  double endpoint_dwell_s,
                                  double initial_dwell_remaining_s) {
  if (!bounded || maximum - minimum < 0.05 || std::abs(velocity) < 1e-9)
    return {position + velocity * time_s, velocity};
  double remaining = std::max(0.0, time_s);
  position = std::clamp(position, minimum, maximum);
  if (initial_dwell_remaining_s > 0.0) {
    if (remaining <= initial_dwell_remaining_s) return {position, 0.0};
    remaining -= initial_dwell_remaining_s;
    if ((position >= maximum - 1e-6 && velocity > 0.0) ||
        (position <= minimum + 1e-6 && velocity < 0.0))
      velocity = -velocity;
  }
  for (int transition = 0; transition < 16 && remaining > 1e-12;
       ++transition) {
    const double boundary = velocity > 0.0 ? maximum : minimum;
    const double travel_s = std::abs(boundary - position) / std::abs(velocity);
    if (remaining < travel_s) return {position + velocity * remaining, velocity};
    position = boundary;
    remaining -= travel_s;
    if (remaining <= endpoint_dwell_s) return {position, 0.0};
    remaining -= endpoint_dwell_s;
    velocity = -velocity;
  }
  return {std::clamp(position + velocity * remaining, minimum, maximum),
          velocity};
}

}  // namespace

PredictionBatch predict_hypotheses(const std::vector<PredictInput>& inputs,
                                   double time_s, double horizon_s,
                                   double frame_dt_s, double max_age_s) {
  PredictionBatch batch;
  batch.stamp_s = time_s;
  batch.frame_dt_s = frame_dt_s;
  if (!(frame_dt_s > 0.0) || horizon_s < 0.0 || max_age_s < 0.0) return batch;
  for (double future_s = 0.0; future_s <= horizon_s + 1e-9;
       future_s += frame_dt_s) {
    batch.future_times_s.push_back(future_s);
  }
  const auto predict_mode = [](const PredictInput& input,
                               core::Vec2 velocity, double prediction_time) {
    DynamicTrack predicted = input.state;
    const auto x = reflected_axis(
        predicted.x_m, velocity.x, prediction_time, input.min_observed.x,
        input.max_observed.x, input.bounded_x, input.endpoint_dwell_s,
        input.initial_dwell_remaining_s);
    const auto y = reflected_axis(
        predicted.y_m, velocity.y, prediction_time, input.min_observed.y,
        input.max_observed.y, input.bounded_y, input.endpoint_dwell_s,
        input.initial_dwell_remaining_s);
    predicted.x_m = x.position;
    predicted.y_m = y.position;
    predicted.vx_mps = x.velocity;
    predicted.vy_mps = y.velocity;
    predicted.position_uncertainty_m = std::min(
        std::max(0.0, input.position_uncertainty_cap_m),
        std::max(0.0, input.state.position_uncertainty_m) +
            std::max(0.0, input.position_uncertainty_growth_mps) *
                std::max(0.0, prediction_time));
    return predicted;
  };
  for (const auto& input : inputs) {
    const double age = time_s - input.last_observed_s;
    if (age < -1e-9 || age > max_age_s) continue;
    struct VelocityMode {
      core::Vec2 velocity;
      bool alternate{false};
    };
    std::vector<VelocityMode> modes;
    const auto append_mode = [&](core::Vec2 velocity, bool alternate) {
      const auto duplicate = std::any_of(
          modes.begin(), modes.end(), [&](const VelocityMode& existing) {
            return std::hypot(existing.velocity.x - velocity.x,
                              existing.velocity.y - velocity.y) <= 1e-9;
          });
      if (!duplicate) modes.push_back({velocity, alternate});
    };
    const auto append_speed_envelope = [&](core::Vec2 velocity, bool alternate) {
      const double speed = std::hypot(velocity.x, velocity.y);
      const double uncertainty = std::max(0.0, input.velocity_uncertainty_mps);
      if (speed > 1e-9 && uncertainty > 0.0) {
        const core::Vec2 direction{velocity.x / speed, velocity.y / speed};
        const double slow = std::max(0.0, speed - uncertainty);
        append_mode({direction.x * slow, direction.y * slow}, alternate);
      }
      append_mode(velocity, alternate);
      if (speed > 1e-9 && uncertainty > 0.0) {
        const core::Vec2 direction{velocity.x / speed, velocity.y / speed};
        const double fast = speed + uncertainty;
        append_mode({direction.x * fast, direction.y * fast}, alternate);
      }
    };
    append_speed_envelope({input.state.vx_mps, input.state.vy_mps}, false);
    if (input.alternate_velocity_valid)
      append_speed_envelope(input.alternate_velocity, true);
    for (std::size_t mode_index = 0; mode_index < modes.size(); ++mode_index) {
      const auto& mode = modes[mode_index];
      PredictedHypothesis hypothesis;
      hypothesis.track_id = input.track_id;
      hypothesis.hypothesis_id = mode_index;
      const double speed = std::hypot(mode.velocity.x, mode.velocity.y);
      hypothesis.model = mode.alternate
                             ? PredictionModel::kAlternateIntent
                             : speed <= 1e-9
                                   ? PredictionModel::kStationary
                                   : (input.bounded_x || input.bounded_y)
                                         ? PredictionModel::kReflectedCorridor
                                         : PredictionModel::kConstantVelocity;
      hypothesis.states.reserve(batch.future_times_s.size());
      for (const double future_s : batch.future_times_s) {
        hypothesis.states.push_back(
            predict_mode(input, mode.velocity, age + future_s));
      }
      batch.hypotheses.push_back(std::move(hypothesis));
    }
  }
  return batch;
}

std::vector<std::vector<DynamicTrack>> flatten_prediction_frames(
    const PredictionBatch& prediction) {
  std::vector<std::vector<DynamicTrack>> frames(
      prediction.future_times_s.size());
  for (const auto& hypothesis : prediction.hypotheses) {
    const std::size_t count =
        std::min(frames.size(), hypothesis.states.size());
    for (std::size_t frame = 0; frame < count; ++frame)
      frames[frame].push_back(hypothesis.states[frame]);
  }
  return frames;
}

std::vector<std::vector<DynamicTrack>> predict_frames(
    const std::vector<PredictInput>& inputs, double time_s, double horizon_s,
    double frame_dt_s, double max_age_s) {
  return flatten_prediction_frames(
      predict_hypotheses(inputs, time_s, horizon_s, frame_dt_s, max_age_s));
}

}  // namespace lost_signal::robot::autonomy
