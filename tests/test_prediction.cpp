#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/autonomy/planning/prediction.hpp"

int main() {
 using namespace lost_signal::robot::autonomy;

 DynamicTrack moving;
 moving.id = 1;
 moving.x_m = 0.0;
 moving.vx_mps = 1.0;
 const PredictionBatch cv = predict_hypotheses(
 {predict_input_from_track(moving, 0.0)}, 0.0, 1.0, 0.5, 1.0);
 assert(cv.hypotheses.size() == 1);
 assert(cv.hypotheses.front().model == PredictionModel::kConstantVelocity);
 assert(std::abs(cv.hypotheses.front().states.back().x_m - 1.0) < 1e-9);

 PredictInput patrol = predict_input_from_track(moving, 0.0);
 patrol.state.x_m = 0.9;
 patrol.state.vx_mps = 1.0;
 patrol.min_observed.x = 0.0;
 patrol.max_observed.x = 1.0;
 patrol.bounded_x = true;
 patrol.endpoint_dwell_s = 0.2;
 const PredictionBatch reflected = predict_hypotheses(
 {patrol}, 0.0, 0.5, 0.1, 1.0);
 assert(reflected.hypotheses.front().model ==
 PredictionModel::kReflectedCorridor);
 assert(reflected.hypotheses.front().states[1].x_m <= 1.0 + 1e-9);
 assert(reflected.hypotheses.front().states.back().vx_mps < 0.0);

 PredictInput multimodal = predict_input_from_track(moving, 0.0);
 multimodal.alternate_velocity_valid = true;
 multimodal.alternate_velocity = {-1.0, 0.0};
 const PredictionBatch alternatives = predict_hypotheses(
 {multimodal}, 0.0, 1.0, 0.5, 1.0);
 assert(alternatives.hypotheses.size() == 2);

 const PredictionBatch stale = predict_hypotheses(
 {predict_input_from_track(moving, 0.0)}, 2.0, 1.0, 0.5, 0.5);
 assert(stale.hypotheses.empty());

 DynamicTrack crossing;
 crossing.id = 2;
 crossing.x_m = 0.5;
 crossing.y_m = -1.0;
 crossing.vy_mps = 4.0;
 crossing.radius_m = 0.1;
 const PredictionBatch coarse = predict_hypotheses(
 {predict_input_from_track(crossing, 0.0)}, 0.0, 0.5, 0.5, 1.0);
 const PredictedClearance swept = minimum_predicted_clearance(
 coarse, {0.0, 0.0}, {1.0, 0.0}, 0.0, 0.5, 0.2);
 assert(swept.clearance_m < 0.0);
 assert(swept.time_s > 0.0 && swept.time_s < 0.5);

 std::printf("prediction ok\n");
 return 0;
}
