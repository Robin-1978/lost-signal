#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace lost_signal::robot::autonomy::diagnostics {

// Presentation-neutral decision evidence owned by ROBOT. SIM and UI only
// transport and render these records.
struct DecisionEvidence {
 std::uint64_t sequence{0};
 double decision_time_s{0.0};
 double end_time_s{-1.0};
 double observation_time_s{std::numeric_limits<double>::quiet_NaN()};
 std::vector<std::string> observation_refs;
 double prediction_horizon_s{0.0};
    std::size_t prediction_hypotheses{0};
 double measured_separation_m{std::numeric_limits<double>::quiet_NaN()};
 double predicted_separation_m{std::numeric_limits<double>::quiet_NaN()};
 std::string constraint_code{"none"};
 std::string reason_code{"unassessed"};
 std::string action_code{"idle"};
 double target_speed_mps{0.0};
 double target_yaw_rate_radps{0.0};
 std::string outcome_code{"active"};
 std::uint64_t superseded_by{0};
};

class DecisionEvidenceSystem {
 public:
 void reset();
 const DecisionEvidence& publish(DecisionEvidence decision, double now_s);
 const DecisionEvidence* close(std::string outcome_code, double now_s);
 const std::vector<DecisionEvidence>& records() const { return records_; }
 const DecisionEvidence* current() const {
 return records_.empty() ? nullptr : &records_.back();
 }
 private:
 std::vector<DecisionEvidence> records_;
 std::uint64_t next_sequence_{0};
};

} // namespace lost_signal::robot::autonomy::diagnostics
