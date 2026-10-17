#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace mission {

// One compact, presentation-neutral explanation of an autonomy decision.
// Stable codes are localized by clients and serialized verbatim by headless
// tools. Non-finite measurements mean that evidence was unavailable;
// algorithms never manufacture a measurement for presentation.
struct DecisionEvidence {
 std::uint64_t sequence{0};
 double decision_time_s{0.0};
 double end_time_s{-1.0};
 double observation_time_s{std::numeric_limits<double>::quiet_NaN()};
 std::vector<std::string> observation_refs;
 double prediction_horizon_s{0.0};
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

} // namespace mission
