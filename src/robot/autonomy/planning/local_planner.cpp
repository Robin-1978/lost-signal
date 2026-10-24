#include "robot/autonomy/planning/local_planner.hpp"

#include "robot/autonomy/planning/dwa_planner.hpp"
#include "robot/autonomy/planning/teb_planner.hpp"

namespace lost_signal::robot::autonomy {

std::unique_ptr<ILocalPlanner> make_local_planner(
 const std::string& planner_kind) {
 if (planner_kind == "dwa") return std::make_unique<DwaPlanner>();
 if (planner_kind == "teb") return std::make_unique<TebPlanner>();
 return nullptr;
}

} // namespace lost_signal::robot::autonomy
