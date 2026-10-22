#include <cstdio>

#include "sim/game/scenario_session.hpp"

int main() {
  lost_signal::sim::game::ScenarioSession session;
  lost_signal::robot::iface::TaskRequest req;
  req.request_id = "demo";
  req.place_id = "dock";
  session.submit_task(req);
  for (int i = 0; i < 5; ++i) session.step_once();
  std::printf("sim ran 5 steps\n");
  return 0;
}
