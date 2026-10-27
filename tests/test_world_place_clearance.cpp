#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "robot/platform/description.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"

namespace {

using lost_signal::sim::world::NamedPlace;
using lost_signal::sim::world::StaticBox;

// Distance from a point to the nearest point of an axis-aligned box; zero when
// the point is inside it.
double distance_to_box_m(double x, double y, const StaticBox& box) {
  const double dx = std::max({box.x0 - x, 0.0, x - box.x1});
  const double dy = std::max({box.y0 - y, 0.0, y - box.y1});
  return std::hypot(dx, dy);
}

struct ClearanceViolation {
  std::string place_id;
  std::string box_id;
  double distance_m = 0.0;
};

// R2: a place is a goal the robot has to arrive at, so every solid box must
// stay at least `minimum_clearance_m` away from it. Flat paint (height_m <= 0)
// has no volume and cannot block arrival, so it is ignored.
std::vector<ClearanceViolation> place_clearance_violations(
    const std::vector<NamedPlace>& places, const std::vector<StaticBox>& boxes,
    double minimum_clearance_m) {
  std::vector<ClearanceViolation> violations;
  for (const auto& place : places) {
    for (std::size_t i = 0; i < boxes.size(); ++i) {
      const StaticBox& box = boxes[i];
      if (!(box.height_m > 0.0)) continue;
      const double distance_m = distance_to_box_m(place.x_m, place.y_m, box);
      if (distance_m < minimum_clearance_m) {
        violations.push_back(
            {place.place_id,
             box.box_id.empty() ? "#" + std::to_string(i) : box.box_id,
             distance_m});
      }
    }
  }
  return violations;
}

} // namespace

int main() {
  using namespace lost_signal;

  // The arrival radius is a parameter, not a literal. It is the larger of the
  // SIM acceptance tolerance and the ROBOT arrival-policy tolerance, so a place
  // that passes here can be completed by both sides. Per-robot configs under
  // configs/robot set 0.20 to 0.25, which is below this default.
  const double arrival_radius_m = std::max(
      sim::eval::ObjectiveAcceptanceConfig{}.position_tolerance_m,
      robot::platform::ArrivalPolicy{}.position_tolerance_m);
  assert(arrival_radius_m > 0.0);

  // The checker on synthetic data: it must flag what it claims to flag.
  {
    StaticBox solid;
    solid.box_id = "solid";
    solid.x0 = 1.0;
    solid.y0 = 1.0;
    solid.x1 = 2.0;
    solid.y1 = 2.0;
    solid.height_m = 1.0;
    StaticBox paint = solid;
    paint.box_id = "paint";
    paint.height_m = 0.0;
    const double radius_m = 0.30;
    const auto flagged = [&](double x, double y, const StaticBox& box) {
      return place_clearance_violations({{"probe", x, y, 0.0}}, {box}, radius_m)
          .size();
    };
    assert(flagged(1.5, 1.5, solid) == 1);   // inside the box
    assert(flagged(2.29, 1.5, solid) == 1);  // 0.29 m from the east face
    assert(flagged(2.31, 1.5, solid) == 0);  // 0.31 m from the east face
    assert(flagged(2.25, 2.25, solid) == 0); // corner: 0.354 m, not 0.25 m
    assert(flagged(1.5, 1.5, paint) == 0);   // flat paint has no volume
    const auto detail =
        place_clearance_violations({{"probe", 2.1, 1.5, 0.0}}, {solid}, radius_m);
    assert(detail.size() == 1);
    assert(detail[0].place_id == "probe" && detail[0].box_id == "solid");
    assert(std::abs(detail[0].distance_m - 0.1) < 1e-9);
  }

  // Every checked-in world. A new world file is picked up without editing
  // this test; scene and actor sidecars share the directory and are skipped.
  const std::filesystem::path worlds =
      std::filesystem::path(TEST_CONFIG_DIR) / "worlds";
  std::size_t checked_worlds = 0;
  std::size_t checked_places = 0;
  double closest_m = 1e9;
  for (const auto& entry : std::filesystem::directory_iterator(worlds)) {
    const std::string filename = entry.path().filename().string();
    if (entry.path().extension() != ".json") continue;
    if (filename.ends_with(".scene.json") || filename.ends_with(".actors.json")) {
      continue;
    }
    sim::eval::WorldFileContent content;
    std::string error;
    const bool loaded =
        sim::eval::load_world_file(entry.path().string(), content, &error);
    if (!loaded) std::fprintf(stderr, "%s: %s\n", filename.c_str(), error.c_str());
    assert(loaded);

    const auto violations =
        place_clearance_violations(content.places, content.boxes, arrival_radius_m);
    for (const auto& violation : violations) {
      std::fprintf(stderr,
                   "%s: place '%s' is %.3f m from solid box '%s' (arrival radius %.2f m)\n",
                   filename.c_str(), violation.place_id.c_str(),
                   violation.distance_m, violation.box_id.c_str(),
                   arrival_radius_m);
    }
    assert(violations.empty());

    for (const auto& place : content.places) {
      for (const auto& box : content.boxes) {
        if (box.height_m > 0.0) {
          closest_m = std::min(closest_m,
                               distance_to_box_m(place.x_m, place.y_m, box));
        }
      }
    }
    ++checked_worlds;
    checked_places += content.places.size();
  }
  // Lower bounds keep an empty or mis-filtered directory from passing
  // silently. At authoring time: 11 worlds and 77 places.
  assert(checked_worlds >= 8);
  assert(checked_places >= 60);

  std::printf(
      "world_place_clearance ok: worlds=%zu places=%zu arrival_radius=%.2f "
      "closest=%.3f m\n",
      checked_worlds, checked_places, arrival_radius_m, closest_m);
  return 0;
}
