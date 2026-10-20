#include "robot/autonomy/mapping/observed_obstacle_layer.hpp"

#include <iostream>
#include <vector>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::GridMap static_map(20, 20, 0.5);
 static_map.setOccupied(1.25, 1.25);
 sim::ObservedObstacleLayer layer(20, 20, 0.5, {0.6, 0.1});

 // The transient layer does not own the world boundary. Inflating an empty
 // observation layer must be an identity operation on the static base map;
 // otherwise a robot near a mapped boundary becomes trapped despite seeing
 // no foreground obstacle.
 const auto empty_composition = layer.compose(static_map, 0.0, 0.8);
 CHECK(empty_composition.isFree(9.75, 9.75));

 CHECK(layer.observe(1.0, {{{2.25, 2.25}, {5.25, 2.25}, true}}));
 CHECK(layer.fresh(1.19));
 CHECK(!layer.fresh(1.21));
 CHECK(layer.active_obstacle_cells(1.0) == 1);
 auto marked = layer.compose(static_map, 1.0, 0.0);
 CHECK(!marked.isFree(5.25, 2.25));
 CHECK(!marked.isFree(1.25, 1.25));

 // A later miss clears the complete observed ray. It cannot clear a cell
 // owned by the immutable static layer.
 CHECK(layer.observe(1.1, {{{2.25, 2.25}, {6.25, 2.25}, false},
 {{1.25, 1.25}, {1.25, 2.25}, false}}));
 auto cleared = layer.compose(static_map, 1.1, 0.0);
 CHECK(cleared.isFree(5.25, 2.25));
 CHECK(!cleared.isFree(1.25, 1.25));

 // Endpoint marking wins over a neighboring clearing ray in the same scan.
 CHECK(layer.observe(1.2, {{{2.25, 2.25}, {5.25, 2.25}, true},
 {{2.25, 2.25}, {6.25, 2.25}, false}}));
 CHECK(!layer.compose(static_map, 1.2, 0.0).isFree(5.25, 2.25));

 // After an anonymous return becomes a confirmed moving track, its recent
 // swept capsule is transferred out of static occupancy. A different current
 // return inside that capsule still wins, so ownership transfer cannot erase
 // independently observed geometry.
 CHECK(layer.observe(1.3, {
 {{2.25, 3.25}, {4.25, 3.25}, true},
 {{2.25, 3.25}, {5.25, 3.25}, true},
 }));
 CHECK(!layer.compose(static_map, 1.3, 0.0).isFree(4.25, 3.25));
 CHECK(!layer.compose(static_map, 1.3, 0.0).isFree(5.25, 3.25));
 CHECK(layer.observe(1.4,
 {{{2.25, 3.25}, {5.25, 3.25}, true}},
 {{{4.25, 3.25}, {5.25, 3.25}, 0.35}}));
 const auto transferred = layer.compose(static_map, 1.4, 0.0);
 CHECK(transferred.isFree(4.25, 3.25));
 CHECK(!transferred.isFree(5.25, 3.25));

 // Unobserved transient obstacles expire; an old frame cannot resurrect or
 // clear newer evidence.
CHECK(layer.active_obstacle_cells(1.99) == 1);
CHECK(layer.active_obstacle_cells(2.01) == 0);
 CHECK(!layer.observe(1.0, {{{2.25, 2.25}, {4.25, 2.25}, true}}));
CHECK(layer.active_obstacle_cells(2.01) == 0);

 layer.reset();
 CHECK(layer.active_obstacle_cells(2.0) == 0);
 CHECK(!layer.fresh(2.0));
 return 0;
}
