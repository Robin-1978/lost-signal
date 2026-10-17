#include "autonomy/mapping/observed_obstacle_layer.hpp"

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

 // Unobserved transient obstacles expire; an old frame cannot resurrect or
 // clear newer evidence.
 CHECK(layer.active_obstacle_cells(1.79) == 1);
 CHECK(layer.active_obstacle_cells(1.81) == 0);
 CHECK(!layer.observe(1.0, {{{2.25, 2.25}, {4.25, 2.25}, true}}));
 CHECK(layer.active_obstacle_cells(1.81) == 0);

 layer.reset();
 CHECK(layer.active_obstacle_cells(2.0) == 0);
 CHECK(!layer.fresh(2.0));
 return 0;
}
