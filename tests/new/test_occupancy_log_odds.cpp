#include "robot/autonomy/mapping/occupancy_log_odds.hpp"

#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::autonomy::OccupancyLogOdds fusion;
 fusion.reset(2);
 CHECK(fusion.value(0) == -1);
 // A single ray passage publishes free space for an unseen cell.
 fusion.observe_free(0);
 CHECK(fusion.value(0) == 0);
 // A single confirmed endpoint outweighs one earlier passage.
 fusion.observe_occupied(0);
 CHECK(fusion.value(0) == 100);
 // A single contrary ray can never erase a confirmed obstacle...
 fusion.observe_free(0);
 CHECK(fusion.value(0) == 100);
 // ...but sustained free evidence clears a transient return.
 fusion.observe_free(0, 8);
 CHECK(fusion.value(0) == 0);
 CHECK(fusion.observed_cells() == 1);
 CHECK(fusion.value(9) == -1);
 CHECK(fusion.cell_count() == 2);
 return 0;
}
