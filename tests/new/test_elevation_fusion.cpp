#include "robot/autonomy/mapping/elevation_fusion.hpp"

#include <cmath>
#include <iostream>
#include <limits>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::autonomy::ElevationFusionConfig config;
 config.return_sigma_m = 0.02;
 config.pose_sigma_m = 0.03;
 config.maximum_sigma_m = 0.20;
 config.maximum_observations = 8;
 sim::autonomy::ElevationFusion fusion(config);
 fusion.reset(2);
 CHECK(fusion.cell_count() == 2);
 auto first = fusion.fuse(0, 1.0);
 CHECK(first.valid);
 CHECK(first.observations == 1);
 CHECK(std::abs(first.height_m - 1.0) < 1e-12);
 CHECK(first.sigma_m > 0.0);
 auto second = fusion.fuse(0, 1.1);
 CHECK(second.valid);
 CHECK(second.observations == 2);
 CHECK(second.height_m > 1.0 && second.height_m < 1.1);
 CHECK(second.sigma_m > first.sigma_m);
 auto other = fusion.fuse(1, 3.0, 0.01);
 CHECK(other.valid);
 CHECK(std::abs(other.height_m - 3.0) < 1e-12);
 CHECK(!fusion.fuse(9, 2.0).valid);
 CHECK(!fusion.fuse(0, std::numeric_limits<double>::quiet_NaN()).valid);

 // Large disagreement becomes uncertain rather than being silently accepted
 // as a precise terrain height.
 auto disagreement = fusion.fuse(0, 8.0);
 CHECK(!disagreement.valid);
 auto retained = fusion.estimate(0);
 CHECK(retained.observations == 3);
 CHECK(!retained.valid);
 return 0;
}
