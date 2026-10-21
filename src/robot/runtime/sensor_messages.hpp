#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sim {

inline constexpr std::uint32_t kMeasurementSchemaVersion = 4;

// Transport-neutral measurement header used by simulation, replay, native
// drivers, and future IPC providers. stamp_s is acquisition time in the
// producer's monotonic clock; arrival time must never replace it.
struct MeasurementHeader {
 double stamp_s{0.0};
 std::uint64_t sequence{0};
 std::string frame_id;
 double latency_s{0.0};
 std::uint32_t schema_version{kMeasurementSchemaVersion};
 std::string source_id;
};

struct Scan {
 std::vector<double> ranges;
 std::vector<char> hit;
 std::vector<double> intensities;
 int n_rays{0};
 double max_range{8.0};
 double angle_min_rad{0.0};
 double angle_increment_rad{0.0};
 double scan_period_s{0.0};
 double time_increment_s{0.0};
 // Optional per-ray acquisition offsets relative to the scan-end stamp.
 // A point-cloud projection may preserve the selected return's firing time.
 std::vector<double> ray_time_offsets_s;
 // Optional selected-return height in the sensor frame, in metres. NaN means
 // that this ray has no height-bearing return; planar scans leave it empty.
 std::vector<double> ray_heights_m;
 MeasurementHeader header;
};

enum class LidarPatternKind { PlanarSpinning, MultiBeamSpinning, SolidState };

struct LidarPointReturn {
 double x_m{0.0};
 double y_m{0.0};
 double z_m{0.0};
 double intensity{0.0};
 double time_offset_s{0.0};
 std::uint16_t ring{0};
 std::uint8_t return_index{0};
};

struct LidarPointCloud {
 std::vector<LidarPointReturn> points;
 LidarPatternKind pattern{LidarPatternKind::MultiBeamSpinning};
 std::uint16_t rings{0};
 std::uint32_t azimuth_samples{0};
 double max_range_m{0.0};
 double scan_period_s{0.0};
 MeasurementHeader header;
};

} // namespace sim
