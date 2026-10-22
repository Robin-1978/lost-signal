#pragma once

#include <string>

#include "robot/platform/drive_profile.hpp"

namespace lost_signal::sim::eval {

// Drive-profile file loading (presentation side). Merges a flat JSON object
// over the factory defaults for its "kind": every key except "kind" is
// optional, so a file may override just the measured numbers (e.g.
// free_speed_mps from a step response) while geometry stays canonical.
// Unknown kind, malformed JSON, or a non-finite value fails closed with a
// reason; the out-profile is untouched on failure.
bool load_drive_profile(const std::string& path,
                        robot::platform::DriveProfile& profile,
                        std::string* error);

}  // namespace lost_signal::sim::eval
