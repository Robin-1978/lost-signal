#pragma once

namespace lost_signal::sim::viewer {

// Runs the interactive SIM viewer. The process entry point stays separate from
// the application composition so the executable boundary remains tiny.
int run_viewer(int argc, char** argv);

}  // namespace lost_signal::sim::viewer
