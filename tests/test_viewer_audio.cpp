#include <cassert>
#include <filesystem>

#include "sim/viewer/viewer_audio.hpp"

int main() {
 using namespace lost_signal::sim::viewer;

 // A disabled device still advances the same evidence state. This is the
 // contract used by mute/headless tests and prevents playback policy from
 // changing physical interpretation.
 ViewerAudio audio(std::filesystem::path{}, false);
 Frame impact;
 impact.step = 10;
 impact.body.drive_free_speed_mps = 2.0;
 impact.body.nominal_normal_load_N = 200.0;
 impact.impact.valid = true;
 impact.impact.normal_impulse_Ns = 4.0;
 impact.impact.relative_speed_mps = 1.0;
 audio.update(impact, false, false, true); // Turbo-suppressed
 assert(audio.physical_sample().step == 10);
 assert(audio.physical_sample().impact_event);

 // Returning to ordinary playback on the same completed frame must not
 // replay the impact hidden during Turbo/pause.
 audio.update(impact, false, false, false);
 assert(!audio.physical_sample().advanced);
 assert(!audio.physical_sample().impact_event);

 Frame moving;
 moving.step = 11;
 moving.body.drive_free_speed_mps = 2.0;
 moving.body.nominal_normal_load_N = 200.0;
 moving.actuation.drive_count = 1;
 moving.actuation.drive_duty[0] = 0.7;
 FrameWheel wheel;
 wheel.driven = true;
 wheel.radius_m = 0.1;
 wheel.spin_rate_radps = 12.0;
 moving.body.wheels.push_back(wheel);
 audio.update(moving, false, false, true); // paused/muted policy
 assert(audio.physical_sample().advanced);
 assert(audio.physical_sample().motor_intensity > 0.0);
 audio.update(moving, false, false, false);
 assert(!audio.physical_sample().advanced);

 // A backwards scrub is a discontinuity, not a new physical impact.
 impact.step = 3;
 audio.update(impact, false, false, false);
 assert(audio.physical_sample().rewound);
 assert(!audio.physical_sample().impact_event);

 audio.reset();
 assert(audio.physical_sample().step == 0);
 assert(!audio.physical_sample().advanced);
 return 0;
}
