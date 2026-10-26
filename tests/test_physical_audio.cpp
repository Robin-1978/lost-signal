#include <cassert>
#include <cmath>
#include <stdexcept>

#include "sim/viewer/physical_audio.hpp"

namespace {

lost_signal::sim::viewer::Frame rolling_frame(std::uint64_t step) {
 using namespace lost_signal::sim::viewer;
 Frame frame;
 frame.step = step;
 frame.body.drive_free_speed_mps = 2.0;
 frame.body.nominal_normal_load_N = 200.0;
 frame.actuation.drive_count = 1;
 frame.actuation.drive_duty[0] = 0.5;
 FrameWheel wheel;
 wheel.driven = true;
 wheel.radius_m = 0.1;
 wheel.spin_rate_radps = 10.0;
 wheel.contact_valid = true;
 wheel.contact_normal_force_N = 100.0;
 wheel.contact_speed_mps = 1.0;
 wheel.contact_motion_x_mps = 1.0;
 wheel.slip_speed_mps = 0.02;
 wheel.contact_slip_x_mps = 0.02;
 frame.body.wheels.push_back(wheel);
 return frame;
}

lost_signal::sim::viewer::PhysicalAudioConfig immediate_config() {
 lost_signal::sim::viewer::PhysicalAudioConfig config;
 config.attack_time_s = 0.0;
 config.release_time_s = 0.0;
 config.impact_decay_time_s = 0.0;
 return config;
}

} // namespace

int main() {
 using namespace lost_signal::sim::viewer;

 bool rejected = false;
 try {
 PhysicalAudioConfig bad;
 bad.sim_step_s = 0.0;
 PhysicalAudioEvidence invalid(bad);
 (void)invalid;
 } catch (const std::invalid_argument&) {
 rejected = true;
 }
 assert(rejected);

 // Motor, rolling, and load are normalized by this chassis' configured free
 // speed and static weight. Removing contact removes rolling/slip evidence,
 // while shaft motion remains audible as motor/gear motion.
 PhysicalAudioEvidence rolling(immediate_config());
 const PhysicalAudioSample loaded = rolling.advance(rolling_frame(1));
 assert(loaded.advanced && !loaded.rewound);
 assert(std::abs(loaded.wheel_speed_ratio - 0.5) < 1e-12);
 assert(std::abs(loaded.contact_load_ratio - 0.5) < 1e-12);
 assert(loaded.motor_intensity > 0.0);
 assert(loaded.rolling_intensity > 0.0);
 assert(loaded.slip_intensity > 0.0);

 Frame airborne = rolling_frame(2);
 airborne.body.wheels[0].contact_valid = false;
 const PhysicalAudioSample no_contact = rolling.advance(airborne);
 assert(no_contact.motor_intensity > 0.0);
 assert(no_contact.rolling_intensity == 0.0);
 assert(no_contact.slip_intensity == 0.0);

 // A single slipping wheel contributes by its measured normal load. A second
 // loaded, non-slipping wheel lowers the load-weighted slip intensity.
 Frame single_slip = rolling_frame(10);
 single_slip.body.wheels[0].contact_slip_x_mps = 0.8;
 single_slip.body.wheels[0].slip_speed_mps = 0.8;
 PhysicalAudioEvidence one_wheel(immediate_config());
 const double one_wheel_slip = one_wheel.advance(single_slip).slip_intensity;
 Frame split_load = single_slip;
 split_load.step = 11;
 FrameWheel quiet = split_load.body.wheels[0];
 quiet.contact_slip_x_mps = 0.0;
 quiet.slip_speed_mps = 0.0;
 split_load.body.wheels.push_back(quiet);
 PhysicalAudioEvidence two_wheels(immediate_config());
 const auto balanced = two_wheels.advance(split_load);
 assert(balanced.contact_load_ratio == 1.0);
 assert(balanced.slip_intensity < one_wheel_slip);

 // Service-brake sound requires both the recorded actuator request and
 // physical wheel/body motion; a stationary holding brake stays quiet.
 Frame braking = rolling_frame(20);
 braking.actuation.service_brake = 1.0;
 PhysicalAudioEvidence brake_state(immediate_config());
 assert(brake_state.advance(braking).brake_intensity > 0.0);
 Frame held = rolling_frame(21);
 held.actuation.service_brake = 1.0;
 held.body.wheels[0].spin_rate_radps = 0.0;
 held.body.wheels[0].contact_speed_mps = 0.0;
 held.body.wheels[0].contact_motion_x_mps = 0.0;
 assert(brake_state.advance(held).brake_intensity == 0.0);

 // Impact gain uses the measured impulse and relative speed. A continuous
 // contact produces one onset, a repeated/paused frame produces none, and a
 // later distinct impact can trigger again.
 Frame impact = rolling_frame(30);
 impact.impact.valid = true;
 impact.impact.normal_impulse_Ns =
 impact.body.nominal_normal_load_N * immediate_config().sim_step_s;
 impact.impact.relative_speed_mps = 1.0;
 PhysicalAudioEvidence impacts(immediate_config());
 PhysicalAudioSample onset = impacts.advance(impact);
 assert(onset.impact_event && onset.impact_intensity > 0.0);
 PhysicalAudioSample paused = impacts.advance(impact);
 assert(!paused.advanced && !paused.impact_event);
 impact.step = 31;
 assert(!impacts.advance(impact).impact_event);
 impact.step = 32;
 impact.impact.valid = false;
 impacts.advance(impact);
 impact.step = 33;
 impact.impact.valid = true;
 assert(impacts.advance(impact).impact_event);

 // Scrubbing backwards synchronizes continuous state but never presents the
 // selected historical impact as a new event. Subsequent forward processing
 // also remembers that the selected contact was already active.
 Frame rewind = impact;
 rewind.step = 5;
 const PhysicalAudioSample rewound = impacts.advance(rewind);
 assert(rewound.rewound && !rewound.advanced && !rewound.impact_event);
 rewind.step = 6;
 assert(!impacts.advance(rewind).impact_event);

 return 0;
}
