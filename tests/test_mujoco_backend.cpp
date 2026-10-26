// Engine behavior gate: the plant must reproduce deployable behavior.
// Pins are behavioral (moves, stops, holds grade, deterministic); exact
// transients belong to the effort-model unit tests.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

#include "robot/iface/task.hpp"
#include "robot/platform/holonomic_kinematics.hpp"
#include "sim/providers/sim_motion.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

sim::world::World make_mujoco_diff() { return sim::world::World(); }

void set_diff_duty(sim::world::World& w, double duty) {
  robot::iface::ActuatorCommand cmd;
  cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
  cmd.stamp_ns = 0;
  cmd.expiry_ns = 1'000'000'000;
  cmd.drive[0] = cmd.drive[1] = duty;
  w.apply(cmd);
}

}  // namespace

int main() {
#ifdef LOST_SIGNAL_WITH_MUJOCO
 // Every public entry point resolves a duration into the same 1 ms plant
 // quanta. A 20 ms harness call must therefore match the Viewer/Session
 // form that executes twenty ticks inside one presentation transition.
 {
 sim::world::World chunked;
 sim::world::World split;
 auto configure = [](sim::world::World& world) {
 sim::world::MotorParams motor = world.motor_params();
 motor.brake_max_torque_Nm = 2.0;
 world.set_motor_params(motor);
 world.set_materials({{"wet", 0.35, 0.006, 0.008, 1.0, 0.2}});
 world.set_material_zones({{"wet_lane", -5.0, -2.0, 8.0, 2.0, "wet"}});
 world.set_environment({{0.0, {{"wet", 1.0}}},
 {2.0, {{"wet", 0.6}}}});
 sim::world::Actor actor;
 actor.x = 2.0;
 actor.y = -1.0;
 actor.radius_m = 0.35;
 assert(sim::world::ActorBehaviorRegistry::global().create(
 "wander", actor.behavior));
 actor.behavior.seed = 97;
 actor.behavior.speed_mps = 0.7;
 actor.behavior.turn_rate_radps = 0.4;
 world.add_actor(actor);
 set_diff_duty(world, 0.65);
 world.set_brake_request(0, 0.4);
 world.set_brake_request(1, 0.4);
 };
 configure(chunked);
 configure(split);
 for (int transition = 0; transition < 100; ++transition) {
 chunked.step(0.02);
 split.begin_fixed_transition();
 for (int quantum = 0; quantum < 20; ++quantum) split.step(0.001);
 split.end_fixed_transition();
 const auto& a = chunked.truth();
 const auto& b = split.truth();
 assert(chunked.sim_time_ns() == split.sim_time_ns());
 assert(a.step == b.step);
 assert(a.body_x_m == b.body_x_m);
 assert(a.body_y_m == b.body_y_m);
 assert(a.body_z_m == b.body_z_m);
 assert(a.body_yaw_rad == b.body_yaw_rad);
 assert(a.forward_mps == b.forward_mps);
 assert(a.lateral_mps == b.lateral_mps);
 assert(a.yaw_rate_radps == b.yaw_rate_radps);
 assert(a.drive_angle_rad == b.drive_angle_rad);
 assert(a.drive_radps == b.drive_radps);
 assert(a.drive_slipping == b.drive_slipping);
 assert(chunked.actors().front().x == split.actors().front().x);
 assert(chunked.actors().front().y == split.actors().front().y);
 assert(chunked.actors().front().vx_mps == split.actors().front().vx_mps);
 assert(chunked.actors().front().vy_mps == split.actors().front().vy_mps);
 assert(chunked.impact().valid == split.impact().valid);
 assert(chunked.impact().normal_impulse_Ns ==
 split.impact().normal_impulse_Ns);
 }

 bool rejected_off_grid = false;
 try {
 chunked.step(0.0015);
 } catch (const std::invalid_argument&) {
 rejected_off_grid = true;
 }
 assert(rejected_off_grid);
 }

 // Preparing the plant exposes the authored zero-time assembly without
 // consuming a physics step. A briefing may inspect this SIM state, but it
 // must not advance actors, publish observations, or create score history.
 {
 sim::world::World w = make_mujoco_diff();
 set_diff_duty(w, 0.0);
 w.prepare();
 assert(w.sim_time_s() == 0.0);
 assert(w.truth().step == 0);
 assert(std::abs(w.truth().body_z_m - w.motor_params().wheel_radius_m) <
 1e-12);
 sim::world::WheelViz wheels[6];
 assert(w.wheels(wheels, 6) == 2);
 assert(wheels[0].pose.valid);
 assert(wheels[1].pose.valid);
 assert(wheels[0].drive_joint.valid);
 assert(std::abs(wheels[0].drive_joint.axis_x) < 1e-9);
 assert(std::abs(wheels[0].drive_joint.axis_y - 1.0) < 1e-9);
 assert(std::abs(wheels[0].drive_joint.axis_z) < 1e-9);
 sim::world::JointAxisViz axes[20];
 const int axis_count = w.joint_axes(axes, 20);
 assert(axis_count >= 2);
 assert(std::isfinite(w.system_com_relative_x_m()));
 assert(std::isfinite(w.system_com_relative_y_m()));
 assert(std::isfinite(w.system_com_relative_z_m()));

 sim::scenario::ScenarioSession session;
 session.prepare_world();
 assert(session.setup_world().sim_time_s() == 0.0);
 assert(session.setup_world().truth().step == 0);
 assert(session.episode().steps().empty());
 assert(session.score().steps == 0);
 }

 // Dynamic actors are mocap bodies in the same MuJoCo scene. Their visual
 // pose and contact geometry are available to SIM presentation without
 // entering the ROBOT observation path.
 {
 sim::world::World w = make_mujoco_diff();
 sim::world::Actor actor;
 actor.x = 0.8;
 actor.y = -0.4;
 actor.radius_m = 0.25;
 actor.height_m = 1.8;
 actor.visual_kind = "pedestrian";
 w.add_actor(actor);
 set_diff_duty(w, 0.0);
 w.prepare();
 sim::world::ActorPoseViz poses[4];
 assert(w.actor_poses(poses, 4) == 1);
 assert(poses[0].valid);
 assert(std::abs(poses[0].x_m - 0.8) < 1e-9);
 assert(std::abs(poses[0].y_m + 0.4) < 1e-9);
 assert(std::abs(poses[0].z_m) < 1e-9);
 }

 // The compiled multibody has the declared mass and COM. Collision-shell
 // dimensions never move when inertia is tuned.
 {
 sim::world::World bare = make_mujoco_diff();
 set_diff_duty(bare, 0.0);
 bare.step(0.0);
 assert(std::abs(bare.compiled_mass_kg() - 24.0) < 1e-9);
 assert(std::abs(bare.system_com_z_m() - 0.1) < 1e-9);

 sim::world::MotorParams low_com = bare.motor_params();
 low_com.com_height_m = 0.07;
 bare.set_motor_params(low_com);
 bare.step(0.0);
 assert(std::abs(bare.compiled_mass_kg() - 24.0) < 1e-9);
 assert(std::abs(bare.system_com_z_m() - 0.07) < 1e-9);

 sim::world::MotorParams loaded = low_com;
 loaded.com_height_m = 0.1;
 loaded.payload_mass_kg = 5.0;
 loaded.payload_height_m = 0.3;
 bare.set_motor_params(loaded);
 bare.step(0.0);
 assert(std::abs(bare.compiled_mass_kg() - 29.0) < 1e-9);
 assert(std::abs(bare.system_com_z_m() - 3.9 / 29.0) < 1e-9);
 }
 // Collision presentation is driven by an integrated MuJoCo contact
 // impulse against authored static geometry. Ordinary wheel/ground load
 // is excluded, and the evidence remains entirely on the SIM side.
 {
 sim::world::World w = make_mujoco_diff();
 w.set_static_boxes({{0.70, -1.0, 0.80, 1.0, 0.6, "impact_wall"}});
 w.prepare();
 assert(!w.impact().valid);
 set_diff_duty(w, 1.0);
 bool saw_impact = false;
 for (int i = 0; i < 200 && !saw_impact; ++i) {
 w.step(0.02);
 const sim::world::ImpactViz& impact = w.impact();
 if (!impact.valid) continue;
 assert(impact.normal_impulse_Ns > 0.0);
 assert(impact.relative_speed_mps > 0.0);
 assert(std::isfinite(impact.x_m));
 assert(std::isfinite(impact.y_m));
 assert(std::isfinite(impact.z_m));
 assert(!impact.spark_eligible); // metal chassis against concrete
 saw_impact = true;
 }
 assert(saw_impact);

 sim::world::World metal;
 metal.set_static_boxes({{0.70, -1.0, 0.80, 1.0, 0.6,
 "metal_fixture", sim::world::ImpactMaterial::kMetal}});
 metal.prepare();
 set_diff_duty(metal, 1.0);
 bool saw_metal_spark = false;
 for (int i = 0; i < 200 && !saw_metal_spark; ++i) {
 metal.step(0.02);
 const sim::world::ImpactViz& impact = metal.impact();
 if (!impact.spark_eligible) continue;
 assert(impact.spark_normal_impulse_Ns > 0.0);
 assert(impact.spark_relative_speed_mps > 0.0);
 assert(impact.spark_robot_material == sim::world::ImpactMaterial::kMetal);
 assert(impact.spark_static_material == sim::world::ImpactMaterial::kMetal);
 saw_metal_spark = true;
 }
 assert(saw_metal_spark);
 }
  // Spin-up: moves, settles below free speed, level attitude.
  {
    sim::world::World w = make_mujoco_diff();
    set_diff_duty(w, 1.0);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(std::isfinite(w.truth().forward_mps));
    assert(w.truth().forward_mps > 1.0);  // pulls hard
    assert(w.truth().forward_mps < 3.5);  // drag-limited, never above free
    assert(std::abs(w.truth().body_roll_rad) < 0.05);
    assert(std::abs(w.truth().body_pitch_rad) < 0.05);
    assert(w.truth().body_z_m > 0.0);  // off the ground plane, on wheels
    // Rendering reads the exact MuJoCo link transforms. Differential wheel
    // centers stay on the base axle plane and roll about the physical lateral
    // Y hinge; there is no terrain lift or reconstructed vertical-axis spin.
    sim::world::WheelViz wheels[6];
    const int wheel_count = w.wheels(wheels, 6);
    assert(wheel_count == 2);
    for (int i = 0; i < wheel_count; ++i) {
    assert(wheels[i].pose.valid);
 assert(wheels[i].contact_valid);
 assert(std::isfinite(wheels[i].contact_slip_speed_mps));
 assert(wheels[i].contact_slip_speed_mps >= 0.0);
 assert(std::isfinite(wheels[i].contact_longitudinal_slip_mps));
 assert(std::isfinite(wheels[i].contact_lateral_slip_mps));
 assert(std::isfinite(wheels[i].contact_motion_x_mps));
 assert(std::isfinite(wheels[i].contact_motion_y_mps));
 assert(std::isfinite(wheels[i].contact_motion_z_mps));
 assert(std::isfinite(wheels[i].contact_slip_x_mps));
 assert(std::isfinite(wheels[i].contact_slip_y_mps));
 assert(std::isfinite(wheels[i].contact_slip_z_mps));
 assert(std::isfinite(wheels[i].contact_friction_utilization));
 assert(wheels[i].contact_friction_utilization >= 0.0);
 const double resolved_slip = std::hypot(
 wheels[i].contact_longitudinal_slip_mps,
 wheels[i].contact_lateral_slip_mps);
 // Multiple contact points are averaged independently, so the norm of the
 // resolved components cannot exceed the averaged tangential-speed norm.
 assert(resolved_slip <= wheels[i].contact_slip_speed_mps + 1e-9);
 const double world_slip = std::sqrt(
 wheels[i].contact_slip_x_mps * wheels[i].contact_slip_x_mps +
 wheels[i].contact_slip_y_mps * wheels[i].contact_slip_y_mps +
 wheels[i].contact_slip_z_mps * wheels[i].contact_slip_z_mps);
 assert(world_slip <= wheels[i].contact_slip_speed_mps + 1e-9);
 assert(wheels[i].contact_normal_force_N > 0.0);
 assert(std::abs(wheels[i].contact_z_m) < 0.01);
    assert(std::abs(wheels[i].pose.z_m) < 1e-9);
    const double qnorm = std::sqrt(
    wheels[i].pose.qw * wheels[i].pose.qw +
    wheels[i].pose.qx * wheels[i].pose.qx +
    wheels[i].pose.qy * wheels[i].pose.qy +
    wheels[i].pose.qz * wheels[i].pose.qz);
    assert(std::abs(qnorm - 1.0) < 1e-9);
    assert(std::abs(wheels[i].pose.qy) > 1e-3);
    }
    // Passive supports are a real SIM assembly: the wheel position includes
 // configured trail and both joint angles are finite engine read-back.
 sim::world::CasterViz casters[4];
 const int caster_count = w.casters(casters, 4);
 assert(caster_count == 2);
 for (int i = 0; i < caster_count; ++i) {
 assert(std::isfinite(casters[i].swivel_rad));
 assert(std::isfinite(casters[i].roll_rad));
 assert(std::abs(casters[i].radius_m - 0.03) < 1e-12);
 const double ex = casters[i].pivot_x_m - 0.02 *
 std::cos(casters[i].swivel_rad);
 const double ey = casters[i].pivot_y_m - 0.02 *
 std::sin(casters[i].swivel_rad);
 assert(std::abs(casters[i].x_m - ex) < 1e-12);
    assert(std::abs(casters[i].y_m - ey) < 1e-12);
    assert(casters[i].fork_pose.valid);
    assert(casters[i].wheel_pose.valid);
    assert(std::abs(casters[i].wheel_pose.z_m +
    (w.motor_params().wheel_radius_m - casters[i].radius_m)) < 1e-9);
    }
 // A compatible geometry rebuild must not restore the old root height after
 // the wheel radius or authored terrain changes. The base frame is the drive
 // axle frame, so the new minimum is terrain height + new wheel radius.
 {
 sim::world::World w = make_mujoco_diff();
 w.set_terrain_patches({{"raised", -2.0, -2.0, 2.0, 2.0, 0.25,
 0.0, 0.0, 0.0, 0.0}});
 set_diff_duty(w, 0.0);
 w.step(0.02);
 // MuJoCo's compliant contact settles within a sub-millimetre of the
 // canonical plane; the renderer's documented guard covers that tolerance.
 assert(w.truth().body_z_m >= 0.25 + 0.1 - 0.001);
 sim::world::MotorParams rebuilt;
 rebuilt.wheel_radius_m = 0.16;
 w.set_motor_params(rebuilt); // forces a model rebuild with a larger wheel
 w.step(0.02);
 assert(w.truth().body_z_m >= 0.25 + 0.16 - 0.001);
 }

 // Coast: joint damping bleeds speed over tens of seconds; over 100
    // steps it glides. Pin no-energy-creation and no-reversal, not a
    // decay rate — the engine owns the transient.
    set_diff_duty(w, 0.0);
    const double v0 = w.truth().forward_mps;
    for (int i = 0; i < 100; ++i) w.step(0.02);
    assert(w.truth().forward_mps <= v0 + 1e-3);
    assert(w.truth().forward_mps >= 0.0);
  }

  // Slope P1 on the engine (mirrors test_contact, engine tolerances:
  // contact stiction, never exact zeros — the engine owns the creep).
  {
    // Gentle incline, zero duty: bounded creep, never the P1 runaway.
    sim::world::World g = make_mujoco_diff();
    g.set_incline(0.005);
    set_diff_duty(g, 0.0);
    for (int i = 0; i < 200; ++i) g.step(0.02);
    assert(std::abs(g.truth().body_x_m) < 1.0);
    assert(std::abs(g.truth().forward_mps) < 0.5);
    // Steep grade, zero duty: breaks away backward, stays backward.
    sim::world::World steep = make_mujoco_diff();
    steep.set_incline(0.3);
    set_diff_duty(steep, 0.0);
    for (int i = 0; i < 100; ++i) steep.step(0.02);
    assert(steep.truth().body_x_m < -1.0);
    assert(steep.truth().forward_mps < -0.5);
    // Service brake holds the steep grade once its lag ramps in.
    sim::world::MotorParams mp;
    mp.brake_max_torque_Nm = 5.0;
    sim::world::World held = make_mujoco_diff();
    held.set_motor_params(mp);
    held.set_incline(0.3);
    set_diff_duty(held, 0.0);
    held.set_brake_request(0, 1.0);
    held.set_brake_request(1, 1.0);
    for (int i = 0; i < 200; ++i) held.step(0.02);
    assert(held.truth().body_x_m > -0.5);
    assert(std::abs(held.truth().forward_mps) < 0.15);
    // Non-finite grade fails closed to flat, never NaN-poisons.
    held.set_incline(std::numeric_limits<double>::quiet_NaN());
    held.step(0.02);
    assert(std::isfinite(held.truth().body_x_m));
    assert(std::isfinite(held.truth().forward_mps));
    // Slope launch: held on grade by brake, duty on + brake off climbs
    // without sustained rollback.
    sim::world::World launch = make_mujoco_diff();
    launch.set_motor_params(mp);
    launch.set_incline(0.1);
    set_diff_duty(launch, 0.0);
    launch.set_brake_request(0, 1.0);
    launch.set_brake_request(1, 1.0);
    for (int i = 0; i < 100; ++i) launch.step(0.02);
    const double x_pre = launch.truth().body_x_m;
    set_diff_duty(launch, 1.0);
    launch.set_brake_request(0, 0.0);
    launch.set_brake_request(1, 0.0);
    double x_min = x_pre;
    for (int i = 0; i < 200; ++i) {
      launch.step(0.02);
      x_min = std::min(x_min, launch.truth().body_x_m);
    }
    assert(x_min >= x_pre - 0.05);  // no sustained rollback on release
    assert(launch.truth().body_x_m > x_pre + 1.0);  // climbs the grade
    assert(launch.truth().forward_mps > 0.5);
  }

  // Determinism: identical runs, identical truth (bit-identical replay).
  {
    auto run = []() {
      sim::world::World w = make_mujoco_diff();
      set_diff_duty(w, 0.7);
      std::vector<double> xs;
      for (int i = 0; i < 100; ++i) {
        w.step(0.02);
        xs.push_back(w.truth().body_x_m);
      }
      return xs;
    };
    const auto a = run();
    const auto b = run();
    assert(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) assert(a[i] == b[i]);
  }

  // Other chassis on the engine: ackermann left arc, mecanum strafe,
  // swerve straight. Engine-appropriate tolerances throughout (no exact
  // numbers — the engine owns the transients).
  {
    // Ackermann drives a left arc.
    sim::world::World w;
    sim::world::MotorParams mp;
    mp.wheel_base_m = 0.62;
    mp.track_width_m = 0.58;
    w.set_motor_params(mp);
    robot::iface::ActuatorCommand cmd;
    cmd.mode = robot::iface::ActuatorMode::kAckermannDuty;
    cmd.stamp_ns = 0;
    cmd.expiry_ns = 1'000'000'000;
    cmd.drive[0] = cmd.drive[1] = 0.5;
    cmd.steer[0] = cmd.steer[1] = 0.5;
    w.apply(cmd);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(w.truth().forward_mps > 0.5);
    assert(w.truth().yaw_rate_radps > 0.05);  // positive steer -> left arc
    assert(w.truth().body_yaw_rad > 0.0);
    // Mecanum strafes left with ~zero forward.
    sim::world::World m;
 sim::providers::SimMotionEndpoint motion(&m);
 robot::iface::MotionFeedback feedback;
 assert(motion.poll_feedback(feedback));
    robot::iface::ActuatorCommand mc;
    mc.mode = robot::iface::ActuatorMode::kMecanumDuty;
    mc.stamp_ns = 0;
    mc.expiry_ns = 1'000'000'000;
    mc.drive = {-0.5, 0.5, 0.5, -0.5};
    m.apply(mc);
 for (int i = 0; i < 200; ++i) {
 m.step(0.02);
 assert(motion.poll_feedback(feedback));
 }
    assert(std::abs(m.truth().forward_mps) < 0.3);
    assert(m.truth().lateral_mps > 0.3);
    assert(m.truth().body_y_m > 0.1);
 std::array<double, 4> wheel_mps{};
 for (std::size_t i = 0; i < wheel_mps.size(); ++i) {
 wheel_mps[i] = feedback.drive_radps[i] *
 m.motor_params().wheel_radius_m;
 }
 double encoder_forward = 0.0;
 double encoder_lateral = 0.0;
 double encoder_yaw = 0.0;
 robot::platform::mecanum_forward(
 wheel_mps, m.motor_params().wheel_base_m,
 m.motor_params().track_width_m, encoder_forward,
 encoder_lateral, encoder_yaw);
 assert(feedback.forward_mps == encoder_forward);
 assert(feedback.lateral_mps == encoder_lateral);
 assert(feedback.yaw_rate_radps == encoder_yaw);
 assert(feedback.lateral_mps > 0.3);
 // Four-wheel layouts carry themselves: a legacy differential caster pair
 // would make a rigid six-point support. Normal-only support contacts must
 // also remain at rest without a hidden zero-friction singularity.
 sim::world::World mecanum_idle;
 robot::iface::ActuatorCommand idle_mc;
 idle_mc.mode = robot::iface::ActuatorMode::kMecanumDuty;
 idle_mc.expiry_ns = 1'000'000'000;
 mecanum_idle.apply(idle_mc);
 for (int i = 0; i < 100; ++i) mecanum_idle.step(0.02);
 sim::world::CasterViz unexpected_casters[2];
 assert(mecanum_idle.casters(unexpected_casters, 2) == 0);
 assert(std::hypot(mecanum_idle.truth().body_x_m,
 mecanum_idle.truth().body_y_m) < 1e-6);
 assert(std::abs(mecanum_idle.truth().body_roll_rad) < 1e-6);
 assert(std::abs(mecanum_idle.truth().body_pitch_rad) < 1e-6);
 sim::world::WheelViz resting_wheels[4];
 assert(mecanum_idle.wheels(resting_wheels, 4) == 4);
 for (const auto& wheel : resting_wheels) {
 assert(wheel.contact_valid);
 assert(wheel.contact_normal_force_N > 0.0);
 }
 // A symmetric pure-yaw drive stays centered and cannot exceed the wheel
 // free-speed kinematic bound. This catches unclosed body-wrench models
 // that amplify yaw independently of wheel speed.
 sim::world::World mecanum_yaw;
 robot::iface::ActuatorCommand yaw_mc = idle_mc;
 yaw_mc.drive = {-0.35, 0.35, -0.35, 0.35};
 mecanum_yaw.apply(yaw_mc);
 for (int i = 0; i < 100; ++i) mecanum_yaw.step(0.02);
 const double mecanum_arm = 0.5 *
 (mecanum_yaw.motor_params().wheel_base_m +
 mecanum_yaw.motor_params().track_width_m);
 const double yaw_free_speed_bound =
 mecanum_yaw.motor_params().free_speed_mps / mecanum_arm;
 assert(mecanum_yaw.truth().yaw_rate_radps > 0.2);
 assert(mecanum_yaw.truth().yaw_rate_radps <=
 yaw_free_speed_bound * (1.0 + 1e-6));
 assert(std::abs(mecanum_yaw.truth().forward_mps) < 0.1);
 assert(std::abs(mecanum_yaw.truth().lateral_mps) < 0.1);
    // Swerve drives straight with centered modules.
    sim::world::World s;
    robot::iface::ActuatorCommand sc;
    sc.mode = robot::iface::ActuatorMode::kSwerveDuty;
    sc.stamp_ns = 0;
    sc.expiry_ns = 1'000'000'000;
    sc.drive = {0.5, 0.5, 0.5, 0.5};
    s.apply(sc);
    for (int i = 0; i < 200; ++i) s.step(0.02);
    assert(s.truth().forward_mps > 0.5);
    assert(std::abs(s.truth().yaw_rate_radps) < 0.3);
 // Differential dry-floor launch above is the quiet control. A turning
 // Ackermann rear axle may legitimately reach the friction cone through
 // combined longitudinal and lateral demand, so it is not relabelled from
 // command intent here.
  }

  // HIL mission on the engine: crossing crosser, hold, resume, arrive.
  {
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"dock", 6.0, 0.0, 0.0});
    sim::world::Actor crosser;
    crosser.x = 2.0;
    crosser.y = -1.5;
    crosser.vx_mps = 0.0;
    crosser.vy_mps = 1.0;
    crosser.radius_m = 0.4;
    session.setup_world().add_actor(crosser);
    robot::iface::TaskRequest req;
    req.request_id = "crossing";
    req.place_id = "dock";
    assert(session.request_goal(req).valid());
    std::vector<double> xs;
    for (int i = 0; i < 280; ++i) {
      session.step_once();
      xs.push_back(session.episode().steps().back().truth_after.body_x_m);
    }
    assert(xs[8] > 0.005);  // drove toward the goal initially
    bool held = false;
    for (std::size_t i = 20; i < 120 && i < xs.size(); ++i) {
      if (xs[i] < 0.5) held = true;
    }
    assert(held);  // held while the actor was near
    assert(xs.back() > 2.0);  // resumed after the actor passed
  }

 // Friction-pair audit (drawbar doctrine): full-duty launch from
 // standstill, slip label watched over the run. Stall 54 N against
 // 117.7 N/wheel load: ice (0.15 -> 17.7 N) must slip, the 0.7 floor
 // (82 N) must hold, and a worn tire (0.4 -> 47 N) must slip on both
 // ordinary and grippy ground. The tire geom owns contact priority, so
 // MuJoCo enforces min(tire, ground) instead of its default max rule.
  {
    auto any_slip = [](sim::world::World& w, int steps) {
      bool slipped = false;
      for (int i = 0; i < steps; ++i) {
        w.step(0.02);
        for (bool s : w.truth().drive_slipping) slipped |= s;
      }
      return slipped;
    };
    // Slick zone pulls DOWN through the tire rating (the min() on the
    // tire side; without it the pair would read max(0.7, 0.15) = 0.7).
    {
      sim::world::World w = make_mujoco_diff();
      w.set_materials({{"ice", 0.15, 0.005, 0.006, 1.0, 0.3}});
      w.set_material_zones({{"belt", -2.0, -3.0, 8.0, 3.0, "ice"}});
      set_diff_duty(w, 1.0);
      assert(any_slip(w, 100));
    }
    // Bare floor holds the same launch (control: demand < capacity).
    {
      sim::world::World w = make_mujoco_diff();
      set_diff_duty(w, 1.0);
      assert(!any_slip(w, 200));
    }
 // Split-mu is resolved per contact, not from an averaged axle load. The
 // left drive starts on ice while the right remains on dry floor; at least
 // one solve must report the low-grip wheel independently before vehicle
 // yaw carries the axle away from the authored boundary.
 {
 sim::world::World w = make_mujoco_diff();
 sim::world::MotorParams split_plant = w.motor_params();
 split_plant.drive_axle_x_m = 0.1;
 split_plant.stall_force_N = 10.0;
 split_plant.support_count = 2;
 split_plant.supports[0].x_m = -0.2;
 split_plant.supports[0].y_m = 0.10;
 split_plant.supports[1].x_m = -0.2;
 split_plant.supports[1].y_m = -0.10;
 w.set_motor_params(split_plant);
 w.set_materials({{"ice", 0.05, 0.005, 0.006, 1.0, 0.3}});
 w.set_material_zones({{"left_ice", -2.0, 0.02, 3.0, 2.0, "ice"}});
 set_diff_duty(w, 0.0);
 for (int i = 0; i < 100; ++i) w.step(0.02);
 robot::iface::ActuatorCommand split_cmd;
 split_cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
 split_cmd.stamp_ns = 0;
 split_cmd.expiry_ns = 1'000'000'000;
 split_cmd.drive[0] = 1.0;
 split_cmd.drive[1] = 1.0;
 w.apply(split_cmd);
 bool left_only_slip = false;
 for (int i = 0; i < 100; ++i) {
 w.step(0.02);
 left_only_slip |= w.truth().drive_slipping[0] &&
 !w.truth().drive_slipping[1];
 }
 assert(left_only_slip);
 }
    // Worn tire governs off-zone (rating path works where allowed).
    {
      sim::world::World w = make_mujoco_diff();
      sim::world::MotorParams mp;
      mp.tire_friction_mu = 0.4;
      w.set_motor_params(mp);
      set_diff_duty(w, 1.0);
      assert(any_slip(w, 100));
    }
 // Worn tire remains the limit on grippy ground: asphalt cannot repair
 // tire rubber, and the slip evidence follows that same physical pair.
    {
      sim::world::World w = make_mujoco_diff();
      sim::world::MotorParams mp;
      mp.tire_friction_mu = 0.4;
      w.set_motor_params(mp);
      w.set_materials({{"asphalt", 1.0, 0.005, 0.006, 1.0, 0.3}});
      w.set_material_zones({{"pad", -2.0, -3.0, 8.0, 3.0, "asphalt"}});
      set_diff_duty(w, 1.0);
 assert(any_slip(w, 200));
    }
  }

  // Chassis hard geometry (config, never constants): axle offset lands in
  // the wheel table, explicit supports build and stand, bad geometry
  // refuses with frozen truth (same fail-closed contract as bad zones).
  {
    // Axle offset shifts the drive pair (default 0 = legacy plant).
    sim::world::World w = make_mujoco_diff();
    sim::world::MotorParams mp;
    mp.drive_axle_x_m = 0.1;
    w.set_motor_params(mp);
    set_diff_duty(w, 0.5);
    for (int i = 0; i < 10; ++i) w.step(0.02);
    sim::world::WheelViz table[6];
    const int n = w.wheels(table, 6);
    assert(n == 2);
    assert(table[0].x_m == 0.1 && table[1].x_m == 0.1);
    // Explicit single rear support (tripod-ish): builds, stands, drives.
    sim::world::World t = make_mujoco_diff();
    sim::world::MotorParams mt;
    mt.support_count = 1;
    mt.supports[0].kind = "caster";
    mt.supports[0].x_m = -0.2;
    mt.supports[0].y_m = 0.04;
    mt.supports[0].trail_m = 0.03;
    mt.supports[0].radius_m = 0.03;
    mt.supports[0].mass_kg = 0.1;
    mt.supports[0].friction_mu = 0.05;
    mt.supports[0].swivel_damping_Nms = 0.005;
    mt.supports[0].swivel_friction_Nm = 0.001;
    mt.supports[0].rolling_friction_m = 0.0005;
    t.set_motor_params(mt);
    set_diff_duty(t, 0.6);
    for (int i = 0; i < 50; ++i) t.step(0.02);
    assert(t.truth().body_z_m > 0.05);  // stands, never tips through
    assert(std::abs(t.truth().body_pitch_rad) < 0.1);
    sim::world::CasterViz configured_caster[1];
    assert(t.casters(configured_caster, 1) == 1);
    assert(std::abs(configured_caster[0].pivot_y_m - 0.04) < 1e-12);
    assert(std::abs(std::hypot(
      configured_caster[0].pivot_x_m - configured_caster[0].x_m,
      configured_caster[0].pivot_y_m - configured_caster[0].y_m) - 0.03) < 1e-9);
    assert(t.truth().body_x_m > 0.1);  // drives, not beached
    // Bad deck (belly at/below the floor): refused, truth frozen.
    sim::world::World bad = make_mujoco_diff();
    sim::world::MotorParams mb;
    mb.body_half_height_m = 0.5;
    bad.set_motor_params(mb);
    set_diff_duty(bad, 0.6);
    for (int i = 0; i < 10; ++i) bad.step(0.02);
    assert(bad.truth().forward_mps == 0.0);
    // Bad support kind: refused, truth frozen.
    sim::world::World bad2 = make_mujoco_diff();
    sim::world::MotorParams mb2;
    mb2.support_count = 1;
    mb2.supports[0].kind = "swivel";
    mb2.supports[0].x_m = -0.2;
    mb2.supports[0].radius_m = 0.03;
    mb2.supports[0].mass_kg = 0.1;
    bad2.set_motor_params(mb2);
    set_diff_duty(bad2, 0.6);
    for (int i = 0; i < 10; ++i) bad2.step(0.02);
    assert(bad2.truth().forward_mps == 0.0);
    // Config switch rebuilds (signature covers every build input): an
    // axle shift on a live world moves the drive pair in the wheel table
    // (previously the stale model survived silently).
    {
      sim::world::World w = make_mujoco_diff();
      set_diff_duty(w, 0.5);
      for (int i = 0; i < 5; ++i) w.step(0.02);
      sim::world::MotorParams ms = w.motor_params();
      ms.drive_axle_x_m = -0.1;
      w.set_motor_params(ms);
      for (int i = 0; i < 5; ++i) w.step(0.02);
      sim::world::WheelViz table[6];
      assert(w.wheels(table, 6) == 2);
      assert(table[0].x_m == -0.1 && table[1].x_m == -0.1);
    // A topology-changing rebuild preserves every compatible named joint.
    // Removing one caster must not teleport the base or reset drive motion;
    // future arm joints follow the same name/type/dimension contract.
    const double x_before_topology = w.truth().body_x_m;
    const double v_before_topology = w.truth().forward_mps;
    const double drive_before_topology = w.truth().drive_angle_rad[0];
    ms.support_count = 1;
    ms.supports[0] = sim::world::MotorParams::SupportParams{};
    ms.supports[0].x_m = -0.2;
    ms.supports[0].y_m = 0.04;
    ms.supports[0].trail_m = 0.03;
    w.set_motor_params(ms);
 w.prepare();
    assert(std::abs(w.truth().body_x_m - x_before_topology) < 1e-3);
    assert(std::abs(w.truth().forward_mps - v_before_topology) < 0.05);
    assert(std::abs(w.truth().drive_angle_rad[0] - drive_before_topology) < 0.01);
    sim::world::CasterViz rebuilt_caster[2];
    assert(w.casters(rebuilt_caster, 2) == 1);
    assert(std::abs(rebuilt_caster[0].pivot_y_m - 0.04) < 1e-12);
    }
    // Declared payload (5 kg at 0.3 m): builds, stands, covers less
    // ground in fixed time (same stall force, more mass — the transient
    // lag integrates to a persistent distance gap, deterministic).
    sim::world::World laden = make_mujoco_diff();
    sim::world::MotorParams ml;
    ml.payload_mass_kg = 5.0;
    ml.payload_height_m = 0.3;
    laden.set_motor_params(ml);
    set_diff_duty(laden, 1.0);
    for (int i = 0; i < 100; ++i) laden.step(0.02);
    assert(laden.truth().body_z_m > 0.05);
    sim::world::World bare = make_mujoco_diff();
    set_diff_duty(bare, 1.0);
    for (int i = 0; i < 100; ++i) bare.step(0.02);
    assert(laden.truth().body_x_m < bare.truth().body_x_m);
    assert(bare.truth().body_x_m > 0.5);  // control actually drove
  }
#else
  // MuJoCo disabled: backend exists but reports invalid, never steps.
  {
    sim::world::World w;
    set_diff_duty(w, 1.0);
    w.step(0.02);
    assert(w.truth().forward_mps == 0.0);
  }
#endif

  std::printf("mujoco_backend ok\n");
  return 0;
}
