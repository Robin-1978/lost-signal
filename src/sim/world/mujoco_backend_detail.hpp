#pragma once

// Private backend state shared by the three backend translation units
// (mujoco_backend.cpp root, mujoco_build.cpp assembly, mujoco_step.cpp
// duty loop). NOT part of the public header: dependents of
// mujoco_backend.hpp never pay for MuJoCo types. One definition, three
// users — hence a detail header, not triplication.
#ifdef LOST_SIGNAL_WITH_MUJOCO

#include <array>

#include <mujoco/mujoco.h>

#include "sim/world/mujoco_backend.hpp"
#include "sim/world/service_brake.hpp"

namespace lost_signal::sim::world {

struct MujocoBackend::Impl {
  mjModel* model = nullptr;
  mjData* data = nullptr;
  std::array<ServiceBrakePlant, 4> brakes;
  ~Impl() {
    if (data != nullptr) mj_deleteData(data);
    if (model != nullptr) mj_deleteModel(model);
  }
};

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
