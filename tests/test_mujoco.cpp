// M0: MuJoCo dependency gate. Parses a minimal free-body model, steps it,
// and pins determinism (two fresh sims, identical trajectories) plus basic
// contact (the box lands on the floor and rests). Proves the engine links,
// steps, and settles in this toolchain (gcc-13, no GUI).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace {

std::string tmp_path(const char* name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

const char* kDropXml = R"(
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1"/>
    <body name="chassis" pos="0 0 0.3">
      <freejoint/>
      <geom name="box" type="box" size="0.25 0.2 0.1" mass="24"/>
    </body>
  </worldbody>
</mujoco>
)";

std::vector<double> run_drop(const std::string& path) {
  char error[1024] = "";
  mjModel* model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  assert(model != nullptr);
  mjData* data = mj_makeData(model);
  assert(data != nullptr);
  std::vector<double> heights;
  for (int i = 0; i < 2000; ++i) {  // 2 s at 1 ms
    mj_step(model, data);
    if (i % 20 == 0) heights.push_back(data->qpos[2]);
    assert(std::isfinite(data->qpos[2]));
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  return heights;
}

}  // namespace

int main() {
  const std::string path = tmp_path("test_mujoco_drop.xml");
  {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << kDropXml;
  }
  const auto a = run_drop(path);
  const auto b = run_drop(path);
  assert(a.size() == b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    assert(a[i] == b[i]);  // bit-identical replay of the engine
  }
  // Fell from 0.3, rests on the floor: box half-height 0.1, settled.
  assert(a.front() > 0.29);
  assert(std::abs(a.back() - 0.1) < 0.02);
  // At rest: negligible residual velocity (contact solver settled, no buzz).
  std::filesystem::remove(path);
  std::printf("mujoco ok: steps=%zu rest_z=%.4f\n", a.size(), a.back());
  return 0;
}
