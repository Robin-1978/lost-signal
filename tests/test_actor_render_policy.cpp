#include <cassert>
#include <cstdio>

#include "sim/viewer/actor_render_policy.hpp"

int main() {
 using lost_signal::sim::viewer::ActorRenderPath;
 using lost_signal::sim::viewer::choose_actor_render_path;

 assert(choose_actor_render_path(false, true, true, true) ==
 ActorRenderPath::kModularForklift);
 assert(choose_actor_render_path(false, false, true, true) ==
 ActorRenderPath::kFullModel);
 assert(choose_actor_render_path(false, false, true, false) ==
 ActorRenderPath::kProceduralFallback);
 assert(choose_actor_render_path(false, false, false, true) ==
 ActorRenderPath::kProceduralFallback);
 // Worker models never take the forklift component path, even if an asset
 // cache happens to contain complete forklift parts.
 assert(choose_actor_render_path(true, true, true, true) ==
 ActorRenderPath::kFullModel);
 assert(choose_actor_render_path(true, false, false, false) ==
 ActorRenderPath::kProceduralFallback);

 std::printf("actor_render_policy ok\n");
 return 0;
}
