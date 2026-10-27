#include "sim/viewer/actor_render_policy.hpp"

namespace lost_signal::sim::viewer {

ActorRenderPath choose_actor_render_path(bool pedestrian,
 bool modular_forklift_ready, bool full_model_ready,
 bool model_definition_ready) {
 if (!pedestrian && modular_forklift_ready) {
 return ActorRenderPath::kModularForklift;
 }
 if (full_model_ready && model_definition_ready) {
 return ActorRenderPath::kFullModel;
 }
 return ActorRenderPath::kProceduralFallback;
}

} // namespace lost_signal::sim::viewer
