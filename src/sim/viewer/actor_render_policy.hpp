#pragma once

namespace lost_signal::sim::viewer {

enum class ActorRenderPath {
 kProceduralFallback = 0,
 kFullModel = 1,
 kModularForklift = 2,
};

// Selects exactly one visual authority for an actor. A complete modular
// forklift wins over its legacy full model; procedural geometry is used only
// when no usable asset path is available.
ActorRenderPath choose_actor_render_path(bool pedestrian,
 bool modular_forklift_ready, bool full_model_ready,
 bool model_definition_ready);

} // namespace lost_signal::sim::viewer
