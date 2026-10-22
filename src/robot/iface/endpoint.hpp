#pragma once

#include <string>

namespace lost_signal::robot::iface {

// Endpoint roles. One endpoint owns exactly one role; two providers must
// never drive the same (endpoint_id, role).
enum class EndpointRole {
  kLidar = 0,
  kImu = 1,
  kEncoder = 2,
  kActuator = 3,
  kPower = 4,
};

enum class ProviderBackend {
  kSim = 0,
  kReplay = 1,
  kNative = 2,
};

struct EndpointDescriptor {
  std::string endpoint_id;  // stable, e.g. "ro1.lidar_front"
  EndpointRole role = EndpointRole::kLidar;
  ProviderBackend backend = ProviderBackend::kSim;
  std::string frame_id;  // e.g. "lidar_link"
};

// Ownership gate: one (endpoint_id, role) -> one backend.
class EndpointRegistry {
 public:
  bool register_endpoint(const EndpointDescriptor& desc);
  bool owns(const std::string& endpoint_id, EndpointRole role) const;

 private:
  static constexpr int kMaxEndpoints = 32;
  EndpointDescriptor entries_[kMaxEndpoints]{};
  int count_ = 0;
};

}  // namespace lost_signal::robot::iface
