#include "robot/iface/endpoint.hpp"

namespace lost_signal::robot::iface {

bool EndpointRegistry::register_endpoint(const EndpointDescriptor& desc) {
  if (desc.endpoint_id.empty()) return false;
  for (int i = 0; i < count_; ++i) {
    if (entries_[i].endpoint_id == desc.endpoint_id &&
        entries_[i].role == desc.role) {
      return false;  // already owned: two drivers must never share it
    }
  }
  if (count_ >= kMaxEndpoints) return false;
  entries_[count_++] = desc;
  return true;
}

bool EndpointRegistry::owns(const std::string& endpoint_id,
                            EndpointRole role) const {
  for (int i = 0; i < count_; ++i) {
    if (entries_[i].endpoint_id == endpoint_id && entries_[i].role == role) {
      return true;
    }
  }
  return false;
}

}  // namespace lost_signal::robot::iface
