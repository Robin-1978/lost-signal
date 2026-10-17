#include "robot/runtime/endpoint_registry.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim::runtime {

void EndpointRegistry::register_endpoint(EndpointDescriptor descriptor) {
 if (descriptor.endpoint_id.empty() || descriptor.provider_id.empty() ||
 descriptor.source_id.empty() || descriptor.frame_id.empty() ||
 descriptor.clock_id.empty() || !std::isfinite(descriptor.nominal_rate_hz) ||
 descriptor.nominal_rate_hz <= 0.0) {
 throw std::invalid_argument("LSRI endpoint metadata is incomplete or invalid");
 }
 Key key{descriptor.endpoint_id, descriptor.role};
 if (!endpoints_.emplace(std::move(key), std::move(descriptor)).second) {
 throw std::runtime_error("LSRI endpoint already has an owner");
 }
}

bool EndpointRegistry::contains(const std::string& endpoint_id, EndpointRole role) const {
 return endpoints_.contains({endpoint_id, role});
}

const EndpointDescriptor& EndpointRegistry::endpoint(const std::string& endpoint_id,
 EndpointRole role) const {
 const auto found = endpoints_.find({endpoint_id, role});
 if (found == endpoints_.end())
 throw std::out_of_range("unknown LSRI endpoint");
 return found->second;
}

std::vector<EndpointDescriptor> EndpointRegistry::endpoints() const {
 std::vector<EndpointDescriptor> result;
 result.reserve(endpoints_.size());
 for (const auto& [key, descriptor] : endpoints_) {
 (void)key;
 result.push_back(descriptor);
 }
 return result;
}

} // namespace sim::runtime
