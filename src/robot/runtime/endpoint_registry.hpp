#pragma once

#include <map>
#include <string>
#include <vector>

namespace sim::runtime {

enum class EndpointRole { ObservationSource, CommandSink };
enum class EndpointKind { Lidar, Imu, Encoder, JointState, Actuator, Map };
enum class ProviderBackend { Simulated, Native, Replay, Remote };

// A provider owns one endpoint, not an entire robot. This permits a real
// LiDAR to run beside simulated IMU, encoder, and actuator endpoints.
struct EndpointDescriptor {
 std::string endpoint_id;
 std::string provider_id;
 std::string source_id;
 std::string frame_id;
 std::string clock_id;
 EndpointRole role{EndpointRole::ObservationSource};
 EndpointKind kind{EndpointKind::Lidar};
 ProviderBackend backend{ProviderBackend::Simulated};
 double nominal_rate_hz{0.0};
};

class EndpointRegistry {
 public:
 void register_endpoint(EndpointDescriptor descriptor);
 bool contains(const std::string& endpoint_id, EndpointRole role) const;
 const EndpointDescriptor& endpoint(const std::string& endpoint_id, EndpointRole role) const;
 std::vector<EndpointDescriptor> endpoints() const;

 private:
 using Key = std::pair<std::string, EndpointRole>;
 std::map<Key, EndpointDescriptor> endpoints_;
};

} // namespace sim::runtime
