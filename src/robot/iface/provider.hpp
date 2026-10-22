#pragma once

#include <string>
#include <type_traits>

#include "robot/iface/endpoint.hpp"
#include "robot/iface/message.hpp"

namespace lost_signal::robot::iface {

// IProvider: non-template provider base. Every sensor/actuator provider owns
// exactly one EndpointDescriptor (claimed via EndpointRegistry); ROBOT only
// consumes typed values through poll(), never provider private state.
class IProvider {
 public:
  virtual ~IProvider() = default;
  virtual const EndpointDescriptor& descriptor() const = 0;
  virtual bool is_ready() const = 0;
};

// ISensorProvider<T>: typed poll contract for one message type.
// Sim, replay, and native drivers implement the same seam:
//   Sim2DLidar / Sim16Lidar / Real2DLidar are all ISensorProviders.
template <typename MessageT>
class ISensorProvider : public IProvider {
 public:
  static_assert(std::is_base_of<IMessage, MessageT>::value,
                "sensor message must implement IMessage");
  virtual bool poll(MessageT& out) = 0;
};

}  // namespace lost_signal::robot::iface
