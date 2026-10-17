#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace sim {

// Fixed-step simulation samples a device and delivers its output at separate
// times. This queue orders equal-time deliveries by production order so a
// deterministic run never depends on a container or wall-clock scheduler.
template <typename Payload> class DeterministicDeliveryQueue {
 public:
 explicit DeterministicDeliveryQueue(std::size_t capacity = 64)
 : capacity_(capacity) {
 if (capacity_ == 0)
 throw std::invalid_argument("delivery queue capacity must be positive");
 }

 bool enqueue(double available_time_s, Payload payload) {
 if (!std::isfinite(available_time_s) || available_time_s < 0.0)
 throw std::invalid_argument("delivery time must be finite and non-negative");
 if (items_.size() >= capacity_)
 return false;
 Item item{available_time_s, next_order_++, std::move(payload)};
 const auto position = std::upper_bound(items_.begin(), items_.end(), item,
 [](const Item& left, const Item& right) {
 if (left.available_time_s != right.available_time_s)
 return left.available_time_s < right.available_time_s;
 return left.production_order < right.production_order;
 });
 items_.insert(position, std::move(item));
 return true;
 }

 std::optional<Payload> pop_ready(double now_s) {
 if (!std::isfinite(now_s) || now_s < 0.0)
 throw std::invalid_argument("delivery clock must be finite and non-negative");
 if (items_.empty() || items_.front().available_time_s > now_s + 1e-12)
 return std::nullopt;
 Payload payload = std::move(items_.front().payload);
 items_.erase(items_.begin());
 return payload;
 }

 void clear() {
 items_.clear();
 next_order_ = 0;
 }

 std::size_t size() const { return items_.size(); }
 bool empty() const { return items_.empty(); }
 std::size_t capacity() const { return capacity_; }
 std::optional<double> next_available_time_s() const {
 if (items_.empty())
 return std::nullopt;
 return items_.front().available_time_s;
 }

 private:
 struct Item {
 double available_time_s{0.0};
 std::uint64_t production_order{0};
 Payload payload;
 };

 std::size_t capacity_{64};
 std::uint64_t next_order_{0};
 std::vector<Item> items_;
};

} // namespace sim
