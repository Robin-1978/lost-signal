#include <cassert>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include "robot/iface/ring_buffer.hpp"

int main() {
 using lost_signal::robot::iface::RingBuffer;

 RingBuffer ring(8);
 const std::uint8_t bytes[] = {1, 2, 3, 4, 5};
 assert(ring.push(bytes, sizeof(bytes)) == sizeof(bytes));
 std::uint8_t peeked[3]{};
 assert(ring.peek(peeked, sizeof(peeked)) == sizeof(peeked));
 assert(peeked[0] == 1 && peeked[2] == 3);
 assert(ring.discard(2) == 2);
 std::uint8_t popped[3]{};
 assert(ring.pop(popped, sizeof(popped)) == sizeof(popped));
 assert(popped[0] == 3 && popped[2] == 5);
 assert(ring.empty());

 RingBuffer full(4);
 assert(full.push(bytes, sizeof(bytes)) == 4);
 assert(full.push(bytes, 1) == 0);
 assert(full.overwrite(bytes, sizeof(bytes)) == sizeof(bytes));
 assert(full.size() == 4);

 // A driver thread may append arbitrary chunks while the consumer parses.
 // The byte ring must preserve every ordered byte and never expose a torn
 // prefix. No sensor/robot time is involved in this test.
 RingBuffer concurrent(64);
 std::vector<std::uint8_t> received;
 std::thread producer([&concurrent] {
 for (int i = 0; i < 100; ++i) {
 const std::uint8_t value = static_cast<std::uint8_t>(i);
 while (concurrent.push(&value, 1) == 0) {}
 }
 });
 while (received.size() < 100) {
 std::uint8_t value = 0;
 if (concurrent.pop(&value, 1) == 1) received.push_back(value);
 }
 producer.join();
 assert(received.size() == 100);
 for (int i = 0; i < 100; ++i) {
 assert(received[static_cast<std::size_t>(i)] ==
 static_cast<std::uint8_t>(i));
 }

 std::puts("ring_buffer ok");
 return 0;
}
