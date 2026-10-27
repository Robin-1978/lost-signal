#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "robot/transport/native_links.hpp"

int main() {
 using namespace lost_signal::robot::transport;

 UdpLink receiver;
 UdpLink sender;
 assert(receiver.bind("127.0.0.1", 0));
 assert(sender.bind("127.0.0.1", 0));
 const std::uint8_t payload[] = {0x16, 0x54, 0x50, 1, 2, 3};
 assert(sender.send_to("127.0.0.1", receiver.local_port(), payload,
 sizeof(payload)));
 std::uint8_t received[32]{};
 assert(receiver.receive(received, sizeof(received), 1000) ==
 static_cast<int>(sizeof(payload)));
 for (std::size_t i = 0; i < sizeof(payload); ++i) assert(received[i] == payload[i]);

 std::vector<std::uint8_t> long_frame(37);
 for (std::size_t i = 0; i < long_frame.size(); ++i) {
 long_frame[i] = static_cast<std::uint8_t>(i + 1);
 }
 std::vector<CanChunk> chunks;
 assert(CanSegmenter::split(0x123, 9, long_frame.data(), long_frame.size(),
 chunks));
 assert(chunks.size() == 10);
 CanSegmenter assembler;
 std::vector<std::uint8_t> complete;
 for (const auto& chunk : chunks) {
 const bool finished = assembler.accept(chunk, complete);
 if (&chunk != &chunks.back()) assert(!finished);
 }
 assert(complete == long_frame);

 // A transfer with a missing middle chunk is refused and does not leak a
 // partial measurement to the provider boundary.
 assembler.reset();
 assert(!assembler.accept(chunks[0], complete));
 assert(!assembler.accept(chunks[2], complete));
 assert(complete.empty());

 std::puts("native_links ok");
 return 0;
}
