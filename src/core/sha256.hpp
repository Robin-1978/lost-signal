#pragma once

#include <array>
#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

namespace sim {

// Small dependency-free SHA-256 implementation used for run identities.
// It is deliberately a byte hash, not a serializer: callers define and test
// their own canonical byte stream before hashing it.
class Sha256 {
 public:
 void update(std::string_view bytes) {
 update(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size());
 }

 void update(const unsigned char* bytes, std::size_t size) {
 total_size_ += size;
 while (size > 0) {
 const std::size_t copied = std::min(size, block_.size() - block_size_);
 for (std::size_t index = 0; index < copied; ++index)
 block_[block_size_ + index] = bytes[index];
 block_size_ += copied;
 bytes += copied;
 size -= copied;
 if (block_size_ == block_.size()) {
 transform(block_.data());
 block_size_ = 0;
 }
 }
 }

 std::string hex_digest() const {
 Sha256 final = *this;
 const std::uint64_t bits = final.total_size_ * 8;
 const unsigned char marker = 0x80;
 final.update(&marker, 1);
 const unsigned char zero = 0;
 while (final.block_size_ != 56)
 final.update(&zero, 1);
 std::array<unsigned char, 8> length{};
 for (std::size_t index = 0; index < length.size(); ++index)
 length[length.size() - 1 - index] = static_cast<unsigned char>(bits >> (index * 8));
 final.update(length.data(), length.size());

 std::ostringstream output;
 output << std::hex << std::setfill('0');
 for (const std::uint32_t word : final.state_)
 output << std::setw(8) << word;
 return output.str();
 }

 private:
 static constexpr std::array<std::uint32_t, 64> kRound = {
 0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
 };

 static std::uint32_t read_word(const unsigned char* bytes) {
 return static_cast<std::uint32_t>(bytes[0]) << 24 |
 static_cast<std::uint32_t>(bytes[1]) << 16 |
 static_cast<std::uint32_t>(bytes[2]) << 8 |
 static_cast<std::uint32_t>(bytes[3]);
 }

 void transform(const unsigned char* bytes) {
 std::array<std::uint32_t, 64> words{};
 for (std::size_t index = 0; index < 16; ++index)
 words[index] = read_word(bytes + index * 4);
 for (std::size_t index = 16; index < words.size(); ++index) {
 const std::uint32_t s0 = std::rotr(words[index - 15], 7) ^ std::rotr(words[index - 15], 18) ^
 (words[index - 15] >> 3);
 const std::uint32_t s1 = std::rotr(words[index - 2], 17) ^ std::rotr(words[index - 2], 19) ^
 (words[index - 2] >> 10);
 words[index] = words[index - 16] + s0 + words[index - 7] + s1;
 }

 std::uint32_t a = state_[0];
 std::uint32_t b = state_[1];
 std::uint32_t c = state_[2];
 std::uint32_t d = state_[3];
 std::uint32_t e = state_[4];
 std::uint32_t f = state_[5];
 std::uint32_t g = state_[6];
 std::uint32_t h = state_[7];
 for (std::size_t index = 0; index < words.size(); ++index) {
 const std::uint32_t sum1 = std::rotr(e, 6) ^ std::rotr(e, 11) ^ std::rotr(e, 25);
 const std::uint32_t choose = (e & f) ^ (~e & g);
 const std::uint32_t first = h + sum1 + choose + kRound[index] + words[index];
 const std::uint32_t sum0 = std::rotr(a, 2) ^ std::rotr(a, 13) ^ std::rotr(a, 22);
 const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
 const std::uint32_t second = sum0 + majority;
 h = g;
 g = f;
 f = e;
 e = d + first;
 d = c;
 c = b;
 b = a;
 a = first + second;
 }
 state_[0] += a;
 state_[1] += b;
 state_[2] += c;
 state_[3] += d;
 state_[4] += e;
 state_[5] += f;
 state_[6] += g;
 state_[7] += h;
 }

 std::array<std::uint32_t, 8> state_ = {
 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
 };
 std::array<unsigned char, 64> block_{};
 std::size_t block_size_{0};
 std::uint64_t total_size_{0};
};

inline std::string sha256(std::string_view bytes) {
 Sha256 hash;
 hash.update(bytes);
 return hash.hex_digest();
}

} // namespace sim
