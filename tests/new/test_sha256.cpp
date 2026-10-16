#include "core/sha256.hpp"

#include <iostream>
#include <string>

namespace {

int failures = 0;

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << "FAIL line " << __LINE__ << ": " #condition "\n"; \
 ++failures; \
 } \
 } while (false)

} // namespace

int main() {
 CHECK(sim::sha256("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
 CHECK(sim::sha256("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");

 sim::Sha256 incremental;
 incremental.update("Lost ");
 incremental.update("Signal");
 CHECK(incremental.hex_digest() == sim::sha256("Lost Signal"));
 CHECK(incremental.hex_digest() == sim::sha256("Lost Signal"));
 return failures == 0 ? 0 : 1;
}
