#include <array>
#include <cassert>
#include <filesystem>

int main() {
 const std::filesystem::path root = TEST_SOURCE_DIR;
 const std::filesystem::path audio = root / "assets" / "audio";
 const std::array<const char*, 9> expected{{
     "collision.wav", "decision_wait.wav", "decision_clear.wav",
     "relay_restored.wav", "motor_drive.wav", "wheel_roll.wav",
     "tire_slip.wav", "service_brake.wav", "environment_bed.wav"}};
 for (const char* name : expected) {
   assert(std::filesystem::is_regular_file(audio / name));
 }
 for (const auto& entry : std::filesystem::directory_iterator(audio)) {
   if (!entry.is_regular_file() || entry.path().extension() != ".wav") continue;
   bool declared = false;
   for (const char* name : expected) declared |= entry.path().filename() == name;
   assert(declared);
 }
 assert(std::filesystem::is_regular_file(audio / "manifest.json"));
 assert(std::filesystem::is_regular_file(audio / "LICENSES.md"));
 return 0;
}
