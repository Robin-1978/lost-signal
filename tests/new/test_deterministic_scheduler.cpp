#include "mission/systems/deterministic_scheduler.hpp"

#include <iostream>
#include <stdexcept>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 mission::DeterministicScheduler schedule;
 schedule.reset(0.1, 0.2, 0.04);
 CHECK(schedule.scan_due(0.0));
 CHECK(schedule.decision_due(0.0));
 CHECK(schedule.replay_due(0.0));
 CHECK(!schedule.scan_due(0.02));
 CHECK(!schedule.decision_due(0.02));
 CHECK(!schedule.replay_due(0.02));
 CHECK(schedule.replay_due(0.04));
 CHECK(schedule.scan_due(0.1));
 CHECK(!schedule.decision_due(0.1));
 CHECK(schedule.decision_due(0.2));

 // Deadlines advance from their previous value, not from a delayed caller,
 // so fixed-step scheduling cannot accumulate phase drift.
 mission::PeriodicDeadline delayed;
 delayed.reset(0.1);
 CHECK(delayed.due(0.0));
 CHECK(delayed.due(0.12));
 CHECK(delayed.next_s() == 0.2);

 // A disabled device owns no deadline and never fires.
 mission::PeriodicDeadline disabled;
 disabled.reset(0.0);
 CHECK(!disabled.due(100.0));

 bool rejected = false;
 try {
 disabled.reset(-0.1);
 } catch (const std::invalid_argument&) {
 rejected = true;
 }
 CHECK(rejected);
 return 0;
}
