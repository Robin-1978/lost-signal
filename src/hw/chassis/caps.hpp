#pragma once

namespace sim {

// What a chassis can physically do. Recovery and planning logic must branch
// on this instead of assuming differential drive: a diff drive spins in
// place, an ackermann cannot (it reverses with steering instead).
struct ChassisCaps {
 bool can_spin{true};
};

} // namespace sim
