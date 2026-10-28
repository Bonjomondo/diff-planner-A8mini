#include "thrust_estimation_guard.h"
#include <cassert>
#include <limits>

int main() {
  using px4ctrl_guard::allowThrustEstimation;
  // Every single unsafe flight-state combination must freeze adaptation.
  for (int mask = 0; mask < 32; ++mask) {
    const bool hover = mask & 1, connected = mask & 2, armed = mask & 4;
    const bool offboard = mask & 8, in_air = mask & 16;
    assert(allowThrustEstimation(hover, connected, armed, offboard, in_air,
                                 0.1, 0.1) == (mask == 31));
  }
  const double invalid[] = {-0.001, 2.501,
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()};
  for (double age : invalid) {
    assert(!allowThrustEstimation(true, true, true, true, true, age, 0.1));
    assert(!allowThrustEstimation(true, true, true, true, true, 0.1, age));
  }
  assert(allowThrustEstimation(true, true, true, true, true, 0.0, 2.5));
  // The recorded post-landing state was AUTO_HOVER + OFFBOARD + disarmed.
  assert(!allowThrustEstimation(true, true, false, true, false, 0.1, 0.1));
  // A landed aircraft must also freeze adaptation while still armed.
  assert(!allowThrustEstimation(true, true, true, true, false, 0.1, 0.1));
}
