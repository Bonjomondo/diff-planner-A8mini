#pragma once

#include <cmath>

namespace px4ctrl_guard {

// Ground contact/disarmed thrust does not represent freely flying rotor thrust.
// Stale flight state must not allow adaptive model updates either.
inline bool allowThrustEstimation(bool hover_or_command, bool connected,
                                 bool armed, bool offboard, bool in_air,
                                 double state_age, double extended_age) {
  const auto fresh = [](double age) {
    return std::isfinite(age) && age >= 0.0 && age <= 2.5;
  };
  return hover_or_command && connected && armed && offboard && in_air &&
         fresh(state_age) && fresh(extended_age);
}

}  // namespace px4ctrl_guard
