#pragma once

#include <cmath>

namespace mission_guard
{
inline bool fresh(double age, double timeout)
{
    return std::isfinite(age) && age >= 0.0 && age <= timeout;
}

struct FlightStatus
{
    bool fresh_state = false;
    bool connected = false;
    bool armed = false;
    bool offboard = false;
    bool in_air = false;
    bool controller_fresh = false;
    bool controller_ready = false;
    bool hover_or_command = false;
};

inline bool flightReady(const FlightStatus &s, bool starting)
{
    return s.fresh_state && s.connected && s.armed && s.offboard && s.in_air &&
           s.controller_fresh && s.hover_or_command && (!starting || s.controller_ready);
}
}  // namespace mission_guard
