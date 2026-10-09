#pragma once

#include <cmath>

namespace mission_guard
{
inline bool fresh(double age, double timeout)
{
    return std::isfinite(age) && age >= 0.0 && age <= timeout;
}

// Sensor stamps are mapped from the FCU clock to ROS time by MAVROS. Small
// synchronization errors can put a freshly received message ahead of ROS time.
// Keep receipt-time/heartbeat checks strict; apply this tolerance only to stamps.
inline bool stampedFresh(double age, double timeout, double future_tolerance)
{
    return std::isfinite(age) && std::isfinite(timeout) && timeout >= 0.0 &&
           std::isfinite(future_tolerance) && future_tolerance >= 0.0 &&
           age >= -future_tolerance && age <= timeout;
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
