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

// 连续失败计时：failing_now 为假时清零；为真时从首次进入开始累计，
// 累计时长达到 debounce 才返回 true。since 由调用方持有（0 表示未在失败中），
// now 需单调递增（用 ros::WallTime::toSec()）。
inline bool sustained(double now, bool failing_now, double& since, double debounce)
{
    if (!failing_now)
    {
        since = 0.0;
        return false;
    }
    if (since <= 0.0) since = now;
    return std::isfinite(now) && std::isfinite(debounce) && debounce >= 0.0 &&
           now - since >= debounce;
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
