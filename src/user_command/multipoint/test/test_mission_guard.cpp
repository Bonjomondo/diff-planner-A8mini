#include "../src/mission_guard.h"
#include <cassert>
#include <limits>
#include <initializer_list>

int main()
{
    using namespace mission_guard;
    FlightStatus status;
    // Recorded 20260916 run: ground, disarmed, ALTCTL.
    status.fresh_state = status.connected = status.controller_fresh = true;
    assert(!flightReady(status, true));
    status.armed = status.offboard = status.in_air = true;
    status.hover_or_command = status.controller_ready = true;
    assert(flightReady(status, true));
    assert(flightReady(status, false));
    for (bool FlightStatus::*member : {&FlightStatus::fresh_state, &FlightStatus::connected,
            &FlightStatus::armed, &FlightStatus::offboard, &FlightStatus::in_air,
            &FlightStatus::controller_fresh, &FlightStatus::hover_or_command})
    {
        status.*member = false;
        assert(!flightReady(status, true));
        assert(!flightReady(status, false));
        status.*member = true;
    }
    status.controller_ready = false;
    assert(!flightReady(status, true));
    assert(flightReady(status, false));
    assert(fresh(0.1, 0.5));
    assert(!fresh(0.6, 0.5));
    assert(!fresh(-0.1, 0.5));
    assert(!fresh(std::numeric_limits<double>::quiet_NaN(), 0.5));
}
