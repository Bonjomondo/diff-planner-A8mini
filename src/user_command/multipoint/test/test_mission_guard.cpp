#include "../src/mission_guard.h"
#include "../src/rc_reset_gesture.h"
#include <cassert>
#include <limits>
#include <initializer_list>

struct GestureDriver
{
    mission_guard::RcResetGesture gesture;
    double now = 100.0;
    int events = 0;
    void hold(int pwm, double seconds, bool eligible = true)
    {
        for (int sample = 0; sample < static_cast<int>(std::ceil(seconds / 0.05)); ++sample)
        {
            now += 0.05;
            if (gesture.update(now, pwm, eligible)) ++events;
        }
    }
};

static void testRcResetGesture()
{
    // Either physical switch direction works, including transit through a middle detent.
    for (int home : {999, 1999})
    {
        const int other = 2998 - home;
        GestureDriver driver;
        driver.hold(home, 3.0);
        assert(driver.events == 0);  // Boot position/holding an endpoint cannot reset.
        driver.hold(other, 0.3);
        driver.hold(home, 0.4);
        assert(driver.events == 0);  // A quick round trip cannot reset.
        driver.hold(1499, 0.3);
        driver.hold(other, 2.3);
        assert(driver.events == 0);  // Full hold still needs the return movement.
        driver.hold(1499, 0.3);
        driver.hold(home, 0.1);
        assert(driver.events == 0);  // Return must settle, too.
        driver.hold(home, 0.3);
        assert(driver.events == 1);
        driver.hold(home, 4.0);
        assert(driver.events == 1);  // Exactly one event, even if the caller stays eligible.
    }
    for (int interruption : {0, 1, 2, 3, 4})
    {
        GestureDriver driver;
        driver.hold(999, 0.4);
        driver.hold(1999, 1.3);
        if (interruption == 0) driver.hold(1999, 0.1, false);  // Airborne/armed/RC8/FCU guard.
        if (interruption == 1) driver.now += 0.6;  // RC stream gap.
        if (interruption == 2) driver.now -= 1.0;  // Clock discontinuity.
        if (interruption == 3) driver.hold(0, 0.1);  // Missing/unassigned/invalid PWM.
        if (interruption == 4) driver.gesture.cancel();  // Unsafe FCU callback between RC samples.
        driver.hold(1999, 1.3);
        driver.hold(999, 0.4);
        assert(driver.events == 0);  // No completion using time accumulated before interruption.
    }
    GestureDriver middle;
    middle.hold(999, 0.4);
    middle.hold(1499, 3.0);
    middle.hold(999, 0.4);
    assert(middle.events == 0);  // Middle position is not the opposite endpoint.
    middle.hold(1999, 1.3);
    middle.hold(1499, 0.1);
    middle.hold(1999, 1.3);
    middle.hold(999, 0.4);
    assert(middle.events == 0);  // Opposite hold must be continuous.
    GestureDriver timeout;
    timeout.hold(999, 0.4);
    timeout.hold(1999, 12.0);
    timeout.hold(999, 0.4);
    assert(timeout.events == 0);  // Returning long after the gesture expired cannot reset.
    GestureDriver unsafe;
    unsafe.hold(999, 0.4, false);
    unsafe.hold(1999, 2.5, false);
    unsafe.hold(999, 0.4);
    assert(unsafe.events == 0);  // A gesture in flight is never deferred until landing.
}

int main()
{
    testRcResetGesture();
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

    // 20261009_174026: stop at 1791538910.685905022, newest odom stamp
    // 1791538910.687395334. Strict negative-age rejection aborted a valid mission.
    const double recorded_age = -0.001490312;
    assert(!fresh(recorded_age, 0.5));
    assert(stampedFresh(recorded_age, 0.5, 0.02));
    assert(stampedFresh(-0.004368782, 0.5, 0.02));
    assert(stampedFresh(-0.02, 0.5, 0.02));
    assert(stampedFresh(0.5, 0.5, 0.02));
    assert(!stampedFresh(-0.020001, 0.5, 0.02));
    assert(!stampedFresh(0.500001, 0.5, 0.02));
    assert(!stampedFresh(-1.0, 0.5, 0.02));
    assert(!stampedFresh(recorded_age, 0.5, 0.0));
    assert(!stampedFresh(0.0, 0.5, -0.02));
    assert(!stampedFresh(std::numeric_limits<double>::quiet_NaN(), 0.5, 0.02));
    assert(!stampedFresh(std::numeric_limits<double>::infinity(), 0.5, 0.02));
    assert(!stampedFresh(0.0, std::numeric_limits<double>::infinity(), 0.02));
    assert(!stampedFresh(0.0, 0.5, std::numeric_limits<double>::quiet_NaN()));
}
