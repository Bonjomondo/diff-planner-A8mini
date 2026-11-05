#include "../src/mission_guard.h"
#include "../../../diff_planner/plan_manage/src/traj_guard.h"
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

    // 地图/围栏校验去抖：连续失效计时（降落阶段的豁免由调用方决定）
    double since = 0.0;
    assert(!sustained(100.0, true, since, 1.0));
    assert(since == 100.0);
    assert(!sustained(100.9, true, since, 1.0));
    assert(sustained(101.0, true, since, 1.0));
    assert(!sustained(101.1, false, since, 1.0));
    assert(since == 0.0);
    assert(!sustained(102.0, true, since, 1.0));   // 恢复后重新计时
    assert(since == 102.0);
    assert(sustained(102.0, true, since, 0.0));    // 零容忍立即生效
    assert(!sustained(0.0, true, since, std::numeric_limits<double>::quiet_NaN()));

    // 地面受控复位：只有未解锁 + 在地面 + 无轨迹跟踪时才允许
    assert(traj_guard::resumeAllowed(false, 1, false));
    assert(!traj_guard::resumeAllowed(true, 1, false));    // 空中（已解锁）
    assert(!traj_guard::resumeAllowed(false, 2, false));   // 非 ON_GROUND
    assert(!traj_guard::resumeAllowed(false, 0, false));   // 状态未知
    assert(!traj_guard::resumeAllowed(false, 1, true));    // 仍在跟踪轨迹
}
