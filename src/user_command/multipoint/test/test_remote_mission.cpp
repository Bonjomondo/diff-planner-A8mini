#include "../src/remote_mission.h"
#include <cassert>
#include <iostream>

template <typename F> void rejects(F action)
{
    bool rejected = false;
    try { action(); }
    catch (const std::runtime_error &) { rejected = true; }
    assert(rejected);
}

int main()
{
    using namespace remote_mission;
    Mission mission;
    mission.session = "map-1";
    mission.validateLimits();
    const std::vector<Point> route{{1, 1, 0, 1, 2}, {2, 2, 0, 1, 0}, {3, 3, 0, 1, 0}};
    rejects([&] { mission.upload("old-map", "route", 1, route); });
    rejects([&] { mission.upload("map-1", "route", 2, route); });
    rejects([&] { mission.upload("map-1", "route", 1, {}); });
    rejects([&] { mission.upload("map-1", "route", 1, {{1, 0, 0, 120, 0}}); });
    rejects([&] { mission.upload("map-1", "route", 1, {{1, 0, 0, 1, 0}, {1, 1, 0, 1, 0}}); });
    rejects([&] { mission.upload("map-1", "route", 1, {{1, NAN, 0, 1, 0}}); });
    mission.upload("map-1", "route", 1, route);
    assert(mission.state == "READY" && mission.completed == 0);
    rejects([&] { mission.checkStart("map-1", "route", 2); });
    rejects([&] { mission.checkStart("map-1", "other", 1); });
    mission.checkStart("map-1", "route", 1);
    mission.state = "FLYING";
    rejects([&] { mission.upload("map-1", "other", 1, route); });
    rejects([&] { mission.checkStart("map-1", "route", 1); });
    rejects([&] { mission.update("map-1", "route", 3, 1, {{4, 4, 0, 1, 0}}); });
    rejects([&] { mission.update("map-1", "route", 2, 4, {}); });
    rejects([&] { mission.update("map-1", "route", 2, 1, {{1, 4, 0, 1, 0}}); });
    assert(mission.revision == 1 && mission.points.size() == 3 && mission.points[1].id == 2);
    mission.update("map-1", "route", 2, 1, {{4, 4, 0, 1, 0}});
    assert(mission.points.size() == 2 && mission.points[0].id == 1 && mission.points[1].id == 4);
    // Replacing a flying target, appending, and truncating only future work.
    mission.update("map-1", "route", 3, 0, {{5, 5, 0, 1, 0}});
    mission.update("map-1", "route", 4, 1, {{6, 6, 0, 1, 0}});
    mission.completed = 1;
    rejects([&] { mission.update("map-1", "route", 5, 0, {{7, 7, 0, 1, 0}}); });
    rejects([&] { mission.update("map-1", "route", 5, 1, {}); });
    mission.state = "HOVERING";
    rejects([&] { mission.update("map-1", "route", 5, 1, {{7, 7, 0, 1, 0}}); });
    mission.update("map-1", "route", 5, 2, {{7, 7, 0, 1, 0}});
    assert(mission.completed == 1 && mission.points[1].id == 6);
    rejects([&] { mission.update("map-1", "route", 6, 3, std::vector<Point>(200, {9, 0, 0, 1, 0})); });
    assert(mission.revision == 5 && mission.points.size() == 3);
    mission.invalidate("map-2", "odom reset");
    assert(mission.state == "FAILED" && mission.session == "map-2");
    rejects([&] { mission.upload("map-2", "other", 1, route); });
    Mission ready;
    ready.session = "map-1";
    ready.upload("map-1", "route", 1, route);
    ready.invalidate("map-2", "gap");
    assert(ready.state == "IDLE" && ready.points.empty() && ready.revision == 0);
    rejects([&] { ready.checkStart("map-1", "route", 1); });
    ready.upload("map-2", "new-route", 1, route);
    ready.state = "LANDING";
    rejects([&] { ready.update("map-2", "new-route", 2, 0, route); });
    ready.max_z = ready.min_z;
    rejects([&] { ready.validateLimits(); });
    Mission history;
    history.session = "map";
    history.upload("map", "one", 1, route);
    history.state = "COMPLETED";
    history.upload("map", "two", 1, route);
    history.state = "COMPLETED";
    rejects([&] { history.upload("map", "one", 1, route); });
    Mission continuation;
    continuation.session = "map";
    continuation.upload("map", "flight", 1, route);
    continuation.state = "COMPLETED";
    continuation.completed = route.size();
    rejects([&] { continuation.update("map", "flight", 2, 2, {{4, 4, 0, 1, 0}}); });
    rejects([&] { continuation.update("map", "flight", 2, 3, {}); });
    continuation.update("map", "flight", 2, 3, {{4, 4, 0, 1, 0}});
    assert(continuation.state == "READY" && continuation.completed == 3 && continuation.points.size() == 4);
    continuation.checkStart("map", "flight", 2);
    rejects([&] { continuation.checkStart("map", "flight", 1); });
    rejects([&] { continuation.update("map", "flight", 3, 2, {{5, 5, 0, 1, 0}}); });
    continuation.prepareReturn("return", {1, .3, -.2, 1.2, 0});
    assert(continuation.state == "FLYING" && continuation.completed == 0 && continuation.points.size() == 1);
    rejects([&] { continuation.checkStart("map", "flight", 2); });
    rejects([&] { continuation.prepareReturn("return", {1, 0, 0, 1, 0}); });
    continuation.state = "LANDING";
    rejects([&] { continuation.prepareReturn("bad", {1, 0, 0, 1, 0}); });
    continuation.state = "FAILED";
    rejects([&] { continuation.prepareReturn("bad", {1, 0, 0, 1, 0}); });
    std::vector<Point> maximum;
    for (uint32_t i = 1; i <= 200; ++i) maximum.push_back({i, 0, 0, 1, 60});
    history.upload("map", "maximum", 1, maximum);
    assert(history.points.size() == 200);
    std::cout << "Remote mission checks passed: atomic routes, bounds, versions, frozen prefix and sessions\n";
}
