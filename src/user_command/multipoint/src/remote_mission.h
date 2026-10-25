#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace remote_mission
{
struct Point
{
    uint32_t id;
    double x, y, z, hover_sec;
};

// Independent of ROS so route transactions can be tested without a flight stack.
class Mission
{
public:
    std::string session, id, state = "IDLE", reason;
    uint32_t revision = 0;
    std::size_t completed = 0;
    std::vector<Point> points;
    double max_abs_xy = 30.0, min_z = 0.3, max_z = 2.5;

    bool active() const { return state == "FLYING" || state == "HOVERING"; }

    void validateLimits() const
    {
        if (!std::isfinite(max_abs_xy) || max_abs_xy <= 0 ||
            !std::isfinite(min_z) || min_z < 0 ||
            !std::isfinite(max_z) || max_z <= min_z)
            throw std::runtime_error("Invalid remote mission coordinate limits");
    }

    void validate(const std::vector<Point> &route) const
    {
        if (route.empty() || route.size() > 200)
            throw std::runtime_error("A mission must contain 1..200 waypoints");
        std::set<uint32_t> ids;
        for (const auto &p : route)
        {
            if (!p.id || !ids.insert(p.id).second)
                throw std::runtime_error("Waypoint IDs must be unique positive UInt32 values");
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                !std::isfinite(p.hover_sec) || std::abs(p.x) > max_abs_xy ||
                std::abs(p.y) > max_abs_xy || p.z < min_z || p.z > max_z ||
                p.hover_sec < 0 || p.hover_sec > 60)
                throw std::runtime_error("Waypoint outside configured world bounds or hover_sec [0,60]");
        }
    }

    void requireIdentity(const std::string &map, const std::string &mission) const
    {
        if (map != session) throw std::runtime_error("Map session changed; review and upload again");
        if (mission.empty() || mission != id) throw std::runtime_error("Mission ID does not match");
    }

    void upload(const std::string &map, const std::string &mission, uint32_t rev,
                const std::vector<Point> &route)
    {
        if (map != session) throw std::runtime_error("Map session changed; review and upload again");
        if (state != "IDLE" && state != "READY" && state != "COMPLETED")
            throw std::runtime_error("Upload requires an idle mission; use update during execution");
        if (mission.empty() || mission.size() > 80 || mission_ids_.count(mission) || rev != 1)
            throw std::runtime_error("Upload requires a new mission ID and revision 1");
        validate(route);
        points = route;
        mission_ids_.insert(mission);
        id = mission;
        revision = rev;
        completed = 0;
        state = "READY";
        reason.clear();
    }

    void checkStart(const std::string &map, const std::string &mission, uint32_t rev) const
    {
        requireIdentity(map, mission);
        if (rev != revision || state != "READY" || completed >= points.size())
            throw std::runtime_error("Start requires READY and the exact uploaded revision");
    }

    void update(const std::string &map, const std::string &mission, uint32_t rev,
                std::size_t from, const std::vector<Point> &suffix)
    {
        requireIdentity(map, mission);
        if (state != "READY" && state != "COMPLETED" && !active())
            throw std::runtime_error("Update requires READY, COMPLETED, FLYING or HOVERING");
        if (revision == std::numeric_limits<uint32_t>::max() || rev != revision + 1)
            throw std::runtime_error("Update revision must be current revision + 1");
        if (from > points.size() || from < completed ||
            (state == "HOVERING" && from == completed))
            throw std::runtime_error("Cannot replace completed waypoints or the current hover action");
        std::vector<Point> candidate(points.begin(), points.begin() + from);
        candidate.insert(candidate.end(), suffix.begin(), suffix.end());
        validate(candidate);
        if ((active() || completed > 0) && candidate.size() <= completed)
            throw std::runtime_error("Update must retain an uncompleted waypoint");
        points.swap(candidate);
        revision = rev;
        if (state == "COMPLETED") state = "READY";
        reason = completed > 0 && state == "READY" ? "Appended route ready; explicit start resumes unfinished waypoints" : "";
    }

    void prepareReturn(const std::string &mission, const Point &home)
    {
        if (state != "IDLE" && state != "READY" && state != "COMPLETED" && !active())
            throw std::runtime_error("Cannot return from a failed or landing mission");
        if (mission.empty() || mission.size() > 80 || mission_ids_.count(mission))
            throw std::runtime_error("Return requires a new mission ID");
        validate({home});
        points = {home};
        mission_ids_.insert(mission);
        id = mission;
        revision = 1;
        completed = 0;
        state = "FLYING";
        reason = "Returning to takeoff position; hover on arrival";
    }

    void invalidate(const std::string &new_session, const std::string &why)
    {
        session = new_session;
        mission_ids_.clear();
        if (active() || state == "FAILED" || state == "LANDING")
            state = state == "LANDING" ? "LANDING" : "FAILED";
        else
        {
            state = "IDLE";
            id.clear();
            revision = 0;
            completed = 0;
            points.clear();
        }
        reason = why;
    }

private:
    std::set<std::string> mission_ids_;
};
}  // namespace remote_mission
