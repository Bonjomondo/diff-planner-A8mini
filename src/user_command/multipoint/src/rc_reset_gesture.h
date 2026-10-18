#pragma once

#include <cmath>

namespace mission_guard
{
// Wall-clock state machine, independent of ROS. Recognize either endpoint as home:
// stable home -> opposite endpoint for 2 s -> stable home. Missing/unsafe samples
// discard the entire gesture, so a gesture started in flight cannot complete later.
class RcResetGesture
{
public:
    void cancel()
    {
        phase_ = BASELINE;
        candidate_ = home_ = 0;
        candidate_since_ = opposite_since_ = return_since_ = last_sample_ = -1.0;
        qualified_ = false;
    }

    bool update(double now, int pwm, bool eligible)
    {
        if (!eligible || !std::isfinite(now) || pwm < 900 || pwm > 2100)
        {
            cancel();
            return false;
        }
        if (last_sample_ >= 0.0 && (now < last_sample_ || now - last_sample_ > 0.5))
            cancel();
        last_sample_ = now;
        const int endpoint = pwm <= 1100 ? -1 : (pwm >= 1900 ? 1 : 0);
        if (phase_ == BASELINE)
        {
            if (endpoint == 0 || endpoint != candidate_)
            {
                candidate_ = endpoint;
                candidate_since_ = now;
            }
            else if (now - candidate_since_ >= 0.2)
            {
                home_ = endpoint;
                phase_ = READY;
            }
            return false;
        }
        if (phase_ == READY)
        {
            if (endpoint == home_) return false;
            phase_ = EXCURSION;
            excursion_since_ = now;
            opposite_since_ = return_since_ = -1.0;
            qualified_ = false;
        }
        if (now - excursion_since_ > 10.0)
        {
            cancel();
            return false;
        }
        if (endpoint == -home_)
        {
            return_since_ = -1.0;
            if (opposite_since_ < 0.0) opposite_since_ = now;
            if (now - opposite_since_ >= 2.0) qualified_ = true;
        }
        else if (endpoint == home_)
        {
            opposite_since_ = -1.0;
            if (!qualified_)
                phase_ = READY;
            else
            {
                if (return_since_ < 0.0) return_since_ = now;
                if (now - return_since_ >= 0.2)
                {
                    cancel();
                    return true;
                }
            }
        }
        else
        {
            opposite_since_ = return_since_ = -1.0;
        }
        return false;
    }

private:
    enum Phase { BASELINE, READY, EXCURSION } phase_ = BASELINE;
    int candidate_ = 0, home_ = 0;
    double candidate_since_ = -1.0, opposite_since_ = -1.0, return_since_ = -1.0;
    double last_sample_ = -1.0, excursion_since_ = 0.0;
    bool qualified_ = false;
};
}  // namespace mission_guard
