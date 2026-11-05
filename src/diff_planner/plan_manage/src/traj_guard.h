#pragma once

namespace traj_guard
{
// 轨迹输出闩锁的复位条件：必须在地面、未解锁、且当前没有在跟踪的轨迹。
// 空中或解锁状态下一律拒绝 —— abort 之后不能再跟随一条已经不可信的轨迹。
// landed_state 用 mavros_msgs/ExtendedState 的原始数值，1 = ON_GROUND。
inline bool resumeAllowed(bool armed, int landed_state, bool tracking)
{
    constexpr int kLandedStateOnGround = 1;
    return !armed && landed_state == kLandedStateOnGround && !tracking;
}
}  // namespace traj_guard
