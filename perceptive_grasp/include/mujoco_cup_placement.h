/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file mujoco_cup_placement.h
 * @brief Private rigid-transform helper for upright simulated cup placement.
 */
#ifndef MUJOCO_CUP_PLACEMENT_H
#define MUJOCO_CUP_PLACEMENT_H

#include <array>
#include <cmath>

#include <mujoco/mujoco.h>

namespace perceptive_grasp {

inline constexpr double kMujocoCupReleaseGap = 0.0025;

/** Bounded positive tabletop clearance avoids both penetration and a handle-catching drop. */
inline bool MujocoCupReleaseReady(double gap, double tilt, bool inside_zone) {
    return inside_zone && std::isfinite(gap) && std::isfinite(tilt) &&
        gap >= 0.002 && gap <= 0.003 && tilt >= 0 && tilt <= 0.025;
}

inline double MujocoQuaternionDistance(const mjtNum* first, const mjtNum* second) {
    mjtNum inverse[4], delta[4];
    mju_negQuat(inverse, first);
    mju_mulQuat(delta, second, inverse);
    return 2 * std::atan2(std::sqrt(delta[1] * delta[1] + delta[2] * delta[2] + delta[3] * delta[3]),
        std::abs(delta[0]));
}

/** Keep the object centre/yaw, make its local +Z vertical, and recover the hold pose. */
inline std::array<mjtNum, 7> MujocoUprightHoldPose(const mjtNum* object_position,
    const mjtNum* object_quaternion, const mjtNum* relative_position,
    const mjtNum* relative_quaternion) {
    mjtNum rotation[9], inverse_relative[4], hold_quaternion[4], offset[3];
    mju_quat2Mat(rotation, object_quaternion);
    const double yaw = std::atan2(rotation[3], rotation[0]);
    const mjtNum upright_object[4] = {std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)};
    mju_negQuat(inverse_relative, relative_quaternion);
    mju_mulQuat(hold_quaternion, upright_object, inverse_relative);
    mju_rotVecQuat(offset, relative_position, hold_quaternion);
    return {object_position[0] - offset[0], object_position[1] - offset[1],
        object_position[2] - offset[2], hold_quaternion[0], hold_quaternion[1],
        hold_quaternion[2], hold_quaternion[3]};
}

}  // namespace perceptive_grasp

#endif  // MUJOCO_CUP_PLACEMENT_H
