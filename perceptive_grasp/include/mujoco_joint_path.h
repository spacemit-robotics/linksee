/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file mujoco_joint_path.h
 * @brief Application-local sampled joint-path precheck with planning-state restoration.
 */
#ifndef MUJOCO_JOINT_PATH_H
#define MUJOCO_JOINT_PATH_H

#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace perceptive_grasp {

inline bool CheckMujocoJointPath(const std::vector<float>& start,
    const std::vector<float>& target, const std::function<void(const std::vector<float>&)>& apply,
    const std::function<bool(std::string*)>& collides, const std::function<void()>& restore,
    std::string* detail) {
    if (start.size() != target.size()) throw std::invalid_argument("joint path size mismatch");
    bool clear = true;
    try {
        constexpr int kSamples = 24;
        for (int sample = 0; sample <= kSamples; ++sample) {
            const float alpha = static_cast<float>(sample) / kSamples;
            std::vector<float> point(start.size());
            for (std::size_t joint = 0; joint < point.size(); ++joint) {
                point[joint] = start[joint] + alpha * (target[joint] - start[joint]);
            }
            apply(point);
            if (collides(detail)) {
                if (detail) *detail = "sample=" + std::to_string(sample) + "/" +
                    std::to_string(kSamples) + " " + *detail;
                clear = false;
                break;
            }
        }
    } catch (...) {
        restore();
        throw;
    }
    restore();
    return clear;
}

}  // namespace perceptive_grasp
#endif  // MUJOCO_JOINT_PATH_H
