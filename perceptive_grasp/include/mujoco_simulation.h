/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_simulation.h
 * @brief Shared MuJoCo simulation state for camera and executor backends.
 */

#ifndef MUJOCO_SIMULATION_H
#define MUJOCO_SIMULATION_H

#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace perceptive_grasp {

class MujocoSimulation {
public:
    ~MujocoSimulation();

    static std::shared_ptr<MujocoSimulation> Get(
        const std::string& xml_path, std::string* error);

    mjModel* model() const { return model_; }
    mjData* data() const { return data_; }
    const std::string& xml_path() const { return xml_path_; }
    std::mutex& mutex() { return mutex_; }
    void Reset();

private:
    explicit MujocoSimulation(std::string xml_path);

    bool Load(std::string* error);
    void RandomizeInitialObjectLayout();

    std::string xml_path_;
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    std::mutex mutex_;
    std::vector<mjtNum> initial_qpos_;
    std::vector<mjtNum> initial_qvel_;
    std::vector<mjtNum> initial_ctrl_;
    std::mt19937 layout_generator_;
    bool layout_generator_initialized_ = false;
};

}  // namespace perceptive_grasp

#endif  // MUJOCO_SIMULATION_H
