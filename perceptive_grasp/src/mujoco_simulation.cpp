/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_simulation.cpp
 * @brief Shared MuJoCo simulation state.
 */

#include "mujoco_simulation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17)
#include <iostream>
#include <map>
#include <mutex>
#include <random>

namespace fs = std::filesystem;

namespace perceptive_grasp {
namespace {

std::mutex g_registry_mutex;
std::map<std::string, std::weak_ptr<MujocoSimulation>> g_registry;

}  // namespace

MujocoSimulation::MujocoSimulation(std::string xml_path)
    : xml_path_(std::move(xml_path)) {}

MujocoSimulation::~MujocoSimulation() {
    if (data_ != nullptr) {
        mj_deleteData(data_);
    }
    if (model_ != nullptr) {
        mj_deleteModel(model_);
    }
}

std::shared_ptr<MujocoSimulation> MujocoSimulation::Get(
    const std::string& xml_path, std::string* error) {
    if (xml_path.empty()) {
        if (error) *error = "mujoco xml path is empty";
        return nullptr;
    }

    std::string resolved = xml_path;
    if (fs::exists(xml_path)) {
        resolved = fs::weakly_canonical(xml_path).string();
    }

    std::lock_guard<std::mutex> lock(g_registry_mutex);
    if (auto existing = g_registry[resolved].lock()) {
        return existing;
    }

    std::shared_ptr<MujocoSimulation> simulation(
        new MujocoSimulation(resolved));
    if (!simulation->Load(error)) {
        return nullptr;
    }
    g_registry[resolved] = simulation;
    return simulation;
}

bool MujocoSimulation::Load(std::string* error) {
    if (!fs::exists(xml_path_)) {
        if (error) *error = "mujoco xml does not exist: " + xml_path_;
        return false;
    }

    char load_error[1024] = {};
    model_ = mj_loadXML(xml_path_.c_str(), nullptr,
                        load_error, sizeof(load_error));
    if (model_ == nullptr) {
        if (error) *error = load_error;
        return false;
    }
    data_ = mj_makeData(model_);
    if (data_ == nullptr) {
        if (error) *error = "mj_makeData failed";
        return false;
    }
    RandomizeInitialObjectLayout();
    mj_forward(model_, data_);
    initial_qpos_.assign(data_->qpos, data_->qpos + model_->nq);
    initial_qvel_.assign(data_->qvel, data_->qvel + model_->nv);
    initial_ctrl_.assign(data_->ctrl, data_->ctrl + model_->nu);
    return true;
}

void MujocoSimulation::RandomizeInitialObjectLayout() {
    if (model_ == nullptr || data_ == nullptr ||
        mj_name2id(model_, mjOBJ_GEOM, "zone_pickup") < 0) {
        return;
    }

    struct ObjectPlacement {
        const char* body_name;
        double z;
        double half_extent_x;
        double half_extent_y;
    };
    struct CandidatePlacement {
        int qpos_addr;
        const char* body_name;
        double x;
        double y;
        double z;
        double yaw;
        double half_extent_x;
        double half_extent_y;
    };
    std::array<ObjectPlacement, 4> objects = {{
        {"Apple", 0.038, 0.040, 0.040},
        {"Banana", 0.028, 0.105, 0.030},
        {"Cup", 0.090, 0.083, 0.048},
        {"Box", 0.030, 0.032, 0.032},
    }};

    if (!layout_generator_initialized_) {
        std::random_device random_device;
        unsigned int seed = random_device();
        if (const char* configured_seed =
                std::getenv("PERCEPTIVE_GRASP_MUJOCO_LAYOUT_SEED")) {
            try {
                seed = static_cast<unsigned int>(
                    std::stoul(configured_seed));
            } catch (const std::exception&) {
                std::cerr
                    << "[MujocoSimulation] Ignoring invalid layout seed: "
                    << configured_seed << std::endl;
            }
        }
        layout_generator_.seed(seed);
        layout_generator_initialized_ = true;
    }

    const int pickup_body_id = mj_name2id(
        model_, mjOBJ_BODY, "zone_pickup");
    const int pickup_geom_id = mj_name2id(
        model_, mjOBJ_GEOM, "zone_pickup");
    if (pickup_body_id < 0 || pickup_geom_id < 0) {
        return;
    }
    const mjtNum* pickup_position =
        model_->body_pos + 3 * pickup_body_id;
    const mjtNum* pickup_size =
        model_->geom_size + 3 * pickup_geom_id;

    std::uniform_real_distribution<double> yaw_distribution(-mjPI, mjPI);
    std::uniform_real_distribution<double> x_jitter_distribution(
        -0.005, 0.005);
    constexpr double kZoneMargin = 0.010;
    constexpr double kObjectGap = 0.060;
    constexpr double kCameraVisibleHalfWidth = 0.340;
    constexpr double kRowCenterOffset = -0.080;
    constexpr double kMaximumRowOffset = 0.015;
    constexpr int kMaximumLayoutAttempts = 100;

    std::vector<CandidatePlacement> layout;
    for (int layout_attempt = 0;
            layout_attempt < kMaximumLayoutAttempts && layout.empty();
            ++layout_attempt) {
        std::shuffle(objects.begin(), objects.end(), layout_generator_);
        std::vector<CandidatePlacement> candidate_layout;
        double required_row_width =
            kObjectGap * static_cast<double>(objects.size() - 1);
        for (size_t index = 0; index < objects.size(); ++index) {
            const ObjectPlacement& object = objects[index];
            const int body_id = mj_name2id(
                model_, mjOBJ_BODY, object.body_name);
            if (body_id < 0 || model_->body_jntnum[body_id] != 1) {
                candidate_layout.clear();
                break;
            }
            const int joint_id = model_->body_jntadr[body_id];
            if (model_->jnt_type[joint_id] != mjJNT_FREE) {
                candidate_layout.clear();
                break;
            }
            const double yaw = yaw_distribution(layout_generator_);
            const double cosine = std::abs(std::cos(yaw));
            const double sine = std::abs(std::sin(yaw));
            const double projected_half_x =
                cosine * object.half_extent_x +
                sine * object.half_extent_y;
            const double projected_half_y =
                sine * object.half_extent_x +
                cosine * object.half_extent_y;
            const double x = pickup_position[0] +
                x_jitter_distribution(layout_generator_);
            const bool fits_x =
                std::abs(x - pickup_position[0]) + projected_half_x <=
                pickup_size[0] - kZoneMargin;
            if (!fits_x) {
                candidate_layout.clear();
                break;
            }
            required_row_width += 2.0 * projected_half_y;
            candidate_layout.push_back({
                model_->jnt_qposadr[joint_id], object.body_name,
                x, 0.0, object.z, yaw,
                projected_half_x, projected_half_y});
        }
        if (candidate_layout.size() != objects.size()) {
            continue;
        }
        const double row_center =
            pickup_position[1] + kRowCenterOffset;
        const double row_lower = std::max(
            pickup_position[1] - pickup_size[1] + kZoneMargin,
            row_center - kCameraVisibleHalfWidth);
        const double row_upper = std::min(
            pickup_position[1] + pickup_size[1] - kZoneMargin,
            row_center + kCameraVisibleHalfWidth);
        const double available_row_width = row_upper - row_lower;
        if (required_row_width > available_row_width) {
            continue;
        }
        const double offset_limit = std::min(
            kMaximumRowOffset,
            0.5 * (available_row_width - required_row_width));
        std::uniform_real_distribution<double> row_offset_distribution(
            -offset_limit, offset_limit);
        double cursor = row_lower +
            0.5 * (available_row_width - required_row_width) +
            row_offset_distribution(layout_generator_);
        for (CandidatePlacement& placement : candidate_layout) {
            placement.y = cursor + placement.half_extent_y;
            cursor += 2.0 * placement.half_extent_y + kObjectGap;
        }
        layout = std::move(candidate_layout);
    }

    if (layout.size() != objects.size()) {
        std::cerr << "[MujocoSimulation] Failed to generate pickup layout"
                << std::endl;
        return;
    }

    std::cout << "[MujocoSimulation] Pickup layout:";
    for (const CandidatePlacement& placement : layout) {
        data_->qpos[placement.qpos_addr] = placement.x;
        data_->qpos[placement.qpos_addr + 1] = placement.y;
        data_->qpos[placement.qpos_addr + 2] = placement.z;
        data_->qpos[placement.qpos_addr + 3] =
            std::cos(0.5 * placement.yaw);
        data_->qpos[placement.qpos_addr + 4] = 0.0;
        data_->qpos[placement.qpos_addr + 5] = 0.0;
        data_->qpos[placement.qpos_addr + 6] =
            std::sin(0.5 * placement.yaw);
        std::cout << " " << placement.body_name << "=["
                << placement.x << "," << placement.y
                << ",yaw=" << placement.yaw << "]";
    }
    std::cout << std::endl;
}

void MujocoSimulation::Reset() {
    if (model_ == nullptr || data_ == nullptr) {
        return;
    }
    mj_resetData(model_, data_);
    if (initial_qpos_.size() == static_cast<size_t>(model_->nq)) {
        std::copy(initial_qpos_.begin(), initial_qpos_.end(), data_->qpos);
    }
    if (initial_qvel_.size() == static_cast<size_t>(model_->nv)) {
        std::copy(initial_qvel_.begin(), initial_qvel_.end(), data_->qvel);
    }
    if (initial_ctrl_.size() == static_cast<size_t>(model_->nu)) {
        std::copy(initial_ctrl_.begin(), initial_ctrl_.end(), data_->ctrl);
    }
    RandomizeInitialObjectLayout();
    mj_forward(model_, data_);
}

}  // namespace perceptive_grasp
