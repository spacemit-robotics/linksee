/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @brief Scene and RGB-D regressions against the executor's public runtime.
 */
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <opencv2/core.hpp>
#include <mujoco/mujoco.h>
#include "manipulation/mujoco_runtime.h"
#include "manipulation_mujoco_camera.h"
#include "mujoco_grasp_executor.h"
#include "mujoco_native_snapshot.h"
#include "mujoco_test_config.h"
#include "mujoco_world.h"
#include "stereo_camera.h"

namespace {
void Require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}
std::array<double, 3> Position(const mujoco_sim::NativeSnapshot& snapshot, const char* name) {
    const int id = mj_name2id(snapshot.Model(), mjOBJ_BODY, name);
    Require(id >= 0, "scene body missing");
    const auto* xyz = snapshot.Data()->xpos + id * 3;
    return {xyz[0], xyz[1], xyz[2]};
}
void CheckScene(const mujoco_sim::NativeSnapshot& snapshot) {
    const auto pickup = Position(snapshot, "zone_pickup");
    const auto drop = Position(snapshot, "zone_drop");
    const auto* model = snapshot.Model();
    const auto* pickup_size = model->geom_size + 3 * mj_name2id(model, mjOBJ_GEOM, "zone_pickup");
    const auto* drop_size = model->geom_size + 3 * mj_name2id(model, mjOBJ_GEOM, "zone_drop");
    Require(pickup_size[0] >= 0.15 && drop_size[0] >= 0.15 && drop[1] < 0 &&
        drop[1] + drop_size[1] < pickup[1] - pickup_size[1], "invalid separated work zones");
    const std::array<const char*, 4> names{"Apple", "Banana", "Cup", "Box"};
    std::array<std::array<double, 3>, 4> positions{};
    const std::array<double, 4> half_y{0.040, 0.030, 0.048, 0.032};
    for (std::size_t i = 0; i < names.size(); ++i) {
        positions[i] = Position(snapshot, names[i]);
        Require(std::abs(positions[i][0] - pickup[0]) < pickup_size[0] &&
            std::abs(positions[i][1] - (pickup[1] - 0.08)) + half_y[i] <= 0.34,
            "object outside camera-visible pickup row");
        for (std::size_t j = 0; j < i; ++j) {
            Require(std::abs(positions[i][1] - positions[j][1]) >= half_y[i] + half_y[j] + 0.025,
                "pickup objects overlap");
            Require(std::abs(positions[i][0] - positions[j][0]) <= 0.012,
                "pickup objects are not in one row");
        }
    }
}
}  // namespace

int main() {
    try {
        auto config = perceptive_grasp::test::Configuration();
        config.observe_joints[0] += 0.12f;
        perceptive_grasp::MujocoGraspExecutor executor(config);
        Require(executor.Init(), "executor init failed");
        const auto runtime = executor.SimulationRuntime();
        auto first = runtime->Snapshot();
        mujoco_sim::NativeSnapshot snapshot(first);
        CheckScene(snapshot);
        const auto initial_cup = Position(snapshot, "Cup");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        snapshot.Update(runtime->Snapshot());
        const auto settled_cup = Position(snapshot, "Cup");
        Require(std::hypot(initial_cup[0] - settled_cup[0], initial_cup[1] - settled_cup[1]) < 0.005 &&
            std::abs(initial_cup[2] - settled_cup[2]) < 0.005, "cup unstable at rest");
        perceptive_grasp::StereoCameraConfig camera_config;
        camera_config.type = "mujoco";
        camera_config.mujoco.camera_name = "realsense";
        auto camera = perceptive_grasp::CreateManipulationMujocoCamera(runtime, camera_config);
        Require(camera->Init(), "shared RGB-D camera init failed");
        cv::Mat before, before_depth, after, after_depth;
        Require(camera->GetFrames(before, before_depth), "before frame failed");
        const auto before_sample = camera->LastFrameId();
        Require(executor.MoveToObserve() == perceptive_grasp::GraspResult::SUCCESS, "public arm move failed");
        const auto low = runtime->Snapshot().SampleId();
        Require(camera->GetFrames(after, after_depth), "after frame failed");
        const auto high = runtime->Snapshot().SampleId();
        const auto sample = static_cast<std::uint64_t>(camera->LastFrameId());
        Require(camera->LastFrameId() > before_sample && sample >= low && sample <= high,
            "camera did not sample executor world");
        snapshot.Update(runtime->Snapshot());
        const int joint = mj_name2id(snapshot.Model(), mjOBJ_JOINT, "robot_shoulder_pan_joint");
        Require(std::abs(snapshot.Data()->qpos[snapshot.Model()->jnt_qposadr[joint]] -
            config.observe_joints[0]) < 0.02, "shared snapshot did not observe device motion");
        Require(cv::norm(before, after, cv::NORM_L1) > 100, "arm motion absent from rendered RGB");
        Require(before_depth.type() == CV_16UC1 && cv::countNonZero(before_depth) > 0,
            "RGB-D depth is invalid");
        executor.SetTargetLabel("cup");
        snapshot.Update(runtime->Snapshot());
        const auto selected_cup = Position(snapshot, "Cup");
        Require(std::hypot(selected_cup[0] - settled_cup[0], selected_cup[1] - settled_cup[1]) < 0.005,
            "target selection reset scene objects");
        executor.ResetScene();
        snapshot.Update(runtime->Snapshot());
        const auto reset_cup = Position(snapshot, "Cup");
        Require(std::hypot(initial_cup[0] - reset_cup[0], initial_cup[1] - reset_cup[1]) < 0.005,
            "reset did not restore device-plan keyframe");
        CheckScene(snapshot);
        std::cout << "shared scene, device motion and RGB-D passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
