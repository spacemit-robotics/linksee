/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_grasp_executor_test.cpp
 * @brief Tests the optional MuJoCo executor backend contract.
 */

#include "mujoco_grasp_executor.h"
#include "mujoco_simulation.h"

#include <array>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <unistd.h>

using perceptive_grasp::ExecutorConfig;
using perceptive_grasp::GraspResult;
using perceptive_grasp::MujocoGraspExecutor;
using perceptive_grasp::MujocoSimulation;
using perceptive_grasp::Pose3D;

namespace {

std::string WriteTestMjcf() {
    const std::string path =
        "/tmp/perceptive_grasp_mujoco_executor_" +
        std::to_string(static_cast<long long>(getpid())) + ".xml";
    std::ofstream output(path);
    output << R"(<mujoco model="executor_test">
<option timestep="0.002"/>
<worldbody>
    <body name="base" pos="0 0 0">
        <joint name="shoulder_pan_joint" type="hinge"
            axis="0 0 1" range="-3.14 3.14"
            damping="0.5" armature="0.01"/>
        <geom type="box" size="0.02 0.02 0.02" mass="0.1"/>
        <body pos="0.12 0 0">
            <joint name="shoulder_lift_joint" type="hinge"
                axis="0 1 0" range="-3.14 3.14"
                damping="0.5" armature="0.01"/>
            <geom type="box" size="0.02 0.02 0.02" mass="0.1"/>
            <body pos="0.12 0 0">
                <joint name="elbow_joint" type="hinge"
                    axis="0 1 0" range="-3.14 3.14"
                    damping="0.5" armature="0.01"/>
                <geom type="box" size="0.02 0.02 0.02" mass="0.1"/>
                <body pos="0.12 0 0">
                    <joint name="wrist_1_joint" type="hinge"
                        axis="1 0 0" range="-3.14 3.14"
                        damping="0.5" armature="0.01"/>
                    <geom type="box" size="0.02 0.02 0.02" mass="0.1"/>
                    <body pos="0.08 0 0">
                        <joint name="wrist_2_joint" type="hinge"
                            axis="0 1 0" range="-3.14 3.14"
                            damping="0.5" armature="0.01"/>
                        <geom type="box" size="0.02 0.02 0.02" mass="0.1"/>
                        <body pos="0.08 0 0">
                            <joint name="wrist_3_joint" type="hinge"
                                axis="1 0 0" range="-3.14 3.14"
                                damping="0.5" armature="0.01"/>
                            <geom type="box" size="0.02 0.02 0.02"
                                mass="0.1"/>
                            <site name="pinch" pos="0.08 0 0"
                                size="0.005"/>
                            <body name="finger" pos="0.05 0 0">
                                <joint name="finger_joint" type="slide"
                                    axis="0 1 0" range="0 1"
                                    damping="0.5" armature="0.01"/>
                                <geom type="box" size="0.01 0.01 0.01"/>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </body>
</worldbody>
<actuator>
    <position name="shoulder_pan" joint="shoulder_pan_joint" kp="10"/>
    <position name="shoulder_lift" joint="shoulder_lift_joint" kp="10"/>
    <position name="elbow" joint="elbow_joint" kp="10"/>
    <position name="wrist_1" joint="wrist_1_joint" kp="10"/>
    <position name="wrist_2" joint="wrist_2_joint" kp="10"/>
    <position name="wrist_3" joint="wrist_3_joint" kp="10"/>
    <position name="fingers_actuator" joint="finger_joint" kp="10"/>
</actuator>
</mujoco>)";
    return path;
}

ExecutorConfig MakeConfig(const std::string& xml_path) {
    ExecutorConfig config;
    config.manip_driver = "mujoco_ur5e";
    config.mujoco.xml_path = xml_path;
    config.mujoco.robot_root_body = "base";
    config.mujoco.gripper_root_body = "finger";
    config.home_joints = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    config.observe_joints = config.home_joints;
    config.side_ready_joints = config.home_joints;
    config.place_joints = config.home_joints;
    config.mujoco.joint_tolerance_rad = 0.2f;
    config.mujoco.max_motion_steps = 2000;
    config.mujoco.settle_steps = 20;
    return config;
}

bool ExpectSuccess(const char* action, GraspResult result,
        const MujocoGraspExecutor* executor = nullptr) {
    if (result == GraspResult::SUCCESS) {
        return true;
    }
    std::cerr << action << " failed with result="
            << static_cast<int>(result) << std::endl;
    if (executor != nullptr) {
        const auto diagnostics = executor->GetDiagnostics();
        std::cerr << "  action=" << diagnostics.last_action
                << " detail=" << diagnostics.last_detail << std::endl;
    }
    return false;
}

ExecutorConfig MakeReleaseSceneConfig() {
    ExecutorConfig config;
    config.manip_driver = "mujoco_ur5e";
    config.mujoco.xml_path =
        std::string(PERCEPTIVE_GRASP_SOURCE_DIR) +
        "/simulation/mujoco/ur5e_scene.xml";
    config.mujoco.end_effector_site = "robot_gripper_pinch";
    config.mujoco.gripper_actuator = "robot_gripper_fingers_actuator";
    config.mujoco.robot_root_body = "robot_base";
    config.mujoco.gripper_root_body = "robot_gripper_base_mount";
    config.mujoco.gripper_open_ctrl = 0.0f;
    config.mujoco.gripper_close_ctrl = 255.0f;
    config.mujoco.arm_stiffness_scale = 12.0f;
    config.mujoco.joint_names = {
        "robot_shoulder_pan_joint",
        "robot_shoulder_lift_joint",
        "robot_elbow_joint",
        "robot_wrist_1_joint",
        "robot_wrist_2_joint",
        "robot_wrist_3_joint",
    };
    config.mujoco.actuator_names = {
        "robot_shoulder_pan",
        "robot_shoulder_lift",
        "robot_elbow",
        "robot_wrist_1",
        "robot_wrist_2",
        "robot_wrist_3",
    };
    config.mujoco.joint_tolerance_rad = 0.04f;
    config.mujoco.ik_position_tolerance_m = 0.006f;
    config.mujoco.cartesian_tracking_tolerance_m = 0.003f;
    config.mujoco.ik_step_scale = 0.80f;
    config.mujoco.ik_damping = 0.010f;
    config.mujoco.ik_iterations = 300;
    config.mujoco.settle_steps = 500;
    config.mujoco.max_motion_steps = 4000;
    config.home_joints = {
        -1.5708f, -1.5708f, 1.5708f, -1.5708f, -1.5708f, 0.0f};
    config.observe_joints = config.home_joints;
    config.side_ready_joints = {
        0.20f, -0.90f, 1.30f, -1.80f, -1.57f, 0.0f};
    config.place_joints = {-1.20f, -1.00f, 1.40f, -1.90f, -1.57f, 0.0f};
    config.move_speed = 0.6f;
    config.line_speed = 0.5f;
    return config;
}

bool TargetInsideDropZone(const ExecutorConfig& config,
        const char* target_name) {
    std::string error;
    const std::shared_ptr<MujocoSimulation> simulation =
        MujocoSimulation::Get(config.mujoco.xml_path, &error);
    if (!simulation) {
        std::cerr << "failed to inspect release scene: " << error << std::endl;
        return false;
    }
    mjModel* model = simulation->model();
    mjData* data = simulation->data();
    const int target_id = mj_name2id(model, mjOBJ_BODY, target_name);
    const int zone_id = mj_name2id(model, mjOBJ_BODY, "zone_drop");
    const int zone_geom_id = mj_name2id(model, mjOBJ_GEOM, "zone_drop");
    if (target_id < 0 || zone_id < 0 || zone_geom_id < 0) return false;
    mj_forward(model, data);
    const mjtNum* target = data->xpos + 3 * target_id;
    const mjtNum* zone = data->xpos + 3 * zone_id;
    const mjtNum* zone_size = model->geom_size + 3 * zone_geom_id;
    return std::abs(target[0] - zone[0]) <= zone_size[0] &&
            std::abs(target[1] - zone[1]) <= zone_size[1] &&
            target[2] > zone[2];
}

bool ReadBodyPosition(const ExecutorConfig& config, const char* body_name,
        std::array<float, 3>* position) {
    std::string error;
    const std::shared_ptr<MujocoSimulation> simulation =
        MujocoSimulation::Get(config.mujoco.xml_path, &error);
    if (!simulation || position == nullptr) {
        std::cerr << "failed to inspect release scene: " << error << std::endl;
        return false;
    }
    mjModel* model = simulation->model();
    mjData* data = simulation->data();
    const int body_id = mj_name2id(model, mjOBJ_BODY, body_name);
    if (body_id < 0) {
        std::cerr << "body not found in release scene: " << body_name
                << std::endl;
        return false;
    }
    mj_forward(model, data);
    const mjtNum* body_position = data->xpos + 3 * body_id;
    *position = {
        static_cast<float>(body_position[0]),
        static_cast<float>(body_position[1]),
        static_cast<float>(body_position[2]),
    };
    return true;
}

bool ReadBodyYaw(const ExecutorConfig& config, const char* body_name,
        float* yaw) {
    std::string error;
    const std::shared_ptr<MujocoSimulation> simulation =
        MujocoSimulation::Get(config.mujoco.xml_path, &error);
    if (!simulation || yaw == nullptr) {
        std::cerr << "failed to inspect release scene: " << error << std::endl;
        return false;
    }
    mjModel* model = simulation->model();
    mjData* data = simulation->data();
    const int body_id = mj_name2id(model, mjOBJ_BODY, body_name);
    if (body_id < 0) {
        std::cerr << "body not found in release scene: " << body_name
                << std::endl;
        return false;
    }
    mj_forward(model, data);
    const mjtNum* rotation = data->xmat + 9 * body_id;
    *yaw = static_cast<float>(std::atan2(rotation[3], rotation[0]));
    return true;
}

bool InitialPickupLayoutIsSeparated(const ExecutorConfig& config) {
    struct ObjectEnvelope {
        const char* body_name;
        float half_extent_x;
        float half_extent_y;
    };
    const std::array<ObjectEnvelope, 4> objects = {{
        {"Apple", 0.040f, 0.040f},
        {"Banana", 0.105f, 0.030f},
        {"Cup", 0.083f, 0.048f},
        {"Box", 0.032f, 0.032f},
    }};
    std::string error;
    const std::shared_ptr<MujocoSimulation> simulation =
        MujocoSimulation::Get(config.mujoco.xml_path, &error);
    if (!simulation) {
        std::cerr << "failed to inspect pickup zone: " << error << std::endl;
        return false;
    }
    mjModel* model = simulation->model();
    mjData* data = simulation->data();
    const int zone_body_id = mj_name2id(
        model, mjOBJ_BODY, "zone_pickup");
    const int zone_geom_id = mj_name2id(
        model, mjOBJ_GEOM, "zone_pickup");
    if (zone_body_id < 0 || zone_geom_id < 0) return false;
    mj_forward(model, data);
    const mjtNum* zone_position = data->xpos + 3 * zone_body_id;
    const mjtNum* zone_size = model->geom_size + 3 * zone_geom_id;

    std::array<std::array<float, 3>, 4> positions{};
    std::array<float, 4> projected_half_x{};
    std::array<float, 4> projected_half_y{};
    for (size_t index = 0; index < objects.size(); ++index) {
        if (!ReadBodyPosition(
                config, objects[index].body_name, &positions[index])) {
            return false;
        }
        float yaw = 0.0f;
        if (!ReadBodyYaw(config, objects[index].body_name, &yaw)) {
            return false;
        }
        const float cosine = std::abs(std::cos(yaw));
        const float sine = std::abs(std::sin(yaw));
        projected_half_x[index] =
            cosine * objects[index].half_extent_x +
            sine * objects[index].half_extent_y;
        projected_half_y[index] =
            sine * objects[index].half_extent_x +
            cosine * objects[index].half_extent_y;
        if (std::abs(positions[index][0] - zone_position[0]) +
                projected_half_x[index] > zone_size[0] ||
            std::abs(positions[index][1] - zone_position[1]) +
                projected_half_y[index] > zone_size[1]) {
            std::cerr << objects[index].body_name
                    << " is outside the pickup zone" << std::endl;
            return false;
        }
        const float camera_row_center = zone_position[1] - 0.080f;
        if (std::abs(positions[index][1] - camera_row_center) +
                projected_half_y[index] > 0.340f) {
            std::cerr << objects[index].body_name
                    << " is outside the camera-visible pickup row"
                    << std::endl;
            return false;
        }
    }
    for (size_t first = 0; first < objects.size(); ++first) {
        for (size_t second = first + 1; second < objects.size(); ++second) {
            constexpr float kMinimumObjectGapM = 0.055f;
            const bool separated_y =
                std::abs(positions[first][1] - positions[second][1]) >=
                projected_half_y[first] + projected_half_y[second] +
                    kMinimumObjectGapM;
            if (!separated_y) {
                std::cerr << objects[first].body_name << " overlaps "
                        << objects[second].body_name << std::endl;
                return false;
            }
        }
    }
    const auto [minimum_x, maximum_x] = std::minmax_element(
        positions.begin(), positions.end(),
        [](const auto& lhs, const auto& rhs) {
            return lhs[0] < rhs[0];
        });
    if ((*maximum_x)[0] - (*minimum_x)[0] > 0.012f) {
        std::cerr << "pickup objects are not arranged in one row"
                << std::endl;
        return false;
    }
    return true;
}

bool ZonesAreWideSeparatedAndRightOfRobot(const ExecutorConfig& config) {
    std::string error;
    const std::shared_ptr<MujocoSimulation> simulation =
        MujocoSimulation::Get(config.mujoco.xml_path, &error);
    if (!simulation) return false;
    mjModel* model = simulation->model();
    mjData* data = simulation->data();
    const int pickup_body_id = mj_name2id(
        model, mjOBJ_BODY, "zone_pickup");
    const int pickup_geom_id = mj_name2id(
        model, mjOBJ_GEOM, "zone_pickup");
    const int drop_body_id = mj_name2id(model, mjOBJ_BODY, "zone_drop");
    const int drop_geom_id = mj_name2id(model, mjOBJ_GEOM, "zone_drop");
    if (pickup_body_id < 0 || pickup_geom_id < 0 ||
        drop_body_id < 0 || drop_geom_id < 0) {
        return false;
    }
    mj_forward(model, data);
    const mjtNum* pickup_position = data->xpos + 3 * pickup_body_id;
    const mjtNum* pickup_size = model->geom_size + 3 * pickup_geom_id;
    const mjtNum* drop_position = data->xpos + 3 * drop_body_id;
    const mjtNum* drop_size = model->geom_size + 3 * drop_geom_id;
    const bool separated =
        drop_position[1] + drop_size[1] <
        pickup_position[1] - pickup_size[1];
    return pickup_size[0] >= 0.15 && drop_size[0] >= 0.15 &&
        drop_position[1] < 0.0 && separated;
}

bool CupRemainsStableAtRest(const ExecutorConfig& config) {
    std::string error;
    const std::shared_ptr<MujocoSimulation> simulation =
        MujocoSimulation::Get(config.mujoco.xml_path, &error);
    if (!simulation) return false;
    std::lock_guard<std::mutex> lock(simulation->mutex());
    simulation->Reset();
    mjModel* model = simulation->model();
    mjData* data = simulation->data();
    const int cup_body_id = mj_name2id(model, mjOBJ_BODY, "Cup");
    if (cup_body_id < 0) return false;
    mj_forward(model, data);
    const mjtNum* initial_position = data->xpos + 3 * cup_body_id;
    const std::array<double, 3> before = {
        initial_position[0], initial_position[1], initial_position[2]};
    for (int step = 0; step < 1000; ++step) {
        mj_step(model, data);
    }
    const mjtNum* position = data->xpos + 3 * cup_body_id;
    const mjtNum* rotation = data->xmat + 9 * cup_body_id;
    const double translation = std::hypot(
        position[0] - before[0], position[1] - before[1]);
    if (translation > 0.005 || std::abs(position[2] - before[2]) > 0.005 ||
        rotation[8] < 0.99) {
        std::cerr << "cup moved while resting: translation=" << translation
                << " z_before=" << before[2] << " z_after=" << position[2]
                << " upright=" << rotation[8] << std::endl;
        return false;
    }
    return true;
}

bool ResetChangesObjectPositionAndYaw(
        const ExecutorConfig& config, MujocoGraspExecutor* executor) {
    std::array<float, 3> before_position{};
    std::array<float, 3> after_position{};
    float before_yaw = 0.0f;
    float after_yaw = 0.0f;
    if (!ReadBodyPosition(config, "Banana", &before_position) ||
        !ReadBodyYaw(config, "Banana", &before_yaw)) {
        return false;
    }
    executor->ResetScene();
    if (!ReadBodyPosition(config, "Banana", &after_position) ||
        !ReadBodyYaw(config, "Banana", &after_yaw)) {
        return false;
    }
    const float position_delta = std::hypot(
        after_position[0] - before_position[0],
        after_position[1] - before_position[1]);
    const float yaw_delta = std::abs(std::remainder(
        after_yaw - before_yaw, 2.0f * static_cast<float>(M_PI)));
    return position_delta > 1e-4f && yaw_delta > 1e-4f &&
        InitialPickupLayoutIsSeparated(config);
}

Pose3D MakeRadialSidePose(const std::array<float, 3>& object_position,
        float height_m) {
    const double radial_norm = std::hypot(
        static_cast<double>(object_position[0]),
        static_cast<double>(object_position[1]));
    const double radial_x = object_position[0] / radial_norm;
    const double radial_y = object_position[1] / radial_norm;
    const mjtNum rotation[9] = {
        0.0, -radial_y, radial_x,
        0.0, radial_x, radial_y,
        -1.0, 0.0, 0.0,
    };
    mjtNum quaternion[4] = {1.0, 0.0, 0.0, 0.0};
    mju_mat2Quat(quaternion, rotation);
    return Pose3D{
        object_position[0], object_position[1], height_m,
        static_cast<float>(quaternion[0]),
        static_cast<float>(quaternion[1]),
        static_cast<float>(quaternion[2]),
        static_cast<float>(quaternion[3]),
    };
}

bool TestReleaseScenePhysicalGrasps() {
    setenv("PERCEPTIVE_GRASP_MUJOCO_LAYOUT_SEED", "4", 1);
    const ExecutorConfig config = MakeReleaseSceneConfig();
    MujocoGraspExecutor executor(config);
    if (!executor.Init()) return false;
    if (!InitialPickupLayoutIsSeparated(config) ||
        !ZonesAreWideSeparatedAndRightOfRobot(config) ||
        !CupRemainsStableAtRest(config) ||
        !ResetChangesObjectPositionAndYaw(config, &executor)) {
        return false;
    }

    if (executor.CloseGripperAndCheck() != GraspResult::EMPTY) {
        std::cerr << "empty gripper close was reported as a grasp" << std::endl;
        return false;
    }

    executor.SetTargetLabel("apple");
    std::array<float, 3> apple_position{};
    if (!ReadBodyPosition(config, "Apple", &apple_position)) return false;
    const Pose3D apple_pre{
        apple_position[0], apple_position[1], apple_position[2] + 0.250f,
        0.0f, 0.0f, 1.0f, 0.0f};
    const Pose3D apple_grasp{
        apple_position[0], apple_position[1], 0.045f,
        0.0f, 0.0f, 1.0f, 0.0f};
    if (!ExpectSuccess("apple observe", executor.MoveToObserve()) ||
        !ExpectSuccess("apple pre-grasp",
            executor.MoveToPreGrasp(apple_pre, NAN, true)) ||
        !ExpectSuccess("apple open", executor.OpenGripperForGrasp()) ||
        !ExpectSuccess("apple grasp move",
            executor.MoveToGrasp(apple_grasp, NAN, true)) ||
        !ExpectSuccess("apple close", executor.CloseGripperAndCheck()) ||
        !ExpectSuccess("apple lift",
            executor.LiftFromGrasp(
                apple_pre, apple_pre, NAN, true)) ||
        !ExpectSuccess("apple place", executor.MoveToPlace()) ||
        !ExpectSuccess("apple release", executor.ReleaseObject()) ||
        !ExpectSuccess("apple home", executor.MoveToObserve()) ||
        !TargetInsideDropZone(config, "Apple")) {
        std::cerr << "apple physical grasp did not finish in the drop zone"
                << ": " << executor.GetDiagnostics().last_detail
                << std::endl;
        return false;
    }

    executor.SetTargetLabel("banana");
    if (!TargetInsideDropZone(config, "Apple")) {
        std::cerr << "selecting the next target reset the placed apple"
                << std::endl;
        return false;
    }
    std::array<float, 3> banana_position{};
    if (!ReadBodyPosition(config, "Banana", &banana_position)) return false;
    float banana_wrist_yaw = 0.0f;
    if (!ReadBodyYaw(config, "Banana", &banana_wrist_yaw)) return false;
    const Pose3D banana_pre{
        banana_position[0], banana_position[1],
        banana_position[2] + 0.250f,
        0.0f, 0.0f, 1.0f, 0.0f};
    const Pose3D banana_grasp{
        banana_position[0], banana_position[1], 0.045f,
        0.0f, 0.0f, 1.0f, 0.0f};
    if (!ExpectSuccess("banana observe", executor.MoveToObserve()) ||
        !ExpectSuccess("banana pre-grasp",
            executor.MoveToPreGrasp(
                banana_pre, banana_wrist_yaw, true)) ||
        !ExpectSuccess("banana open", executor.OpenGripperForGrasp()) ||
        !ExpectSuccess("banana grasp move",
            executor.MoveToGrasp(
                banana_grasp, banana_wrist_yaw, true)) ||
        !ExpectSuccess("banana close", executor.CloseGripperAndCheck()) ||
        !ExpectSuccess("banana lift",
            executor.LiftFromGrasp(
                banana_pre, banana_pre, banana_wrist_yaw, true)) ||
        !ExpectSuccess("banana place", executor.MoveToPlace()) ||
        !ExpectSuccess("banana release", executor.ReleaseObject()) ||
        !TargetInsideDropZone(config, "Banana") ||
        !ExpectSuccess(
            "banana home", executor.MoveToObserve(), &executor)) {
        std::cerr << "banana physical grasp did not finish in the drop zone"
                << ": " << executor.GetDiagnostics().last_detail
                << std::endl;
        return false;
    }
    if (!TargetInsideDropZone(config, "Banana")) {
        std::array<float, 3> position{};
        ReadBodyPosition(config, "Banana", &position);
        std::cerr << "banana left the drop zone while returning home: ["
                << position[0] << "," << position[1] << ","
                << position[2] << "]" << std::endl;
        return false;
    }

    executor.SetTargetLabel("cup");
    std::array<float, 3> cup_position{};
    if (!ReadBodyPosition(config, "Cup", &cup_position)) return false;
    const Pose3D cup_grasp = MakeRadialSidePose(cup_position, 0.168f);
    const float radial_norm = std::hypot(
        cup_position[0], cup_position[1]);
    const float radial_x = cup_position[0] / radial_norm;
    const float radial_y = cup_position[1] / radial_norm;
    Pose3D cup_pre = cup_grasp;
    cup_pre.x -= 0.100f * radial_x;
    cup_pre.y -= 0.100f * radial_y;
    Pose3D cup_retreat = cup_grasp;
    cup_retreat.z += 0.050f;
    Pose3D cup_lift = cup_pre;
    cup_lift.z += 0.050f;
    std::string validation_detail;
    if (!ExpectSuccess(
            "cup path validation",
            executor.ValidateGraspPoses(
                cup_pre, cup_grasp, cup_retreat, cup_lift, 0.372f, 0.0f,
                false, 1000, &validation_detail)) ||
        !ExpectSuccess(
            "cup side observe", executor.MoveToSideObserve(), &executor) ||
        !ExpectSuccess("cup pre-grasp",
            executor.MoveToPreGrasp(cup_pre, 0.0f, false)) ||
        !ExpectSuccess("cup open", executor.OpenGripperForGrasp()) ||
        !ExpectSuccess("cup grasp move",
            executor.MoveToGrasp(cup_grasp, 0.0f, false)) ||
        !ExpectSuccess("cup close", executor.CloseGripperAndCheck()) ||
        !ExpectSuccess("cup lift",
            executor.LiftFromGrasp(
                cup_retreat, cup_lift, 0.0f, false)) ||
        !ExpectSuccess("cup place", executor.MoveToPlace(), &executor) ||
        !ExpectSuccess("cup release", executor.ReleaseObject()) ||
        !TargetInsideDropZone(config, "Cup")) {
        std::cerr << "cup physical grasp did not finish in the drop zone; "
                << validation_detail << std::endl;
        return false;
    }

    executor.SetTargetLabel("cube");
    if (!TargetInsideDropZone(config, "Apple") ||
        !TargetInsideDropZone(config, "Banana") ||
        !TargetInsideDropZone(config, "Cup")) {
        std::cerr << "selecting the fourth target reset a placed object"
                << std::endl;
        return false;
    }
    std::array<float, 3> box_position{};
    if (!ReadBodyPosition(config, "Box", &box_position)) return false;
    const Pose3D box_pre{
        box_position[0], box_position[1], box_position[2] + 0.250f,
        0.0f, 0.0f, 1.0f, 0.0f};
    const Pose3D box_grasp{
        box_position[0], box_position[1], 0.045f,
        0.0f, 0.0f, 1.0f, 0.0f};
    if (!ExpectSuccess("box observe", executor.MoveToObserve()) ||
        !ExpectSuccess("box pre-grasp",
            executor.MoveToPreGrasp(box_pre, NAN, true)) ||
        !ExpectSuccess("box open", executor.OpenGripperForGrasp()) ||
        !ExpectSuccess("box grasp move",
            executor.MoveToGrasp(box_grasp, NAN, true)) ||
        !ExpectSuccess("box close", executor.CloseGripperAndCheck()) ||
        !ExpectSuccess("box lift",
            executor.LiftFromGrasp(box_pre, box_pre, NAN, true)) ||
        !ExpectSuccess("box place", executor.MoveToPlace(), &executor) ||
        !ExpectSuccess("box release", executor.ReleaseObject()) ||
        !TargetInsideDropZone(config, "Box") ||
        !ExpectSuccess("box home", executor.MoveToObserve(), &executor)) {
        std::cerr << "box physical grasp did not finish in the drop zone"
                << ": " << executor.GetDiagnostics().last_detail
                << std::endl;
        return false;
    }
    return true;
}

}  // namespace

int main() {
    const std::string xml_path = WriteTestMjcf();
    ExecutorConfig config = MakeConfig(xml_path);
    MujocoGraspExecutor executor(config);

    if (!executor.Init()) {
        std::cerr << "mujoco executor init failed" << std::endl;
        std::remove(xml_path.c_str());
        return 1;
    }
    if (!ExpectSuccess("observe", executor.MoveToObserve()) ||
        !ExpectSuccess("side_observe", executor.MoveToSideObserve()) ||
        !ExpectSuccess("place", executor.MoveToPlace()) ||
        !ExpectSuccess("home", executor.MoveToHome())) {
        std::remove(xml_path.c_str());
        return 1;
    }

    executor.EmergencyStop();
    const auto emergency_diagnostics = executor.GetDiagnostics();
    if (emergency_diagnostics.last_result != GraspResult::SUCCESS ||
        emergency_diagnostics.last_action != "emergency_stop") {
        std::cerr << "emergency stop did not hold the current state"
                << std::endl;
        std::remove(xml_path.c_str());
        return 1;
    }

    Pose3D pose;
    if (!executor.GetCurrentPose(pose) ||
        !std::isfinite(pose.x) ||
        !std::isfinite(pose.y) ||
        !std::isfinite(pose.z)) {
        std::cerr << "current pose is invalid" << std::endl;
        std::remove(xml_path.c_str());
        return 1;
    }

    if (!TestReleaseScenePhysicalGrasps()) {
        std::remove(xml_path.c_str());
        return 1;
    }

    std::remove(xml_path.c_str());
    return 0;
}
