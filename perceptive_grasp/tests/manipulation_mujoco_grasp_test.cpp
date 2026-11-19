/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file manipulation_mujoco_grasp_test.cpp
 * @brief Physical grasp regression using the shared manipulation simulation world.
 */
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17): the application requires C++17.
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include "grasp_executor.h"
#include "mujoco_test_config.h"
#include "manipulation/device_plan.h"
#include "manipulation/mujoco_runtime.h"
#include "mujoco_cup_placement.h"
#include "mujoco_grasp_executor.h"
#include "mujoco_joint_path.h"
#include "mujoco_native_snapshot.h"
#include "mujoco_world.h"

namespace {
using perceptive_grasp::ExecutorConfig;
using perceptive_grasp::GraspResult;
using perceptive_grasp::MujocoGraspExecutor;
using perceptive_grasp::Pose3D;
using perceptive_grasp::test::Configuration;


void CheckRejected(const ExecutorConfig& config, const std::string& expected) {
    MujocoGraspExecutor executor(config);
    if (executor.Init() || executor.SimulationRuntime() ||
        executor.GetDiagnostics().last_detail.find(expected) == std::string::npos) {
        throw std::runtime_error("invalid planning/device contract was not rejected: " + expected);
    }
}

void CheckContracts() {
    auto missing = Configuration();
    missing.mujoco.device_plan.clear();
    CheckRejected(missing, "device_plan is required");
    auto config = Configuration();
    const auto path = std::filesystem::temp_directory_path() /
        ("grasp-contract-" + std::to_string(std::chrono::steady_clock::now()
            .time_since_epoch().count()));
    if (!std::filesystem::create_directory(path)) throw std::runtime_error("temporary directory");
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() { std::error_code error; std::filesystem::remove_all(path, error); }
    } cleanup{path};
    auto device = YAML::LoadFile(config.mujoco.device_plan);
    auto mapping = YAML::LoadFile((std::filesystem::path(config.mujoco.device_plan)
        .parent_path() / "mapping.yaml").string());
    mapping["scene"] = std::string(PERCEPTIVE_GRASP_SOURCE_DIR) + "/simulation/mujoco/manipulation_scene.xml";
    device["device"]["arm"]["options"]["mapping_file"] = (path / "mapping.yaml").string();
    device["device"]["effector"]["options"]["mapping_file"] = (path / "mapping.yaml").string();
    // Custom mappings may use another order; the application's IK still has a fixed order.
    device["device"]["arm"]["driver"] = "mujoco_mapped_arm";
    const auto first_joint = YAML::Clone(device["device"]["arm"]["joints"][0]);
    device["device"]["arm"]["joints"][0] = YAML::Clone(device["device"]["arm"]["joints"][1]);
    device["device"]["arm"]["joints"][1] = first_joint;
    const auto first_mapping = YAML::Clone(mapping["arm"][0]);
    mapping["arm"][0] = YAML::Clone(mapping["arm"][1]);
    mapping["arm"][1] = first_mapping;
    std::ofstream(path / "mapping.yaml") << mapping;
    std::ofstream(path / "device.yaml") << device;
    config.mujoco.device_plan = (path / "device.yaml").string();
    // This remains a valid registered driver plan, but no longer matches the application's IK.
    const auto registry = spacemit::manipulation::MakeBuiltinDeviceRegistry();
    spacemit::manipulation::LoadDevicePlan(config.mujoco.device_plan, registry);
    CheckRejected(config, "joint order must match");
    config = Configuration();
    device = YAML::LoadFile(config.mujoco.device_plan);
    mapping = YAML::LoadFile((std::filesystem::path(config.mujoco.device_plan)
        .parent_path() / "mapping.yaml").string());
    mapping["scene"] = std::string(PERCEPTIVE_GRASP_SOURCE_DIR) + "/simulation/mujoco/manipulation_scene.xml";
    device["device"]["arm"]["options"]["mapping_file"] = (path / "mapping.yaml").string();
    device["device"]["effector"]["options"]["mapping_file"] = (path / "mapping.yaml").string();
    std::ofstream(path / "device.yaml") << device;
    config.mujoco.device_plan = (path / "device.yaml").string();
    mapping.remove("synchronized_joint_point");
    std::ofstream(path / "mapping.yaml") << mapping;
    CheckRejected(config, "requires synchronized_joint_point");
    mapping["synchronized_joint_point"] = false;
    std::ofstream(path / "mapping.yaml") << mapping;
    CheckRejected(config, "requires synchronized_joint_point");
    config = Configuration();
    config.mujoco.actuator_names[0] = config.mujoco.actuator_names[1];
    CheckRejected(config, "actuators must match");
    config = Configuration();
    config.mujoco.gripper_actuator = "robot_shoulder_pan";
    CheckRejected(config, "gripper endpoints must match");
    config = Configuration();
    std::swap(config.mujoco.gripper_open_ctrl, config.mujoco.gripper_close_ctrl);
    CheckRejected(config, "gripper endpoints must match");
    config = Configuration();
    config.mujoco.gripper_open_ctrl = NAN;
    CheckRejected(config, "gripper endpoints must match");
    config = Configuration();
    device = YAML::LoadFile(config.mujoco.device_plan);
    mapping = YAML::LoadFile((std::filesystem::path(config.mujoco.device_plan)
        .parent_path() / "mapping.yaml").string());
    mapping["scene"] = std::string(PERCEPTIVE_GRASP_SOURCE_DIR) + "/simulation/mujoco/manipulation_scene.xml";
    // A valid split of the scene's final arm actuator into a second tool channel is not a
    // single-gripper application profile, even though the common driver can represent it.
    device["device"]["arm"]["driver"] = "mujoco_mapped_arm";
    device["device"]["effector"]["driver"] = "mujoco_mapped_effector";
    device["device"]["arm"]["joints"].remove(5);
    mapping["arm"].remove(5);
    config.mujoco.joint_names.pop_back();
    config.mujoco.actuator_names.pop_back();
    device["device"]["effector"]["channels"].push_back("wrist_tool");
    auto second_tool = YAML::Clone(mapping["effector"][0]);
    second_tool["channel"] = "wrist_tool";
    second_tool["joint"] = "robot_wrist_3_joint";
    second_tool["actuator"] = "robot_wrist_3";
    second_tool["joint_closed"] = second_tool["control_closed"] = 0.0;
    second_tool["joint_open"] = second_tool["control_open"] = 1.0;
    mapping["effector"].push_back(second_tool);
    device["device"]["arm"]["options"]["mapping_file"] = (path / "mapping.yaml").string();
    device["device"]["effector"]["options"]["mapping_file"] = (path / "mapping.yaml").string();
    std::ofstream(path / "mapping.yaml") << mapping;
    std::ofstream(path / "device.yaml") << device;
    config.mujoco.device_plan = (path / "device.yaml").string();
    spacemit::manipulation::LoadDevicePlan(config.mujoco.device_plan, registry);
    CheckRejected(config, "one gripper channel");
    config = Configuration();
    device = YAML::LoadFile(config.mujoco.device_plan);
    const auto original_mapping = (std::filesystem::path(config.mujoco.device_plan)
        .parent_path() / "mapping.yaml").string();
    device["device"]["arm"]["options"]["mapping_file"] = original_mapping;
    device["device"]["effector"]["options"]["mapping_file"] = original_mapping;
    device["device"]["arm"]["driver"] = "mujoco_mapped_arm";
    config.mujoco.device_plan = (path / "device.yaml").string();
    for (const auto* effector_driver : {"mujoco_mapped_effector", "mujoco_robotiq_2f85"}) {
        device["device"]["effector"]["driver"] = effector_driver;
        std::ofstream(path / "device.yaml") << device;
        MujocoGraspExecutor executor(config);
        if (!executor.Init() || !executor.SimulationRuntime()) {
            throw std::runtime_error("compatible simulation driver names were rejected: " +
                executor.GetDiagnostics().last_detail);
        }
    }
    device["device"]["arm"]["driver"] = "mujoco";
    std::ofstream(path / "device.yaml") << device;
    CheckRejected(config, "unknown arm driver");
    device["device"]["arm"]["driver"] = "mujoco_ur5e";
    device["device"]["effector"]["driver"] = "mujoco";
    std::ofstream(path / "device.yaml") << device;
    CheckRejected(config, "unknown effector driver");
    std::cout << "planning/device contracts and named simulation drivers passed" << std::endl;
}

std::array<float, 3> BodyPosition(MujocoGraspExecutor* executor, const std::string& body) {
    static mujoco_sim::NativeSnapshot snapshot(executor->SimulationRuntime()->Snapshot());
    snapshot.Update(executor->SimulationRuntime()->Snapshot());
    const int id = mj_name2id(snapshot.Model(), mjOBJ_BODY, body.c_str());
    if (id < 0) throw std::runtime_error("missing target " + body);
    const auto* p = snapshot.Data()->xpos + 3 * id;
    return {static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])};
}

void Check(GraspResult result, const std::string& stage, MujocoGraspExecutor* executor) {
    if (result != GraspResult::SUCCESS) {
        throw std::runtime_error(stage + ": " + executor->GetDiagnostics().last_detail);
    }
    std::cout << stage << " completed" << std::endl;
}

void CheckPlaced(MujocoGraspExecutor* executor, const std::string& body) {
    const auto actual = BodyPosition(executor, body);
    if (std::abs(actual[0] - 0.56f) > 0.44f || std::abs(actual[1] + 0.40f) > 0.17f ||
            actual[2] < 0.0f) {
        throw std::runtime_error(body + " not physically inside drop area");
    }
}

void CheckFailedReleaseRetainsConstraint() {
    auto config = Configuration();
    const auto path = std::filesystem::temp_directory_path() /
        ("grasp-release-" + std::to_string(std::chrono::steady_clock::now()
            .time_since_epoch().count()));
    if (!std::filesystem::create_directory(path)) throw std::runtime_error("temporary directory");
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() { std::error_code error; std::filesystem::remove_all(path, error); }
    } cleanup{path};
    auto device = YAML::LoadFile(config.mujoco.device_plan);
    const auto mapping = (std::filesystem::path(config.mujoco.device_plan)
        .parent_path() / "mapping.yaml").string();
    device["device"]["arm"]["options"]["mapping_file"] = mapping;
    device["device"]["effector"]["options"]["mapping_file"] = mapping;
    // Streaming contact closure is separate from a point command. A short point deadline
    // lets us refuse the later opening before any write, with a real held object present.
    device["device"]["effector"]["timeout_s"] = 0.05;
    std::ofstream(path / "device.yaml") << device;
    config.mujoco.device_plan = (path / "device.yaml").string();
    MujocoGraspExecutor executor(config);
    if (!executor.Init()) throw std::runtime_error(executor.GetDiagnostics().last_detail);
    executor.SetTargetLabel("apple");
    const auto position = BodyPosition(&executor, "Apple");
    const Pose3D pre{position[0], position[1], position[2] + 0.25f, 0, 0, 1, 0};
    const Pose3D grasp{position[0], position[1], 0.045f, 0, 0, 1, 0};
    Check(executor.MoveToPreGrasp(pre, NAN, true), "retained release pregrasp", &executor);
    Check(executor.MoveToGrasp(grasp, NAN, true), "retained release approach", &executor);
    Check(executor.CloseGripperAndCheck(), "retained release contact", &executor);
    mujoco_sim::NativeSnapshot snapshot(executor.SimulationRuntime()->Snapshot());
    const int weld = mj_name2id(snapshot.Model(), mjOBJ_EQUALITY, "grasp_weld");
    if (weld < 0 || !snapshot.Data()->eq_active[weld]) {
        throw std::runtime_error("release regression requires an attached object");
    }
    int rendering_calls = 0;
    executor.SetVisualStepCallback([&] {
        ++rendering_calls;
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
    });
    bool refused = false;
    try {
        executor.ReleaseObject();
    } catch (const spacemit::manipulation::CommandRejected& error) {
        refused = std::string(error.what()).find("exceeds the effector timeout_s") !=
            std::string::npos;
    }
    snapshot.Update(executor.SimulationRuntime()->Snapshot());
    if (!refused || !snapshot.Data()->eq_active[weld] || rendering_calls != 0) {
        throw std::runtime_error("failed opening released the object constraint");
    }
    executor.SetVisualStepCallback({});
    executor.EmergencyStop();
    Check(executor.GetDiagnostics().last_result, "retained release hold", &executor);
    std::cout << "opening refused without releasing the held object" << std::endl;
}

void CheckPlanningPathGuard() {
    using perceptive_grasp::MujocoCupReleaseReady;
    if (!MujocoCupReleaseReady(.0025, .001, true) ||
        MujocoCupReleaseReady(.001999, .001, true) || MujocoCupReleaseReady(.003001, .001, true) ||
        MujocoCupReleaseReady(.0025, .025001, true) || MujocoCupReleaseReady(.0025, .001, false) ||
        MujocoCupReleaseReady(NAN, .001, true)) {
        throw std::runtime_error("low cup release clearance/attitude/region gate failed");
    }
    const mjtNum identity[4] = {1, 0, 0, 0};
    const mjtNum rotated[4] = {std::cos(.01), std::sin(.01), 0, 0};
    if (std::abs(perceptive_grasp::MujocoQuaternionDistance(identity, rotated) - .02) > 1e-12) {
        throw std::runtime_error("retreat orientation error gate is not measured in radians");
    }
    // Exercise the same transform helper used by the actual cup placement path.
    const mjtNum centre[3] = {0.56, -0.4, 0.22};
    const mjtNum tilted_object[4] = {0.977, 0.089, 0.065, -0.182};
    mjtNum object[4], relative[4] = {0.7, -0.3, 0.5, 0.4};
    mju_copy4(object, tilted_object);
    mju_normalize4(object);
    mju_normalize4(relative);
    const mjtNum offset[3] = {0.018, -0.025, 0.08};
    const auto upright = perceptive_grasp::MujocoUprightHoldPose(centre, object, offset, relative);
    mjtNum recovered_offset[3], recovered_object[4], rotation[9], original_rotation[9];
    mju_rotVecQuat(recovered_offset, offset, upright.data() + 3);
    mju_mulQuat(recovered_object, upright.data() + 3, relative);
    mju_quat2Mat(rotation, recovered_object);
    mju_quat2Mat(original_rotation, object);
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(upright[axis] + recovered_offset[axis] - centre[axis]) > 1e-12) {
            throw std::runtime_error("upright transform moved the cup centre");
        }
    }
    if (std::abs(rotation[8] - 1) > 1e-12 ||
        std::abs(std::atan2(rotation[3], rotation[0]) -
            std::atan2(original_rotation[3], original_rotation[0])) > 1e-12) {
        throw std::runtime_error("upright transform did not preserve yaw and vertical cup axis");
    }
    const auto path = std::filesystem::temp_directory_path() /
        ("grasp-path-" + std::to_string(std::chrono::steady_clock::now()
            .time_since_epoch().count()) + ".xml");
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() { std::error_code error; std::filesystem::remove(path, error); }
    } cleanup{path};
    std::ofstream(path) << R"(<mujoco><worldbody>
        <geom name="obstacle" type="sphere" size="0.04" pos="0 0 0.2"/>
        <body name="held_object" pos="0 0 0.2"><joint name="slide" type="slide" axis="1 0 0"/>
        <geom name="object" type="sphere" size="0.03"/></body></worldbody>
        <keyframe><key name="start" qpos="-0.2"/></keyframe></mujoco>)";
    mujoco_sim::World world(path.string());
    world.Reset("start");
    const auto original = world.Snapshot();
    mujoco_sim::NativeSnapshot native(original);
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_copyModel(nullptr, native.Model()), mj_deleteModel);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    mj_copyData(data.get(), model.get(), native.Data());
    const auto apply = [&](const std::vector<float>& point) {
        data->qpos[0] = point[0];
        mj_fwdPosition(model.get(), data.get());
    };
    const auto collision = [&](std::string*) { return data->ncon > 0; };
    int restored = 0;
    const auto restore = [&] {
        ++restored;
        mj_copyData(data.get(), model.get(), native.Data());
    };
    std::string detail;
    apply({-0.2f});
    if (collision(nullptr)) throw std::runtime_error("path start is not clear");
    apply({0.2f});
    if (collision(nullptr)) throw std::runtime_error("path endpoint is not clear");
    restore();
    if (perceptive_grasp::CheckMujocoJointPath(
            {-0.2f}, {0.2f}, apply, collision, restore, &detail)) {
        throw std::runtime_error("intermediate held-object obstacle was missed");
    }
    if (!perceptive_grasp::CheckMujocoJointPath(
            {-0.2f}, {-0.1f}, apply, collision, restore, &detail)) {
        throw std::runtime_error("clear sampled path was rejected");
    }
    bool exception = false;
    try {
        perceptive_grasp::CheckMujocoJointPath({-0.2f}, {0.2f}, apply,
            [](std::string*) -> bool { throw std::runtime_error("injected planning failure"); },
            restore, &detail);
    } catch (const std::runtime_error& error) {
        exception = std::string(error.what()) == "injected planning failure";
    }
    if (!exception || restored != 4 || std::abs(data->qpos[0] + 0.2) > 1e-12 ||
        world.SampleId() != original.SampleId() ||
        std::abs(world.Joint("slide").position + 0.2) > 1e-12) {
        throw std::runtime_error("planning rejection/exception changed the live world or copy");
    }
    std::cout << "intermediate obstacle rejected; success/failure/exception restored planning data"
        << std::endl;

    // A flat cylinder needs a support manifold rather than a single contact that changes
    // sides each step. Use the scene's solver settings and unchanged settling thresholds.
    constexpr const char* support_xml = R"(<mujoco>
        <option timestep="0.002" integrator="implicitfast" solver="Newton"
            gravity="0 0 -9.81" cone="elliptic" iterations="80"><flag multiccd="enable"/></option>
        <default><geom friction="1.3 0.03 0.02" condim="6" solref="0.006 1"
            solimp="0.9 0.98 0.002"/></default><worldbody>
        <geom type="box" pos="0 0 -0.05" size="0.8 0.6 0.05"/>
        <body pos="0 0 0.098166" quat=".985150 .000266 -.000074 -.171695">
            <joint type="free" damping="0.5"/>
            <geom type="cylinder" size=".040 .090" density="300"/>
            <geom type="cylinder" pos="0 0 -.085" size=".038 .005" density="5000"/>
        </body></worldbody></mujoco>)";
    char support_error[1024] = {};
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
        mj_parseXMLString(support_xml, nullptr, support_error, sizeof(support_error)), mj_deleteSpec);
    if (!spec) throw std::runtime_error(support_error);
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> support(mj_compile(spec.get(), nullptr), mj_deleteModel);
    if (!support) throw std::runtime_error("cannot compile cylinder support fixture");
    std::unique_ptr<mjData, decltype(&mj_deleteData)> resting(mj_makeData(support.get()), mj_deleteData);
    double quiet_seconds = 0;
    for (int step = 0; step < 1200; ++step) {
        mj_step(support.get(), resting.get());
        double linear = 0, angular = 0;
        for (int axis = 0; axis < 3; ++axis) {
            linear += std::pow(resting->qvel[axis], 2);
            angular += std::pow(resting->qvel[axis + 3], 2);
        }
        quiet_seconds = linear <= 1e-6 && angular <= .005 * .005
            ? quiet_seconds + support->opt.timestep : 0;
    }
    if (resting->ncon < 4 || quiet_seconds < .2) {
        throw std::runtime_error("flat cylinder support failed the original settling gate");
    }
    std::cout << "multi-contact cylinder support settles without changing velocity thresholds" << std::endl;
}

void TopGrasp(MujocoGraspExecutor* executor, const std::string& label, const std::string& body,
        float yaw) {
    const auto started = std::chrono::steady_clock::now();
    executor->SetTargetLabel(label);
    const auto position = BodyPosition(executor, body);
    const Pose3D pre{position[0], position[1], position[2] + 0.25f, 0, 0, 1, 0};
    const Pose3D grasp{position[0], position[1], 0.045f, 0, 0, 1, 0};
    Check(executor->MoveToObserve(), label + " observe", executor);
    Check(executor->MoveToPreGrasp(pre, yaw, true), label + " pregrasp", executor);
    Check(executor->OpenGripperForGrasp(), label + " open", executor);
    Check(executor->MoveToGrasp(grasp, yaw, true), label + " approach", executor);
    Check(executor->CloseGripperAndCheck(), label + " contact", executor);
    Check(executor->LiftFromGrasp(pre, pre, yaw, true), label + " lift", executor);
    Check(executor->MoveToPlace(), label + " place", executor);
    Check(executor->ReleaseObject(), label + " release", executor);
    Check(executor->MoveToObserve(), label + " home", executor);
    CheckPlaced(executor, body);
    std::cout << label << " cycle elapsed_s="
        << std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()
        << std::endl;
}

enum class CupFailure { kNone, kPlacement, kOpening, kDeparture };

void SideGrasp(MujocoGraspExecutor* executor, CupFailure fault = CupFailure::kNone) {
    const auto started = std::chrono::steady_clock::now();
    executor->SetTargetLabel("cup");
    const auto position = BodyPosition(executor, "Cup");
    const float radius = std::hypot(position[0], position[1]);
    const float x = position[0] / radius;
    const float y = position[1] / radius;
    const mjtNum rotation[9] = {0, -y, x, 0, x, y, -1, 0, 0};
    mjtNum quaternion[4];
    mju_mat2Quat(quaternion, rotation);
    const Pose3D grasp{position[0], position[1], 0.168f, static_cast<float>(quaternion[0]),
            static_cast<float>(quaternion[1]), static_cast<float>(quaternion[2]),
            static_cast<float>(quaternion[3])};
    Pose3D pre = grasp;
    pre.x -= 0.1f * x;
    pre.y -= 0.1f * y;
    Pose3D retreat = grasp;
    retreat.z += 0.05f;
    Pose3D lift = pre;
    lift.z += 0.05f;
    const auto before = executor->SimulationRuntime()->Snapshot();
    std::string detail;
    Check(executor->ValidateGraspPoses(
                    pre, grasp, retreat, lift, 0.372f, 0.0f, false, 1000, &detail),
            "cup planning", executor);
    const auto after = executor->SimulationRuntime()->Snapshot();
    for (const auto& name : Configuration().mujoco.joint_names) {
        if (std::abs(before.Joint(name).position - after.Joint(name).position) > 0.003) {
            throw std::runtime_error("planning changed the live robot state");
        }
    }
    Check(executor->MoveToSideObserve(), "cup observe", executor);
    Check(executor->MoveToPreGrasp(pre, 0, false), "cup pregrasp", executor);
    Check(executor->OpenGripperForGrasp(), "cup open", executor);
    Check(executor->MoveToGrasp(grasp, 0, false), "cup approach", executor);
    Check(executor->CloseGripperAndCheck(), "cup contact", executor);
    Check(executor->LiftFromGrasp(retreat, lift, 0, false), "cup lift", executor);
    if (fault == CupFailure::kPlacement) {
        mujoco_sim::NativeSnapshot snapshot(executor->SimulationRuntime()->Snapshot());
        const int cup = mj_name2id(snapshot.Model(), mjOBJ_BODY, "Cup");
        const int weld = mj_name2id(snapshot.Model(), mjOBJ_EQUALITY, "grasp_weld");
        bool interrupted = false;
        executor->SetVisualStepCallback([&] {
            snapshot.Update(executor->SimulationRuntime()->Snapshot());
            if (snapshot.Data()->xpos[3 * cup + 2] < 0.12) {
                interrupted = true;
                throw std::runtime_error("injected low placement interruption");
            }
        });
        const auto placed = executor->MoveToPlace();
        executor->SetVisualStepCallback({});
        snapshot.Update(executor->SimulationRuntime()->Snapshot());
        if (!interrupted || placed == GraspResult::SUCCESS || !snapshot.Data()->eq_active[weld] ||
            executor->MoveToHome() != GraspResult::MOVE_FAILED ||
            executor->GetDiagnostics().last_detail.find("cup placement incomplete") ==
                std::string::npos || executor->MoveToObserve() != GraspResult::MOVE_FAILED) {
            throw std::runtime_error("failed low placement released its weld or allowed later movement");
        }
        executor->EmergencyStop();
        Check(executor->GetDiagnostics().last_result, "placement failure hold", executor);
        std::cout << "failed low placement keeps its weld and blocks home and observation" << std::endl;
        return;
    }
    Check(executor->MoveToPlace(), "cup place", executor);
    if (fault == CupFailure::kOpening) {
        mujoco_sim::NativeSnapshot snapshot(executor->SimulationRuntime()->Snapshot());
        const int weld = mj_name2id(snapshot.Model(), mjOBJ_EQUALITY, "grasp_weld");
        bool interrupted = false;
        executor->SetVisualStepCallback([&] {
            interrupted = true;
            throw std::runtime_error("injected opening interruption");
        });
        bool rejected = false;
        try { rejected = executor->ReleaseObject() != GraspResult::SUCCESS; }
        catch (const std::runtime_error& error) {
            rejected = std::string(error.what()) == "injected opening interruption";
        }
        executor->SetVisualStepCallback({});
        snapshot.Update(executor->SimulationRuntime()->Snapshot());
        if (!interrupted || !rejected || !snapshot.Data()->eq_active[weld] ||
            executor->MoveToHome() != GraspResult::MOVE_FAILED ||
            executor->GetDiagnostics().last_detail.find("cup placement incomplete") ==
                std::string::npos || executor->MoveToObserve() != GraspResult::MOVE_FAILED) {
            throw std::runtime_error("failed cup opening released its weld or allowed later movement");
        }
        executor->EmergencyStop();
        Check(executor->GetDiagnostics().last_result, "opening failure hold", executor);
        std::cout << "failed cup opening keeps its weld and blocks home and observation" << std::endl;
        return;
    }
    if (fault == CupFailure::kDeparture) {
        mujoco_sim::NativeSnapshot snapshot(executor->SimulationRuntime()->Snapshot());
        const int weld = mj_name2id(snapshot.Model(), mjOBJ_EQUALITY, "grasp_weld");
        executor->SetVisualStepCallback([&] {
            snapshot.Update(executor->SimulationRuntime()->Snapshot());
            if (!snapshot.Data()->eq_active[weld]) {
                throw std::runtime_error("injected departure interruption");
            }
        });
        bool interrupted = false;
        try { executor->ReleaseObject(); } catch (const std::runtime_error& error) {
            interrupted = std::string(error.what()) == "injected departure interruption";
        }
        executor->SetVisualStepCallback({});
        if (!interrupted || executor->MoveToHome() != GraspResult::MOVE_FAILED ||
            executor->GetDiagnostics().last_detail.find("cup placement incomplete") ==
                std::string::npos || executor->MoveToObserve() != GraspResult::MOVE_FAILED) {
            throw std::runtime_error("interrupted cup departure allowed a later arm command");
        }
        executor->EmergencyStop();
        Check(executor->GetDiagnostics().last_result, "departure failure hold", executor);
        std::cout << "interrupted departure blocks home and observation" << std::endl;
        return;
    }
    Check(executor->ReleaseObject(), "cup release", executor);
    Check(executor->MoveToObserve(), "cup home", executor);
    CheckPlaced(executor, "Cup");
    std::cout << "cup cycle elapsed_s="
        << std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()
        << std::endl;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--contract-only") {
            CheckContracts();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "--release-guard-only") {
            CheckFailedReleaseRetainsConstraint();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "--path-guard-only") {
            CheckPlanningPathGuard();
            return 0;
        }
        MujocoGraspExecutor executor(Configuration());
        if (!executor.Init()) throw std::runtime_error(executor.GetDiagnostics().last_detail);
        if (!executor.SimulationRuntime()) throw std::runtime_error("missing shared world");
        if (argc == 2 && std::string(argv[1]) == "--departure-guard-only") {
            SideGrasp(&executor, CupFailure::kDeparture);
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "--placement-guard-only") {
            SideGrasp(&executor, CupFailure::kPlacement);
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "--cup-opening-guard-only") {
            SideGrasp(&executor, CupFailure::kOpening);
            return 0;
        }
        BodyPosition(&executor, "Apple");
        const auto empty_started = std::chrono::steady_clock::now();
        if (executor.CloseGripperAndCheck() != GraspResult::EMPTY) {
            throw std::runtime_error("empty close accepted as a grasp");
        }
        std::cout << "empty grasp rejected elapsed_s="
            << std::chrono::duration<double>(
                std::chrono::steady_clock::now() - empty_started).count()
            << std::endl;
        TopGrasp(&executor, "apple", "Apple", NAN);
        TopGrasp(&executor, "banana", "Banana", static_cast<float>(mjPI / 2));
        SideGrasp(&executor);
        TopGrasp(&executor, "cube", "Box", NAN);
        CheckPlaced(&executor, "Apple");
        CheckPlaced(&executor, "Banana");
        CheckPlaced(&executor, "Cup");
        executor.EmergencyStop();
        Check(executor.GetDiagnostics().last_result, "hold", &executor);
    } catch (const std::exception& error) {
        std::cerr << "shared-world grasp test: " << error.what() << std::endl;
        return 1;
    }
    return 0;
}
