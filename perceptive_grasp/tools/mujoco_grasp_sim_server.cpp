/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file mujoco_grasp_sim_server.cpp
* @brief Remote MuJoCo simulation server for perceptive_grasp.
*/

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include "mujoco_grasp_executor.h"
#include "mujoco_simulation.h"
#include "mujoco_stereo_camera.h"
#include "remote_mujoco_protocol.h"

namespace fs = std::filesystem;

namespace perceptive_grasp {
namespace {

static std::atomic<bool> g_stop{false};

struct ServerConfig {
    std::string config_path = "../config/grasp_pipeline_mujoco_ur5e.yaml";
    std::string listen_host = "127.0.0.1";
    int port = 9090;
    bool viewer = false;
    StereoCameraConfig camera;
    ExecutorConfig executor;
};

static void HandleSignal(int) {
    g_stop.store(true);
}

static void ResolveConfigPath(const fs::path& config_dir,
                            std::string* path) {
    if (path == nullptr || path->empty()) return;

    std::string expanded = *path;
    const char* home = std::getenv("HOME");
    if (home != nullptr) {
        if (expanded == "~") {
            expanded = home;
        } else if (expanded.rfind("~/", 0) == 0) {
            expanded = (fs::path(home) / expanded.substr(2)).string();
        } else if (expanded.rfind("$HOME/", 0) == 0) {
            expanded = (fs::path(home) / expanded.substr(6)).string();
        }
    }

    fs::path resolved(expanded);
    if (!resolved.is_absolute()) resolved = config_dir / resolved;
    *path = fs::weakly_canonical(resolved).string();
}

static void ParseJointList(const YAML::Node& node,
                        std::vector<float>* output) {
    if (!node) return;
    output->clear();
    for (size_t i = 0; i < node.size(); ++i) {
        output->push_back(node[i].as<float>());
    }
}

static void ParseStringList(const YAML::Node& node,
                            std::vector<std::string>* output) {
    if (!node) return;
    output->clear();
    for (size_t i = 0; i < node.size(); ++i) {
        output->push_back(node[i].as<std::string>());
    }
}

static ServerConfig LoadConfig(const std::string& path) {
    ServerConfig config;
    config.config_path = path;
    const YAML::Node root = YAML::LoadFile(path);
    fs::path config_dir = fs::path(path).parent_path();
    if (config_dir.empty()) config_dir = ".";

    const YAML::Node server = root["mujoco_server"];
    if (server) {
        config.listen_host =
            server["listen"].as<std::string>(config.listen_host);
        config.port = server["port"].as<int>(config.port);
    }

    const YAML::Node camera = root["camera"];
    const YAML::Node mujoco_camera = camera["mujoco"];
    if (!mujoco_camera || !mujoco_camera.IsMap()) {
        throw std::runtime_error("camera.mujoco configuration is required");
    }
    config.camera.type = "mujoco";
    auto& camera_settings = config.camera.mujoco;
    camera_settings.xml_path = mujoco_camera["xml_path"].as<std::string>(
        camera_settings.xml_path);
    camera_settings.camera_name =
        mujoco_camera["camera_name"].as<std::string>(
            camera_settings.camera_name);
    camera_settings.width = mujoco_camera["width"].as<int>(
        camera_settings.width);
    camera_settings.height = mujoco_camera["height"].as<int>(
        camera_settings.height);
    if (const YAML::Node depth = mujoco_camera["depth"]) {
        camera_settings.min_depth_m = depth["min_m"].as<float>(
            camera_settings.min_depth_m);
        camera_settings.max_depth_m = depth["max_m"].as<float>(
            camera_settings.max_depth_m);
    }
    ResolveConfigPath(config_dir, &camera_settings.xml_path);

    const YAML::Node manipulator = root["manipulator"];
    const YAML::Node mujoco_executor = manipulator["mujoco"];
    if (!mujoco_executor || !mujoco_executor.IsMap()) {
        throw std::runtime_error("manipulator.mujoco configuration is required");
    }
    config.executor.manip_driver = "mujoco_ur5e";
    ParseJointList(manipulator["home_joints"], &config.executor.home_joints);
    ParseJointList(manipulator["observe_joints"],
                &config.executor.observe_joints);
    ParseJointList(manipulator["side_ready_joints"],
                &config.executor.side_ready_joints);
    if (root["place"]) {
        ParseJointList(root["place"]["place_joints"],
                    &config.executor.place_joints);
    }
    auto& executor_settings = config.executor.mujoco;
    executor_settings.xml_path =
        mujoco_executor["xml_path"].as<std::string>(
            executor_settings.xml_path);
    executor_settings.end_effector_site =
        mujoco_executor["end_effector_site"].as<std::string>(
            executor_settings.end_effector_site);
    executor_settings.gripper_actuator =
        mujoco_executor["gripper_actuator"].as<std::string>(
            executor_settings.gripper_actuator);
    executor_settings.robot_root_body =
        mujoco_executor["robot_root_body"].as<std::string>(
            executor_settings.robot_root_body);
    executor_settings.gripper_root_body =
        mujoco_executor["gripper_root_body"].as<std::string>(
            executor_settings.gripper_root_body);
    executor_settings.gripper_open_ctrl =
        mujoco_executor["gripper_open_ctrl"].as<float>(
            executor_settings.gripper_open_ctrl);
    executor_settings.gripper_close_ctrl =
        mujoco_executor["gripper_close_ctrl"].as<float>(
            executor_settings.gripper_close_ctrl);
    executor_settings.gravity_compensation =
        mujoco_executor["gravity_compensation"].as<bool>(
            executor_settings.gravity_compensation);
    executor_settings.arm_stiffness_scale =
        mujoco_executor["arm_stiffness_scale"].as<float>(
            executor_settings.arm_stiffness_scale);
    if (!std::isfinite(executor_settings.arm_stiffness_scale) ||
        executor_settings.arm_stiffness_scale <= 0.0f) {
        throw std::runtime_error(
            "manipulator.mujoco.arm_stiffness_scale must be positive");
    }
    executor_settings.joint_tolerance_rad =
        mujoco_executor["joint_tolerance_rad"].as<float>(
            executor_settings.joint_tolerance_rad);
    executor_settings.ik_position_tolerance_m =
        mujoco_executor["ik_position_tolerance_m"].as<float>(
            executor_settings.ik_position_tolerance_m);
    executor_settings.cartesian_tracking_tolerance_m =
        mujoco_executor["cartesian_tracking_tolerance_m"].as<float>(
            executor_settings.cartesian_tracking_tolerance_m);
    executor_settings.ik_step_scale =
        mujoco_executor["ik_step_scale"].as<float>(
            executor_settings.ik_step_scale);
    executor_settings.ik_damping =
        mujoco_executor["ik_damping"].as<float>(
            executor_settings.ik_damping);
    executor_settings.ik_iterations =
        mujoco_executor["ik_iterations"].as<int>(
            executor_settings.ik_iterations);
    executor_settings.settle_steps =
        mujoco_executor["settle_steps"].as<int>(
            executor_settings.settle_steps);
    executor_settings.max_motion_steps =
        mujoco_executor["max_motion_steps"].as<int>(
            executor_settings.max_motion_steps);
    ParseStringList(mujoco_executor["joint_names"],
                    &executor_settings.joint_names);
    ParseStringList(mujoco_executor["actuator_names"],
                    &executor_settings.actuator_names);
    ResolveConfigPath(config_dir, &executor_settings.xml_path);
    return config;
}

static bool ReadPose(remote_mujoco::BufferReader* reader, Pose3D* pose) {
    return reader->ReadF32(&pose->x) &&
        reader->ReadF32(&pose->y) &&
        reader->ReadF32(&pose->z) &&
        reader->ReadF32(&pose->qw) &&
        reader->ReadF32(&pose->qx) &&
        reader->ReadF32(&pose->qy) &&
        reader->ReadF32(&pose->qz);
}

static void WritePose(remote_mujoco::BufferWriter* writer,
                    const Pose3D& pose) {
    writer->WriteF32(pose.x);
    writer->WriteF32(pose.y);
    writer->WriteF32(pose.z);
    writer->WriteF32(pose.qw);
    writer->WriteF32(pose.qx);
    writer->WriteF32(pose.qy);
    writer->WriteF32(pose.qz);
}

static bool ReadBool(remote_mujoco::BufferReader* reader, bool* value) {
    std::uint8_t raw = 0;
    if (!reader->ReadU8(&raw)) return false;
    *value = raw != 0;
    return true;
}

static std::vector<std::uint8_t> EncodePosePayload(const Pose3D& pose) {
    remote_mujoco::BufferWriter writer;
    WritePose(&writer, pose);
    return writer.Data();
}

class SimServer {
public:
    SimServer(std::unique_ptr<StereoCamera> camera,
            std::unique_ptr<MujocoGraspExecutor> executor)
        : camera_(std::move(camera)), executor_(std::move(executor)) {}

    void Tick(float dt_s) {
        executor_->Tick(dt_s);
    }

    bool Handle(remote_mujoco::Command command,
                const std::vector<std::uint8_t>& request,
                bool* ok,
                std::int32_t* result,
                std::string* detail,
                std::vector<std::uint8_t>* payload) {
        *ok = true;
        *result = static_cast<std::int32_t>(GraspResult::SUCCESS);
        detail->clear();
        payload->clear();

        if (command == remote_mujoco::Command::GET_FRAME) {
            return HandleGetFrame(ok, result, detail, payload);
        }

        remote_mujoco::BufferReader reader(request);
        GraspResult action_result = GraspResult::SUCCESS;
        switch (command) {
        case remote_mujoco::Command::MOVE_TO_OBSERVE:
            action_result = executor_->MoveToObserve();
            break;
        case remote_mujoco::Command::MOVE_TO_SIDE_OBSERVE:
            action_result = executor_->MoveToSideObserve();
            break;
        case remote_mujoco::Command::MOVE_TO_HOME:
            action_result = executor_->MoveToHome();
            break;
        case remote_mujoco::Command::MOVE_TO_PRE_GRASP: {
            Pose3D pose;
            float yaw = NAN;
            bool top = true;
            if (!ReadPose(&reader, &pose) || !reader.ReadF32(&yaw) ||
                !ReadBool(&reader, &top)) {
                return Reject(ok, result, detail, "invalid pre-grasp payload");
            }
            action_result = executor_->MoveToPreGrasp(pose, yaw, top);
            break;
        }
        case remote_mujoco::Command::OPEN_GRIPPER: {
            float opening = NAN;
            if (!reader.ReadF32(&opening)) {
                return Reject(ok, result, detail, "invalid gripper payload");
            }
            action_result = executor_->OpenGripperForGrasp(opening);
            break;
        }
        case remote_mujoco::Command::MOVE_TO_GRASP: {
            Pose3D pose;
            float yaw = NAN;
            bool top = true;
            if (!ReadPose(&reader, &pose) || !reader.ReadF32(&yaw) ||
                !ReadBool(&reader, &top)) {
                return Reject(ok, result, detail, "invalid grasp payload");
            }
            action_result = executor_->MoveToGrasp(pose, yaw, top);
            break;
        }
        case remote_mujoco::Command::CLOSE_GRIPPER_AND_CHECK:
            action_result = executor_->CloseGripperAndCheck();
            break;
        case remote_mujoco::Command::LIFT_FROM_GRASP: {
            Pose3D retreat;
            Pose3D lift;
            float yaw = NAN;
            bool top = true;
            if (!ReadPose(&reader, &retreat) || !ReadPose(&reader, &lift) ||
                !reader.ReadF32(&yaw) || !ReadBool(&reader, &top)) {
                return Reject(ok, result, detail, "invalid lift payload");
            }
            action_result = executor_->LiftFromGrasp(retreat, lift, yaw, top);
            break;
        }
        case remote_mujoco::Command::VALIDATE_GRASP_POSES: {
            Pose3D pre;
            Pose3D grasp;
            Pose3D retreat;
            Pose3D lift;
            float clearance = 0.0f;
            float yaw = NAN;
            bool top = true;
            std::int32_t timeout = 0;
            if (!ReadPose(&reader, &pre) || !ReadPose(&reader, &grasp) ||
                !ReadPose(&reader, &retreat) || !ReadPose(&reader, &lift) ||
                !reader.ReadF32(&clearance) || !reader.ReadF32(&yaw) ||
                !ReadBool(&reader, &top) || !reader.ReadI32(&timeout)) {
                return Reject(ok, result, detail, "invalid validate payload");
            }
            action_result = executor_->ValidateGraspPoses(
                pre, grasp, retreat, lift, clearance, yaw, top, timeout,
                detail);
            break;
        }
        case remote_mujoco::Command::SET_SUPPORT_PLANE: {
            SupportPlane plane;
            std::uint8_t valid = 0;
            std::uint8_t bounds_valid = 0;
            if (!reader.ReadF32(&plane.normal_x) ||
                !reader.ReadF32(&plane.normal_y) ||
                !reader.ReadF32(&plane.normal_z) ||
                !reader.ReadF32(&plane.d) ||
                !reader.ReadU8(&valid) ||
                !reader.ReadF32(&plane.min_x) ||
                !reader.ReadF32(&plane.max_x) ||
                !reader.ReadF32(&plane.min_y) ||
                !reader.ReadF32(&plane.max_y) ||
                !reader.ReadU8(&bounds_valid)) {
                return Reject(ok, result, detail, "invalid support plane");
            }
            plane.valid = valid != 0;
            plane.bounds_valid = bounds_valid != 0;
            executor_->SetSupportPlane(plane);
            action_result = GraspResult::SUCCESS;
            break;
        }
        case remote_mujoco::Command::MOVE_TO_PLACE:
            action_result = executor_->MoveToPlace();
            break;
        case remote_mujoco::Command::RELEASE_OBJECT:
            action_result = executor_->ReleaseObject();
            break;
        case remote_mujoco::Command::CLOSE_GRIPPER:
            action_result = executor_->CloseGripper();
            break;
        case remote_mujoco::Command::GET_CURRENT_POSE: {
            Pose3D pose;
            if (!executor_->GetCurrentPose(pose)) {
                const auto diagnostics = executor_->GetDiagnostics();
                return Reject(ok, result, detail, diagnostics.last_detail);
            }
            *payload = EncodePosePayload(pose);
            action_result = GraspResult::SUCCESS;
            break;
        }
        case remote_mujoco::Command::TICK: {
            float dt = 0.0f;
            if (!reader.ReadF32(&dt)) {
                return Reject(ok, result, detail, "invalid tick payload");
            }
            executor_->Tick(dt);
            action_result = GraspResult::SUCCESS;
            break;
        }
        case remote_mujoco::Command::SET_TARGET_LABEL: {
            std::string target_label;
            if (!reader.ReadString(&target_label)) {
                return Reject(ok, result, detail, "invalid target payload");
            }
            executor_->SetTargetLabel(target_label);
            action_result = GraspResult::SUCCESS;
            break;
        }
        case remote_mujoco::Command::RESET_SCENE:
            if (!request.empty()) {
                return Reject(ok, result, detail,
                            "reset scene payload must be empty");
            }
            executor_->ResetScene();
            action_result = GraspResult::SUCCESS;
            break;
        case remote_mujoco::Command::EMERGENCY_STOP:
            executor_->EmergencyStop();
            action_result = executor_->GetDiagnostics().last_result;
            break;
        default:
            return Reject(ok, result, detail, "unsupported command");
        }

        const auto diagnostics = executor_->GetDiagnostics();
        *result = static_cast<std::int32_t>(action_result);
        if (detail->empty()) *detail = diagnostics.last_detail;
        return true;
    }

private:
    bool HandleGetFrame(bool* ok,
                        std::int32_t* result,
                        std::string* detail,
                        std::vector<std::uint8_t>* payload) {
        cv::Mat color;
        cv::Mat depth;
        if (!camera_->GetFrames(color, depth) ||
            color.empty() || depth.empty() ||
            color.type() != CV_8UC3 || depth.type() != CV_16UC1 ||
            color.size() != depth.size()) {
            return Reject(ok, result, detail, "failed to render frame");
        }
        remote_mujoco::FramePacket frame;
        frame.frame_id = camera_->LastFrameId();
        frame.width = color.cols;
        frame.height = color.rows;
        if (!camera_->GetIntrinsics(
                &frame.fx, &frame.fy, &frame.cx, &frame.cy)) {
            return Reject(ok, result, detail, "camera intrinsics unavailable");
        }
        const cv::Mat color_contiguous =
            color.isContinuous() ? color : color.clone();
        const cv::Mat depth_contiguous =
            depth.isContinuous() ? depth : depth.clone();
        const size_t color_size =
            color_contiguous.total() * color_contiguous.elemSize();
        const size_t depth_size =
            depth_contiguous.total() * depth_contiguous.elemSize();
        frame.color_bgr.assign(
            color_contiguous.data, color_contiguous.data + color_size);
        frame.depth_u16.assign(
            depth_contiguous.data, depth_contiguous.data + depth_size);
        *payload = remote_mujoco::EncodeFramePacket(frame);
        if (payload->empty()) {
            return Reject(ok, result, detail, "rendered frame is too large");
        }
        *result = static_cast<std::int32_t>(GraspResult::SUCCESS);
        return true;
    }

    bool Reject(bool* ok,
                std::int32_t* result,
                std::string* detail,
                const std::string& message) {
        *ok = false;
        *result = static_cast<std::int32_t>(GraspResult::MOVE_FAILED);
        *detail = message;
        return true;
    }

    std::unique_ptr<StereoCamera> camera_;
    std::unique_ptr<MujocoGraspExecutor> executor_;
};

class MujocoViewer {
public:
    ~MujocoViewer() {
        if (context_ready_) {
            mjr_freeContext(&context_);
            mjv_freeScene(&scene_);
        }
        if (window_ != nullptr) {
            glfwDestroyWindow(window_);
        }
    }

    bool Init(const std::string& xml_path,
            const std::string& camera_name,
            std::string* error) {
        simulation_ = MujocoSimulation::Get(xml_path, error);
        if (!simulation_) return false;

        fixed_camera_id_ = mj_name2id(
            simulation_->model(), mjOBJ_CAMERA, camera_name.c_str());
        if (fixed_camera_id_ < 0) {
            if (error) *error = "viewer inset camera not found: " + camera_name;
            return false;
        }

        if (!glfwInit()) {
            if (error) *error = "glfwInit failed";
            return false;
        }
        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
        window_ = glfwCreateWindow(
            1280, 720, "perceptive_grasp mujoco simulation",
            nullptr, nullptr);
        if (window_ == nullptr) {
            if (error) *error = "viewer window creation failed";
            return false;
        }
        glfwSetWindowUserPointer(window_, this);
        glfwSetCursorPosCallback(window_, &MujocoViewer::CursorPositionCallback);
        glfwSetScrollCallback(window_, &MujocoViewer::ScrollCallback);
        glfwShowWindow(window_);
        glfwFocusWindow(window_);

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&option_);
        mjv_defaultScene(&scene_);
        mjr_defaultContext(&context_);
        {
            std::lock_guard<std::mutex> lock(simulation_->mutex());
            mjv_makeScene(simulation_->model(), &scene_, 4000);
            mjr_makeContext(
                simulation_->model(), &context_, mjFONTSCALE_100);
            camera_.type = mjCAMERA_FREE;
            camera_.azimuth = 135.0;
            camera_.elevation = -32.0;
            camera_.distance = 1.7;
            camera_.lookat[0] = 0.8;
            camera_.lookat[1] = 0.6;
            camera_.lookat[2] = 0.9;
            fixed_camera_.type = mjCAMERA_FIXED;
            fixed_camera_.fixedcamid = fixed_camera_id_;
        }
        context_ready_ = true;
        return true;
    }

    bool ShouldClose() const {
        return window_ == nullptr || glfwWindowShouldClose(window_);
    }

    void Render() {
        if (!context_ready_ || window_ == nullptr) return;
        glfwMakeContextCurrent(window_);
        glfwPollEvents();
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window_, &width, &height);
        if (width <= 0 || height <= 0) return;
        mjrRect viewport = {0, 0, width, height};
        {
            std::lock_guard<std::mutex> lock(simulation_->mutex());
            mjv_updateScene(
                simulation_->model(), simulation_->data(), &option_, nullptr,
                &camera_, mjCAT_ALL, &scene_);
            mjr_render(viewport, &scene_, &context_);
            RenderCameraInset(width, height);
        }
        glfwSwapBuffers(window_);
    }

private:
    static void CursorPositionCallback(GLFWwindow* window,
            double xpos,
            double ypos) {
        auto* self = static_cast<MujocoViewer*>(
            glfwGetWindowUserPointer(window));
        if (self) self->OnMouseMove(xpos, ypos);
    }

    static void ScrollCallback(GLFWwindow* window,
            double,
            double yoffset) {
        auto* self = static_cast<MujocoViewer*>(
            glfwGetWindowUserPointer(window));
        if (self) self->OnScroll(yoffset);
    }

    void OnMouseMove(double xpos, double ypos) {
        if (!context_ready_ || window_ == nullptr || !simulation_) return;
        if (!has_last_mouse_) {
            last_mouse_x_ = xpos;
            last_mouse_y_ = ypos;
            has_last_mouse_ = true;
            return;
        }

        const double dx = xpos - last_mouse_x_;
        const double dy = ypos - last_mouse_y_;
        last_mouse_x_ = xpos;
        last_mouse_y_ = ypos;

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window_, &width, &height);
        if (height <= 0) return;

        const bool left =
            glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        const bool right =
            glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        const bool middle =
            glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        if (!left && !right && !middle) return;

        int action = mjMOUSE_ROTATE_H;
        if (right) {
            action = mjMOUSE_MOVE_H;
        } else if (middle) {
            action = mjMOUSE_ZOOM;
        } else if (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) {
            action = mjMOUSE_ROTATE_V;
        }

        std::lock_guard<std::mutex> lock(simulation_->mutex());
        mjv_moveCamera(
            simulation_->model(), action,
            static_cast<mjtNum>(dx / height),
            static_cast<mjtNum>(dy / height),
            &scene_, &camera_);
    }

    void OnScroll(double yoffset) {
        if (!context_ready_ || !simulation_) return;
        std::lock_guard<std::mutex> lock(simulation_->mutex());
        mjv_moveCamera(
            simulation_->model(), mjMOUSE_ZOOM,
            0.0, static_cast<mjtNum>(-0.05 * yoffset),
            &scene_, &camera_);
    }

    void RenderCameraInset(int width, int height) {
        const int inset_width = std::max(220, width / 4);
        const int inset_height = std::max(160, height / 4);
        const int margin = 18;
        const mjrRect inset = {
            width - inset_width - margin,
            height - inset_height - margin,
            inset_width,
            inset_height,
        };
        const mjrRect border = {
            inset.left - 2,
            inset.bottom - 2,
            inset.width + 4,
            inset.height + 4,
        };
        mjr_rectangle(border, 0.05f, 0.08f, 0.10f, 0.90f);
        mjv_updateScene(
            simulation_->model(), simulation_->data(), &option_, nullptr,
            &fixed_camera_, mjCAT_ALL, &scene_);
        mjr_render(inset, &scene_, &context_);
        mjr_overlay(
            mjFONT_NORMAL, mjGRID_TOPLEFT, inset,
            "camera", "", &context_);
    }

    std::shared_ptr<MujocoSimulation> simulation_;
    GLFWwindow* window_ = nullptr;
    bool context_ready_ = false;
    bool has_last_mouse_ = false;
    double last_mouse_x_ = 0.0;
    double last_mouse_y_ = 0.0;
    int fixed_camera_id_ = -1;
    mjvCamera camera_;
    mjvCamera fixed_camera_;
    mjvOption option_;
    mjvScene scene_;
    mjrContext context_;
};

static int CreateListenSocket(const std::string& host, int port) {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        throw std::runtime_error(
            "socket failed: " + std::string(std::strerror(errno)));
    }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
        close(fd);
        throw std::runtime_error("invalid listen address: " + host);
    }
    if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        const std::string error = std::strerror(errno);
        close(fd);
        throw std::runtime_error("bind failed: " + error);
    }
    if (listen(fd, 8) != 0) {
        const std::string error = std::strerror(errno);
        close(fd);
        throw std::runtime_error("listen failed: " + error);
    }
    return fd;
}

static void PrintUsage(const char* argv0) {
    std::cout << "Usage: " << argv0
            << " --config <yaml> [--listen 127.0.0.1] [--port 9090]"
            << " [--viewer]\n";
}

}  // namespace
}  // namespace perceptive_grasp

int main(int argc, char* argv[]) {
    using perceptive_grasp::ServerConfig;
    ServerConfig config;
    bool listen_overridden = false;
    bool port_overridden = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--help") || (arg == "-h")) {
            perceptive_grasp::PrintUsage(argv[0]);
            return 0;
        } else if (arg == "--config" && i + 1 < argc) {
            config.config_path = argv[++i];
        } else if (arg == "--listen" && i + 1 < argc) {
            config.listen_host = argv[++i];
            listen_overridden = true;
        } else if (arg == "--port" && i + 1 < argc) {
            config.port = std::atoi(argv[++i]);
            port_overridden = true;
        } else if (arg == "--viewer") {
            config.viewer = true;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            perceptive_grasp::PrintUsage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, perceptive_grasp::HandleSignal);
    signal(SIGTERM, perceptive_grasp::HandleSignal);

    try {
        const std::string cli_config_path = config.config_path;
        const std::string cli_listen_host = config.listen_host;
        const int cli_port = config.port;
        const bool cli_viewer = config.viewer;
        config = perceptive_grasp::LoadConfig(cli_config_path);
        if (listen_overridden) config.listen_host = cli_listen_host;
        if (port_overridden) config.port = cli_port;
        config.viewer = cli_viewer;
    } catch (const std::exception& error) {
        std::cerr << "[MujocoSimServer] config error: "
                << error.what() << std::endl;
        return 1;
    }

    std::unique_ptr<perceptive_grasp::StereoCamera> camera =
        perceptive_grasp::CreateMujocoStereoCamera(config.camera);
    if (!camera || !camera->Init()) {
        std::cerr << "[MujocoSimServer] failed to initialize camera"
                << std::endl;
        return 1;
    }
    auto executor =
        std::make_unique<perceptive_grasp::MujocoGraspExecutor>(
            config.executor);
    if (!executor->Init()) {
        std::cerr << "[MujocoSimServer] failed to initialize executor"
                << std::endl;
        return 1;
    }

    std::unique_ptr<perceptive_grasp::MujocoViewer> viewer;
    if (config.viewer) {
        viewer = std::make_unique<perceptive_grasp::MujocoViewer>();
        std::string error;
        if (!viewer->Init(
                config.executor.mujoco.xml_path,
                config.camera.mujoco.camera_name,
                &error)) {
            std::cerr << "[MujocoSimServer] viewer init failed: "
                    << error << std::endl;
            return 1;
        }
        std::cout << "[MujocoSimServer] Viewer enabled: "
                << "perceptive_grasp mujoco simulation" << std::endl;
        std::cout << "[MujocoSimServer] Viewer controls: "
                << "left-drag rotate, shift+left-drag pitch, "
                << "right-drag pan, middle-drag/scroll zoom; "
                << "inset camera=" << config.camera.mujoco.camera_name
                << std::endl;
        auto* viewer_ptr = viewer.get();
        executor->SetVisualStepCallback([viewer_ptr]() {
            if (viewer_ptr && !viewer_ptr->ShouldClose()) {
                viewer_ptr->Render();
            }
        });
    }
    perceptive_grasp::SimServer server(std::move(camera), std::move(executor));

    int listen_fd = -1;
    try {
        listen_fd = perceptive_grasp::CreateListenSocket(
            config.listen_host, config.port);
    } catch (const std::exception& error) {
        std::cerr << "[MujocoSimServer] " << error.what() << std::endl;
        return 1;
    }

    std::cout << "[MujocoSimServer] Listening on "
            << config.listen_host << ":" << config.port << std::endl;
    auto last_tick = std::chrono::steady_clock::now();
    while (!perceptive_grasp::g_stop.load()) {
        const auto now = std::chrono::steady_clock::now();
        const float elapsed_s = std::chrono::duration<float>(
            now - last_tick).count();
        last_tick = now;
        server.Tick(std::clamp(elapsed_s, 0.0f, 0.05f));
        if (viewer) {
            if (viewer->ShouldClose()) {
                perceptive_grasp::g_stop.store(true);
                break;
            }
            viewer->Render();
        }
        pollfd pfd{};
        pfd.fd = listen_fd;
        pfd.events = POLLIN;
        const int poll_result = poll(&pfd, 1, 16);
        if (poll_result == 0) continue;
        if (poll_result < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[MujocoSimServer] poll failed: "
                    << std::strerror(errno) << std::endl;
            continue;
        }
        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        const int client_fd = accept(
            listen_fd, reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[MujocoSimServer] accept failed: "
                    << std::strerror(errno) << std::endl;
            continue;
        }

        perceptive_grasp::remote_mujoco::Command command;
        std::vector<std::uint8_t> request;
        std::string error;
        bool ok = false;
        std::int32_t result = 0;
        std::string detail;
        std::vector<std::uint8_t> payload;
        if (perceptive_grasp::remote_mujoco::ReadRequest(
                client_fd, &command, &request, &error)) {
            server.Handle(command, request, &ok, &result, &detail, &payload);
        } else {
            detail = error;
            result = static_cast<std::int32_t>(
                perceptive_grasp::GraspResult::MOVE_FAILED);
        }
        perceptive_grasp::remote_mujoco::SendResponse(
            client_fd, ok, result, detail, payload, &error);
        close(client_fd);
    }
    close(listen_fd);
    std::cout << "[MujocoSimServer] Stopped" << std::endl;
    return 0;
}
