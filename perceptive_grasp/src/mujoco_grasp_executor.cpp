/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_grasp_executor.cpp
 * @brief MuJoCo-backed grasp executor for PC simulation.
 */

#include "mujoco_grasp_executor.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mujoco_simulation.h"

namespace perceptive_grasp {
namespace {

constexpr double kMinFiniteDepth = 1e-9;
constexpr double kMujocoDropTcpClearance = 0.24;
constexpr double kMujocoPlaceTransitClearance = 0.20;
constexpr double kMujocoDropBoundaryClearance = 0.040;
constexpr double kMujocoPlacementSettleMargin = 0.025;
constexpr int kVisualFrameInterval = 16;
constexpr int kVisualFrameSleepMs = 24;
constexpr int kMujocoPlaceCorrectionAttempts = 4;

void QuatConjugate(const mjtNum q[4], mjtNum out[4]) {
    out[0] = q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

void QuatMultiply(const mjtNum a[4], const mjtNum b[4], mjtNum out[4]) {
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void RotateVectorByQuat(const mjtNum q[4], const mjtNum v[3], mjtNum out[3]) {
    const mjtNum vec_quat[4] = {0.0, v[0], v[1], v[2]};
    mjtNum q_conj[4] = {};
    mjtNum tmp[4] = {};
    mjtNum rotated[4] = {};
    QuatConjugate(q, q_conj);
    QuatMultiply(q, vec_quat, tmp);
    QuatMultiply(tmp, q_conj, rotated);
    out[0] = rotated[1];
    out[1] = rotated[2];
    out[2] = rotated[3];
}

bool Solve3x3(double a[3][3], const double b[3], double x[3]) {
    double aug[3][4] = {
        {a[0][0], a[0][1], a[0][2], b[0]},
        {a[1][0], a[1][1], a[1][2], b[1]},
        {a[2][0], a[2][1], a[2][2], b[2]},
    };
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 3; ++row) {
            if (std::abs(aug[row][col]) > std::abs(aug[pivot][col])) {
                pivot = row;
            }
        }
        if (std::abs(aug[pivot][col]) < 1e-12) {
            return false;
        }
        if (pivot != col) {
            for (int k = col; k < 4; ++k) {
                std::swap(aug[col][k], aug[pivot][k]);
            }
        }
        const double div = aug[col][col];
        for (int k = col; k < 4; ++k) {
            aug[col][k] /= div;
        }
        for (int row = 0; row < 3; ++row) {
            if (row == col) continue;
            const double factor = aug[row][col];
            for (int k = col; k < 4; ++k) {
                aug[row][k] -= factor * aug[col][k];
            }
        }
    }
    x[0] = aug[0][3];
    x[1] = aug[1][3];
    x[2] = aug[2][3];
    return true;
}

bool SolveLinearSystem(std::vector<double> a,
                    std::vector<double> b,
                    std::vector<double>* x) {
    const size_t n = b.size();
    if (n == 0 || a.size() != n * n || x == nullptr) {
        return false;
    }

    for (size_t col = 0; col < n; ++col) {
        size_t pivot = col;
        for (size_t row = col + 1; row < n; ++row) {
            if (std::abs(a[row * n + col]) >
                std::abs(a[pivot * n + col])) {
                pivot = row;
            }
        }
        if (std::abs(a[pivot * n + col]) < 1e-12) {
            return false;
        }
        if (pivot != col) {
            for (size_t k = col; k < n; ++k) {
                std::swap(a[col * n + k], a[pivot * n + k]);
            }
            std::swap(b[col], b[pivot]);
        }

        const double div = a[col * n + col];
        for (size_t k = col; k < n; ++k) {
            a[col * n + k] /= div;
        }
        b[col] /= div;

        for (size_t row = 0; row < n; ++row) {
            if (row == col) continue;
            const double factor = a[row * n + col];
            for (size_t k = col; k < n; ++k) {
                a[row * n + k] -= factor * a[col * n + k];
            }
            b[row] -= factor * b[col];
        }
    }

    *x = std::move(b);
    return true;
}

bool IsFinitePose(const Pose3D& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
            std::isfinite(pose.z);
}

bool PoseHasFiniteQuaternion(const Pose3D& pose) {
    return std::isfinite(pose.qw) && std::isfinite(pose.qx) &&
        std::isfinite(pose.qy) && std::isfinite(pose.qz);
}

bool NormalizeQuaternion(const Pose3D& pose, mjtNum q[4]) {
    if (!PoseHasFiniteQuaternion(pose)) {
        return false;
    }
    const double norm = std::sqrt(
        static_cast<double>(pose.qw) * pose.qw +
        static_cast<double>(pose.qx) * pose.qx +
        static_cast<double>(pose.qy) * pose.qy +
        static_cast<double>(pose.qz) * pose.qz);
    if (norm < 1e-9) {
        return false;
    }
    q[0] = pose.qw / norm;
    q[1] = pose.qx / norm;
    q[2] = pose.qy / norm;
    q[3] = pose.qz / norm;
    return true;
}

Pose3D AlignOpeningAxisYaw(const Pose3D& pose, float opening_yaw_rad) {
    if (!std::isfinite(opening_yaw_rad)) {
        return pose;
    }
    mjtNum pose_quat[4] = {};
    if (!NormalizeQuaternion(pose, pose_quat)) {
        return pose;
    }

    const mjtNum local_opening_axis[3] = {0.0, 1.0, 0.0};
    mjtNum world_opening_axis[3] = {};
    RotateVectorByQuat(
        pose_quat, local_opening_axis, world_opening_axis);
    const double projected_norm = std::hypot(
        world_opening_axis[0], world_opening_axis[1]);
    if (projected_norm < 1e-6) {
        return pose;
    }

    const double current_yaw = std::atan2(
        world_opening_axis[1], world_opening_axis[0]);
    // Grasp candidates use the Linksee wrist convention: yaw describes the
    // finger direction, which is perpendicular to the Robotiq opening axis.
    // Convert that convention locally so the simulation backend does not
    // change the established real-hardware planning behavior.
    const double desired_opening_yaw =
        static_cast<double>(opening_yaw_rad) + 0.5 * M_PI;
    const double yaw_delta = std::remainder(
        desired_opening_yaw - current_yaw,
        2.0 * M_PI);
    const mjtNum yaw_quat[4] = {
        std::cos(0.5 * yaw_delta), 0.0, 0.0,
        std::sin(0.5 * yaw_delta),
    };
    mjtNum aligned_quat[4] = {};
    QuatMultiply(yaw_quat, pose_quat, aligned_quat);

    Pose3D aligned = pose;
    aligned.qw = static_cast<float>(aligned_quat[0]);
    aligned.qx = static_cast<float>(aligned_quat[1]);
    aligned.qy = static_cast<float>(aligned_quat[2]);
    aligned.qz = static_cast<float>(aligned_quat[3]);
    return aligned;
}

std::string Lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                [](unsigned char ch) {
                    return static_cast<char>(std::tolower(ch));
                });
    return value;
}

bool ContainsNameToken(const std::string& name, const std::string& token) {
    return Lowercase(name).find(token) != std::string::npos;
}

struct GripperObjectContact {
    bool left_pad = false;
    bool right_pad = false;
};

}  // namespace

struct MujocoGraspExecutor::Impl {
    std::shared_ptr<MujocoSimulation> simulation;
    mjModel* model = nullptr;
    mjData* data = nullptr;
    int ee_site_id = -1;
    int hold_site_id = -1;
    int gripper_actuator_id = -1;
    int gripper_free_joint_id = -1;
    int gripper_free_qpos_addr = -1;
    int gripper_free_qvel_addr = -1;
    int flange_body_id = -1;
    int attach_equality_id = -1;
    int grasp_weld_equality_id = -1;
    int held_body_id = -1;
    int held_joint_id = -1;
    int held_qpos_addr = -1;
    int held_qvel_addr = -1;
    int robot_root_body_id = -1;
    int gripper_root_body_id = -1;
    std::string target_label;
    std::function<void()> visual_step_callback;
    std::array<mjtNum, 3> held_rel_pos = {0.0, 0.0, 0.0};
    std::array<mjtNum, 4> held_rel_quat = {1.0, 0.0, 0.0, 0.0};
    std::vector<int> graspable_body_ids;
    std::vector<int> joint_ids;
    std::vector<int> qpos_addr;
    std::vector<int> qvel_addr;
    std::vector<int> actuator_ids;
    std::vector<std::vector<float>> validated_pre_grasp_path;
    std::vector<std::vector<float>> validated_grasp_path;
    std::vector<std::vector<float>> validated_lift_path;
    Pose3D last_pre_grasp_pose{};
    bool last_pre_grasp_valid = false;

    bool Load(const ExecutorConfig& config, std::string* error) {
        simulation = MujocoSimulation::Get(config.mujoco.xml_path, error);
        if (!simulation) {
            return false;
        }
        model = simulation->model();
        data = simulation->data();

        ee_site_id = mj_name2id(model, mjOBJ_SITE,
                                config.mujoco.end_effector_site.c_str());
        if (ee_site_id < 0) {
            if (error) {
                *error = "end effector site not found: " +
                    config.mujoco.end_effector_site;
            }
            return false;
        }
        hold_site_id = mj_name2id(model, mjOBJ_SITE, "pinch");
        if (hold_site_id < 0) {
            hold_site_id = ee_site_id;
        }

        joint_ids.clear();
        qpos_addr.clear();
        qvel_addr.clear();
        for (const std::string& name : config.mujoco.joint_names) {
            const int id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
            if (id < 0) {
                if (error) *error = "joint not found: " + name;
                return false;
            }
            if (model->jnt_type[id] != mjJNT_HINGE &&
                model->jnt_type[id] != mjJNT_SLIDE) {
                if (error) *error = "joint is not 1-DoF: " + name;
                return false;
            }
            joint_ids.push_back(id);
            qpos_addr.push_back(model->jnt_qposadr[id]);
            qvel_addr.push_back(model->jnt_dofadr[id]);
        }

        actuator_ids.clear();
        for (const std::string& name : config.mujoco.actuator_names) {
            const int id = mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
            if (id < 0) {
                if (error) *error = "actuator not found: " + name;
                return false;
            }
            actuator_ids.push_back(id);
        }
        if (actuator_ids.size() != joint_ids.size()) {
            if (error) {
                *error = "mujoco actuator count must match joint count";
            }
            return false;
        }
        const double stiffness_scale = std::max(
            0.1, static_cast<double>(config.mujoco.arm_stiffness_scale));
        const double damping_scale = std::sqrt(stiffness_scale);
        for (int actuator_id : actuator_ids) {
            model->actuator_gainprm[
                actuator_id * mjNGAIN] *= stiffness_scale;
            model->actuator_biasprm[
                actuator_id * mjNBIAS + 1] *= stiffness_scale;
            model->actuator_biasprm[
                actuator_id * mjNBIAS + 2] *= damping_scale;
            if (model->actuator_forcelimited[actuator_id]) {
                model->actuator_forcerange[2 * actuator_id] *= damping_scale;
                model->actuator_forcerange[2 * actuator_id + 1] *=
                    damping_scale;
            }
        }

        gripper_actuator_id = mj_name2id(
            model, mjOBJ_ACTUATOR, config.mujoco.gripper_actuator.c_str());
        if (gripper_actuator_id < 0) {
            if (error) {
                *error = "gripper actuator not found: " +
                    config.mujoco.gripper_actuator;
            }
            return false;
        }

        attach_equality_id = mj_name2id(model, mjOBJ_EQUALITY, "attach");
        grasp_weld_equality_id =
            mj_name2id(model, mjOBJ_EQUALITY, "grasp_weld");
        gripper_free_joint_id = mj_name2id(model, mjOBJ_JOINT, "2f85");
        flange_body_id = mj_name2id(model, mjOBJ_BODY, "flange");
        robot_root_body_id = mj_name2id(
            model, mjOBJ_BODY, config.mujoco.robot_root_body.c_str());
        gripper_root_body_id = mj_name2id(
            model, mjOBJ_BODY, config.mujoco.gripper_root_body.c_str());
        if (robot_root_body_id < 0) {
            if (error) {
                *error = "robot root body not found: " +
                    config.mujoco.robot_root_body;
            }
            return false;
        }
        if (gripper_root_body_id < 0) {
            if (error) {
                *error = "gripper root body not found: " +
                    config.mujoco.gripper_root_body;
            }
            return false;
        }
        if (config.mujoco.gravity_compensation) {
            for (int body_id = 1; body_id < model->nbody; ++body_id) {
                if (IsDescendantBody(body_id, robot_root_body_id) ||
                    IsDescendantBody(body_id, gripper_root_body_id)) {
                    model->body_gravcomp[body_id] = 1.0;
                }
            }
        }
        if (gripper_free_joint_id >= 0) {
            gripper_free_qpos_addr = model->jnt_qposadr[gripper_free_joint_id];
            gripper_free_qvel_addr = model->jnt_dofadr[gripper_free_joint_id];
        }
        BuildGraspableBodyList();

        mj_forward(model, data);
        SyncFreeGripperToFlange();
        mj_forward(model, data);
        return true;
    }

    void ResetTaskScene(const ExecutorConfig& config) {
        held_body_id = -1;
        held_joint_id = -1;
        held_qpos_addr = -1;
        held_qvel_addr = -1;
        held_rel_pos = {0.0, 0.0, 0.0};
        held_rel_quat = {1.0, 0.0, 0.0, 0.0};
        if (grasp_weld_equality_id >= 0) {
            data->eq_active[grasp_weld_equality_id] = 0;
        }
        if (simulation) {
            simulation->Reset();
        }
        ForwardWithAttachments();
        if (config.home_joints.size() == joint_ids.size()) {
            SetJointPositionsDirect(config.home_joints);
        }
        if (gripper_actuator_id >= 0) {
            data->ctrl[gripper_actuator_id] =
                config.mujoco.gripper_open_ctrl;
        }
        ForwardWithAttachments();
    }

    void BuildGraspableBodyList() {
        graspable_body_ids.clear();
        for (int body_id = 0; body_id < model->nbody; ++body_id) {
            if (model->body_jntnum[body_id] != 1) {
                continue;
            }
            const int joint_id = model->body_jntadr[body_id];
            if (model->jnt_type[joint_id] != mjJNT_FREE ||
                joint_id == gripper_free_joint_id) {
                continue;
            }
            const char* name = mj_id2name(model, mjOBJ_BODY, body_id);
            if (!name || std::string(name) == "mocap") {
                continue;
            }
            graspable_body_ids.push_back(body_id);
        }
    }

    void SyncFreeGripperToFlange() {
        if (attach_equality_id >= 0) {
            data->eq_active[attach_equality_id] = 0;
        }
        if (gripper_free_qpos_addr < 0 || gripper_free_qvel_addr < 0 ||
            flange_body_id < 0) {
            return;
        }
        for (int i = 0; i < 3; ++i) {
            data->qpos[gripper_free_qpos_addr + i] =
                data->xpos[3 * flange_body_id + i];
            data->qvel[gripper_free_qvel_addr + i] = 0.0;
        }
        for (int i = 0; i < 4; ++i) {
            data->qpos[gripper_free_qpos_addr + 3 + i] =
                data->xquat[4 * flange_body_id + i];
        }
        for (int i = 3; i < 6; ++i) {
            data->qvel[gripper_free_qvel_addr + i] = 0.0;
        }
    }

    void ForwardWithAttachments() {
        mj_forward(model, data);
        SyncFreeGripperToFlange();
        mj_forward(model, data);
    }

    void MaybeRenderVisualStep(int step) {
        if (!visual_step_callback || step % kVisualFrameInterval != 0) {
            return;
        }
        visual_step_callback();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kVisualFrameSleepMs));
    }

    bool IsDescendantBody(int body_id, int root_id) const {
        if (body_id < 0 || root_id < 0) {
            return false;
        }
        for (int current = body_id; current > 0;
                current = model->body_parentid[current]) {
            if (current == root_id) {
                return true;
            }
        }
        return false;
    }

    bool IsRobotBody(int body_id) const {
        return IsDescendantBody(body_id, robot_root_body_id) ||
            IsDescendantBody(body_id, gripper_root_body_id);
    }

    int GraspableRoot(int body_id) const {
        for (int root_id : graspable_body_ids) {
            if (IsDescendantBody(body_id, root_id)) return root_id;
        }
        return -1;
    }

    bool HasUnsafeCollision(bool allow_target_contact,
                            std::string* detail) const {
        const auto geom_label = [this](int geom_id) {
            const char* geom_name = mj_id2name(model, mjOBJ_GEOM, geom_id);
            if (geom_name && geom_name[0] != '\0') {
                return std::string(geom_name);
            }
            const int body_id = model->geom_bodyid[geom_id];
            const char* body_name = mj_id2name(model, mjOBJ_BODY, body_id);
            if (body_name && body_name[0] != '\0') {
                return std::string("body:") + body_name;
            }
            return std::string("geom:") + std::to_string(geom_id);
        };
        for (int i = 0; i < data->ncon; ++i) {
            const mjContact& contact = data->contact[i];
            const int body1 = model->geom_bodyid[contact.geom1];
            const int body2 = model->geom_bodyid[contact.geom2];
            const bool robot1 = IsRobotBody(body1);
            const bool robot2 = IsRobotBody(body2);
            if (!robot1 && !robot2) continue;

            if (robot1 && robot2) {
                const bool adjacent_links =
                    model->body_parentid[body1] == body2 ||
                    model->body_parentid[body2] == body1;
                const bool internal_gripper =
                    IsDescendantBody(body1, gripper_root_body_id) &&
                    IsDescendantBody(body2, gripper_root_body_id);
                if (adjacent_links || internal_gripper) continue;
            } else {
                const int robot_body = robot1 ? body1 : body2;
                const int other_body = robot1 ? body2 : body1;
                const int object_root = GraspableRoot(other_body);
                if (allow_target_contact && object_root >= 0 &&
                    IsDescendantBody(robot_body, gripper_root_body_id) &&
                    BodyMatchesTarget(object_root)) {
                    continue;
                }
            }
            if (detail) {
                *detail = std::string("unsafe MuJoCo contact: ") +
                    geom_label(contact.geom1) + " vs " +
                    geom_label(contact.geom2);
            }
            return true;
        }
        return false;
    }

    GripperObjectContact GetGripperObjectContact(int object_body_id) const {
        GripperObjectContact result;
        if (object_body_id < 0) {
            return result;
        }
        for (int i = 0; i < data->ncon; ++i) {
            const mjContact& contact = data->contact[i];
            const int body1 = model->geom_bodyid[contact.geom1];
            const int body2 = model->geom_bodyid[contact.geom2];
            int gripper_geom = -1;
            if (IsDescendantBody(body1, gripper_root_body_id) &&
                body2 == object_body_id) {
                gripper_geom = contact.geom1;
            } else if (IsDescendantBody(body2, gripper_root_body_id) &&
                    body1 == object_body_id) {
                gripper_geom = contact.geom2;
            }
            if (gripper_geom < 0) {
                continue;
            }
            const char* geom_name =
                mj_id2name(model, mjOBJ_GEOM, gripper_geom);
            const int gripper_body = model->geom_bodyid[gripper_geom];
            const char* body_name =
                mj_id2name(model, mjOBJ_BODY, gripper_body);
            const std::string geom = geom_name ? Lowercase(geom_name) : "";
            const std::string body = body_name ? Lowercase(body_name) : "";
            const bool pad = ContainsNameToken(geom, "pad") ||
                ContainsNameToken(body, "pad");
            if (!pad) {
                continue;
            }
            if (ContainsNameToken(geom, "left") ||
                ContainsNameToken(body, "left")) {
                result.left_pad = true;
            }
            if (ContainsNameToken(geom, "right") ||
                ContainsNameToken(body, "right")) {
                result.right_pad = true;
            }
        }
        return result;
    }

    bool HasOpposingPadObjectContact(int object_body_id) const {
        const GripperObjectContact contact =
            GetGripperObjectContact(object_body_id);
        return contact.left_pad && contact.right_pad;
    }

    bool HasVerifiedPhysicalHold() const {
        if (held_body_id < 0) {
            return false;
        }
        if (grasp_weld_equality_id >= 0 &&
            data->eq_active[grasp_weld_equality_id] != 0) {
            return true;
        }
        return HasOpposingPadObjectContact(held_body_id);
    }

    bool AttachContactObject(std::string* detail) {
        if (hold_site_id < 0 || graspable_body_ids.empty()) {
            if (detail) *detail = "no graspable MuJoCo object";
            return false;
        }
        mj_forward(model, data);
        const mjtNum* site_pos = data->site_xpos + 3 * hold_site_id;
        int best_body_id = -1;
        double best_distance = std::numeric_limits<double>::infinity();
        std::vector<int> candidates;
        for (int body_id : graspable_body_ids) {
            if (BodyMatchesTarget(body_id)) {
                candidates.push_back(body_id);
            }
        }
        if (candidates.empty()) {
            candidates = graspable_body_ids;
        }
        int contact_body_id = -1;
        double contact_distance = std::numeric_limits<double>::infinity();
        for (int body_id : candidates) {
            const mjtNum* body_pos = data->xpos + 3 * body_id;
            const double dx = body_pos[0] - site_pos[0];
            const double dy = body_pos[1] - site_pos[1];
            const double dz = body_pos[2] - site_pos[2];
            const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance < best_distance) {
                best_distance = distance;
                best_body_id = body_id;
            }
            if (HasOpposingPadObjectContact(body_id) &&
                distance < contact_distance) {
                contact_distance = distance;
                contact_body_id = body_id;
            }
        }

        if (contact_body_id < 0) {
            if (detail) {
                const char* name = best_body_id >= 0
                    ? mj_id2name(model, mjOBJ_BODY, best_body_id)
                    : nullptr;
                const GripperObjectContact pad_contact =
                    GetGripperObjectContact(best_body_id);
                const mjtNum* nearest_position = best_body_id >= 0
                    ? data->xpos + 3 * best_body_id : nullptr;
                const int left_pad_body_id = mj_name2id(
                    model, mjOBJ_BODY, "robot_gripper_left_pad");
                const int right_pad_body_id = mj_name2id(
                    model, mjOBJ_BODY, "robot_gripper_right_pad");
                const mjtNum* left_pad_position = left_pad_body_id >= 0
                    ? data->xpos + 3 * left_pad_body_id : nullptr;
                const mjtNum* right_pad_position = right_pad_body_id >= 0
                    ? data->xpos + 3 * right_pad_body_id : nullptr;
                *detail = "gripper did not contact target; nearest_m=" +
                    std::to_string(best_distance) +
                    " target=" + target_label +
                    " nearest=" + (name ? name : "none") +
                    " pad_contact=[left=" +
                    (pad_contact.left_pad ? "true" : "false") +
                    ",right=" +
                    (pad_contact.right_pad ? "true" : "false") + "]" +
                    " tcp=[" + std::to_string(site_pos[0]) + "," +
                    std::to_string(site_pos[1]) + "," +
                    std::to_string(site_pos[2]) + "]" +
                    (nearest_position
                        ? " object=[" +
                            std::to_string(nearest_position[0]) + "," +
                            std::to_string(nearest_position[1]) + "," +
                            std::to_string(nearest_position[2]) + "]"
                        : "") +
                    (left_pad_position
                        ? " left_pad=[" +
                            std::to_string(left_pad_position[0]) + "," +
                            std::to_string(left_pad_position[1]) + "," +
                            std::to_string(left_pad_position[2]) + "]"
                        : "") +
                    (right_pad_position
                        ? " right_pad=[" +
                            std::to_string(right_pad_position[0]) + "," +
                            std::to_string(right_pad_position[1]) + "," +
                            std::to_string(right_pad_position[2]) + "]"
                        : "") +
                    " required_contact=left_pad+right_pad";
            }
            return false;
        }

        best_body_id = contact_body_id;
        best_distance = contact_distance;
        const int joint_id = model->body_jntadr[best_body_id];
        held_body_id = best_body_id;
        held_joint_id = joint_id;
        held_qpos_addr = model->jnt_qposadr[joint_id];
        held_qvel_addr = model->jnt_dofadr[joint_id];

        const mjtNum* body_pos = data->xpos + 3 * best_body_id;
        const mjtNum* body_quat = data->xquat + 4 * best_body_id;
        const mjtNum* gripper_pos =
            data->xpos + 3 * gripper_root_body_id;
        const mjtNum* gripper_quat =
            data->xquat + 4 * gripper_root_body_id;
        mjtNum inv_gripper_quat[4] = {};
        mjtNum delta_pos[3] = {
            body_pos[0] - gripper_pos[0],
            body_pos[1] - gripper_pos[1],
            body_pos[2] - gripper_pos[2],
        };
        QuatConjugate(gripper_quat, inv_gripper_quat);
        RotateVectorByQuat(
            inv_gripper_quat, delta_pos, held_rel_pos.data());
        QuatMultiply(
            inv_gripper_quat, body_quat, held_rel_quat.data());

        if (grasp_weld_equality_id >= 0) {
            model->eq_obj1id[grasp_weld_equality_id] =
                gripper_root_body_id;
            model->eq_obj2id[grasp_weld_equality_id] = held_body_id;
            mjtNum* weld_data = model->eq_data +
                grasp_weld_equality_id * mjNEQDATA;
            std::fill(weld_data, weld_data + 10, 0.0);
            std::copy(held_rel_pos.begin(), held_rel_pos.end(), weld_data + 3);
            std::copy(
                held_rel_quat.begin(), held_rel_quat.end(), weld_data + 6);
            if (weld_data[10] <= 0.0) {
                weld_data[10] = 1.0;
            }
            data->eq_active[grasp_weld_equality_id] = 1;
            mj_forward(model, data);
        }

        if (detail) {
            const char* name = mj_id2name(model, mjOBJ_BODY, held_body_id);
            *detail = std::string("holding ") + (name ? name : "object") +
                " contact=left_pad+right_pad distance_m=" +
                std::to_string(best_distance);
        }
        return true;
    }

    bool CloseGripperOnContact(
        const ExecutorConfig& config, std::string* detail) {
        if (gripper_actuator_id < 0) {
            if (detail) *detail = "missing gripper actuator";
            return false;
        }
        const double start_ctrl = data->ctrl[gripper_actuator_id];
        const int close_steps = std::max(1, config.mujoco.settle_steps);
        for (int step = 1; step <= close_steps; ++step) {
            const double alpha = static_cast<double>(step) /
                static_cast<double>(close_steps);
            data->ctrl[gripper_actuator_id] = start_ctrl + alpha *
                (config.mujoco.gripper_close_ctrl - start_ctrl);
            mj_step(model, data);
            ForwardWithAttachments();
            MaybeRenderVisualStep(step);
            if (AttachContactObject(nullptr)) {
                if (detail) {
                    const char* name =
                        mj_id2name(model, mjOBJ_BODY, held_body_id);
                    *detail = std::string("holding ") +
                        (name ? name : "object") +
                        " after opposing pad contact; close_ctrl=" +
                        std::to_string(
                            data->ctrl[gripper_actuator_id]);
                }
                return true;
            }
        }
        return AttachContactObject(detail);
    }

    void ReleaseHeldObject(int settle_steps) {
        if (held_joint_id >= 0 && held_qvel_addr >= 0 &&
            model->jnt_type[held_joint_id] == mjJNT_FREE) {
            std::fill(
                data->qvel + held_qvel_addr,
                data->qvel + held_qvel_addr + 6, 0.0);
        }
        if (grasp_weld_equality_id >= 0) {
            data->eq_active[grasp_weld_equality_id] = 0;
        }
        mj_forward(model, data);
        held_body_id = -1;
        held_joint_id = -1;
        held_qpos_addr = -1;
        held_qvel_addr = -1;
        const int steps = std::max(0, settle_steps);
        for (int i = 0; i < steps; ++i) {
            mj_step(model, data);
            SyncFreeGripperToFlange();
            mj_forward(model, data);
            MaybeRenderVisualStep(i);
        }
    }

    double BodyHorizontalRadius(int body_id) const {
        double radius = 0.0;
        for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
            if (model->geom_bodyid[geom_id] != body_id ||
                (model->geom_contype[geom_id] == 0 &&
                    model->geom_conaffinity[geom_id] == 0)) {
                continue;
            }
            const mjtNum* local_position = model->geom_pos + 3 * geom_id;
            radius = std::max(radius,
                std::hypot(
                    static_cast<double>(local_position[0]),
                    static_cast<double>(local_position[1])) +
                static_cast<double>(model->geom_rbound[geom_id]));
        }
        return radius;
    }

    bool BodyInsideDropZoneXY(int body_id, std::string* detail) const {
        const int zone_body_id = mj_name2id(model, mjOBJ_BODY, "zone_drop");
        const int zone_geom_id = mj_name2id(model, mjOBJ_GEOM, "zone_drop");
        if (body_id < 0 || zone_body_id < 0 || zone_geom_id < 0) {
            if (detail) *detail = "target body or drop zone not found";
            return false;
        }
        const mjtNum* body_pos = data->xpos + 3 * body_id;
        const mjtNum* zone_pos = data->xpos + 3 * zone_body_id;
        const mjtNum* zone_size = model->geom_size + 3 * zone_geom_id;
        const double radius = BodyHorizontalRadius(body_id);
        const double usable_x = std::max(
            0.0, static_cast<double>(zone_size[0]) - radius -
                kMujocoDropBoundaryClearance);
        const double usable_y = std::max(
            0.0, static_cast<double>(zone_size[1]) - radius -
                kMujocoDropBoundaryClearance);
        const double dx = body_pos[0] - zone_pos[0];
        const double dy = body_pos[1] - zone_pos[1];
        const bool inside = std::abs(dx) <= usable_x &&
            std::abs(dy) <= usable_y;
        if (detail) {
            *detail = "object_offset_to_drop_zone=[" +
                std::to_string(dx) + "," + std::to_string(dy) +
                "] usable_half_size=[" + std::to_string(usable_x) + "," +
                std::to_string(usable_y) + "] object_radius_m=" +
                std::to_string(radius) + " inside_zone_xy=" +
                (inside ? "true" : "false");
        }
        return inside;
    }

    std::string DescribeTargetDropStatus(bool* inside_zone) const {
        if (inside_zone) *inside_zone = false;
        int target_body_id = -1;
        for (int body_id : graspable_body_ids) {
            if (BodyMatchesTarget(body_id)) {
                target_body_id = body_id;
                break;
            }
        }
        if (target_body_id < 0) {
            return "target body not found after release";
        }

        std::string detail;
        const bool inside_xy = BodyInsideDropZoneXY(target_body_id, &detail);
        if (inside_zone) *inside_zone = inside_xy;
        const char* target_name = mj_id2name(model, mjOBJ_BODY, target_body_id);
        return std::string("target=") + (target_name ? target_name : "object") +
            " " + detail;
    }

    bool HeldObjectInsideDropZoneXY(std::string* detail) {
        if (held_body_id < 0) {
            if (detail) *detail = "no held object";
            return false;
        }
        ForwardWithAttachments();
        return BodyInsideDropZoneXY(held_body_id, detail);
    }

    bool BodyMatchesTarget(int body_id) const {
        if (target_label.empty()) {
            return false;
        }
        const char* raw_name = mj_id2name(model, mjOBJ_BODY, body_id);
        if (!raw_name) {
            return false;
        }
        const std::string name = Lowercase(raw_name);
        const std::string target = Lowercase(target_label);
        if (name == target) {
            return true;
        }
        if (target == "banana") return name == "banana";
        if (target == "apple") return name == "apple";
        if (target == "box" || target == "cube") return name == "box";
        if (target == "block" || target == "t_block") return name == "t_block";
        if (target == "cylinder") return name == "cylinder";
        return false;
    }

    bool GetDropZonePose(Pose3D* pose) {
        const int zone_body_id = mj_name2id(model, mjOBJ_BODY, "zone_drop");
        const int zone_geom_id = mj_name2id(model, mjOBJ_GEOM, "zone_drop");
        if (zone_body_id < 0 || zone_geom_id < 0 || pose == nullptr) {
            return false;
        }
        const mjtNum* zone_pos = data->xpos + 3 * zone_body_id;
        const mjtNum* zone_size = model->geom_size + 3 * zone_geom_id;
        Pose3D current;
        if (!GetPose(current)) {
            return false;
        }
        *pose = current;
        float drop_x = static_cast<float>(zone_pos[0]);
        float drop_y = static_cast<float>(zone_pos[1]);
        if (held_body_id >= 0 && hold_site_id >= 0) {
            ForwardWithAttachments();
            const mjtNum* held_pos = data->xpos + 3 * held_body_id;
            const mjtNum* hold_pos = data->site_xpos + 3 * hold_site_id;
            const double held_radius = BodyHorizontalRadius(held_body_id);
            const double usable_x = std::max(
                0.0, static_cast<double>(zone_size[0]) - held_radius -
                    kMujocoDropBoundaryClearance);
            const double usable_y = std::max(
                0.0, static_cast<double>(zone_size[1]) - held_radius -
                    kMujocoDropBoundaryClearance);
            const double placement_x = std::max(
                0.0, usable_x - kMujocoPlacementSettleMargin);
            const double placement_y = std::max(
                0.0, usable_y - kMujocoPlacementSettleMargin);
            double best_x = zone_pos[0];
            double best_y = zone_pos[1];
            double best_clearance = -std::numeric_limits<double>::infinity();
            constexpr int kPlacementColumns = 9;
            constexpr int kPlacementRows = 5;
            for (int row = 0; row < kPlacementRows; ++row) {
                const double row_alpha = static_cast<double>(row) /
                    static_cast<double>(kPlacementRows - 1);
                const double candidate_y = zone_pos[1] - placement_y +
                    2.0 * placement_y * row_alpha;
                for (int column = 0; column < kPlacementColumns; ++column) {
                    const double column_alpha =
                        static_cast<double>(column) /
                        static_cast<double>(kPlacementColumns - 1);
                    const double candidate_x = zone_pos[0] - placement_x +
                        2.0 * placement_x * column_alpha;
                    double minimum_clearance =
                        std::numeric_limits<double>::infinity();
                    bool collision_free = true;
                    bool has_placed_object = false;
                    for (int body_id : graspable_body_ids) {
                        if (body_id == held_body_id) continue;
                        const mjtNum* other_pos =
                            data->xpos + 3 * body_id;
                        const double other_radius =
                            BodyHorizontalRadius(body_id);
                        const bool in_drop_area =
                            std::abs(other_pos[0] - zone_pos[0]) <=
                                zone_size[0] &&
                            std::abs(other_pos[1] - zone_pos[1]) <=
                                zone_size[1];
                        if (!in_drop_area) continue;
                        has_placed_object = true;
                        const double clearance = std::hypot(
                            candidate_x - other_pos[0],
                            candidate_y - other_pos[1]) -
                            held_radius - other_radius - 0.015;
                        minimum_clearance = std::min(
                            minimum_clearance, clearance);
                        if (clearance < 0.0) {
                            collision_free = false;
                            break;
                        }
                    }
                    if (!collision_free) continue;
                    const double center_distance = std::hypot(
                        candidate_x - zone_pos[0],
                        candidate_y - zone_pos[1]);
                    const double score = has_placed_object
                        ? minimum_clearance - 0.01 * center_distance
                        : -center_distance;
                    if (score > best_clearance) {
                        best_clearance = score;
                        best_x = candidate_x;
                        best_y = candidate_y;
                    }
                }
            }
            if (!std::isfinite(best_clearance)) {
                return false;
            }
            drop_x = static_cast<float>(
                best_x - (held_pos[0] - hold_pos[0]));
            drop_y = static_cast<float>(
                best_y - (held_pos[1] - hold_pos[1]));
        }
        pose->x = drop_x;
        pose->y = drop_y;
        pose->z = static_cast<float>(
            zone_pos[2] + kMujocoDropTcpClearance);
        return true;
    }

    std::vector<float> CurrentJoints() const {
        std::vector<float> joints;
        joints.reserve(qpos_addr.size());
        for (int addr : qpos_addr) {
            joints.push_back(static_cast<float>(data->qpos[addr]));
        }
        return joints;
    }

    void SetJointPositionsDirect(const std::vector<float>& joints) {
        const size_t count = std::min(joints.size(), qpos_addr.size());
        for (size_t i = 0; i < count; ++i) {
            data->qpos[qpos_addr[i]] = joints[i];
            data->qvel[qvel_addr[i]] = 0.0;
            data->ctrl[actuator_ids[i]] = joints[i];
        }
        ForwardWithAttachments();
    }

    void StepWithTargets(const std::vector<float>& targets) {
        const size_t count = std::min(targets.size(), actuator_ids.size());
        for (size_t i = 0; i < count; ++i) {
            data->ctrl[actuator_ids[i]] = targets[i];
        }
        mj_step(model, data);
        ForwardWithAttachments();
    }

    void HoldCurrentState() {
        const size_t count = std::min(actuator_ids.size(), qpos_addr.size());
        for (size_t i = 0; i < count; ++i) {
            data->ctrl[actuator_ids[i]] = data->qpos[qpos_addr[i]];
            data->qvel[qvel_addr[i]] = 0.0;
        }
        ForwardWithAttachments();
    }

    double JointError(const std::vector<float>& targets) const {
        double max_error = 0.0;
        const size_t count = std::min(targets.size(), qpos_addr.size());
        for (size_t i = 0; i < count; ++i) {
            max_error = std::max(
                max_error,
                std::abs(static_cast<double>(targets[i]) -
                            data->qpos[qpos_addr[i]]));
        }
        return max_error;
    }

    std::string JointErrorDetail(const std::vector<float>& targets) const {
        std::string detail;
        const size_t count = std::min(targets.size(), qpos_addr.size());
        for (size_t i = 0; i < count; ++i) {
            if (!detail.empty()) detail += ",";
            detail += std::to_string(i) + ":target=" +
                std::to_string(targets[i]) + "/actual=" +
                std::to_string(data->qpos[qpos_addr[i]]) + "/error=" +
                std::to_string(targets[i] - data->qpos[qpos_addr[i]]);
        }
        return detail;
    }

    void ClampJoint(int index) {
        const int joint_id = joint_ids[index];
        if (!model->jnt_limited[joint_id]) {
            return;
        }
        const double low = model->jnt_range[2 * joint_id];
        const double high = model->jnt_range[2 * joint_id + 1];
        data->qpos[qpos_addr[index]] =
            std::clamp(data->qpos[qpos_addr[index]], low, high);
    }

    bool GetSitePose(int site_id, Pose3D& pose) const {
        if (site_id < 0) {
            return false;
        }
        const mjtNum* pos = data->site_xpos + 3 * site_id;
        const mjtNum* mat = data->site_xmat + 9 * site_id;
        double quat[4] = {1.0, 0.0, 0.0, 0.0};
        mju_mat2Quat(quat, mat);
        pose.x = static_cast<float>(pos[0]);
        pose.y = static_cast<float>(pos[1]);
        pose.z = static_cast<float>(pos[2]);
        pose.qw = static_cast<float>(quat[0]);
        pose.qx = static_cast<float>(quat[1]);
        pose.qy = static_cast<float>(quat[2]);
        pose.qz = static_cast<float>(quat[3]);
        return true;
    }

    bool GetPose(Pose3D& pose) const {
        return GetSitePose(ee_site_id, pose);
    }

    bool GetHoldPose(Pose3D& pose) const {
        return GetSitePose(hold_site_id, pose);
    }

    Pose3D SolverPoseForHoldPose(const Pose3D& hold_pose) {
        if (hold_site_id < 0 || ee_site_id < 0 ||
            hold_site_id == ee_site_id) {
            return hold_pose;
        }
        ForwardWithAttachments();
        const mjtNum* hold_pos = data->site_xpos + 3 * hold_site_id;
        const mjtNum* hold_mat = data->site_xmat + 9 * hold_site_id;
        const mjtNum* ee_pos = data->site_xpos + 3 * ee_site_id;
        const mjtNum* ee_mat = data->site_xmat + 9 * ee_site_id;

        mjtNum hold_quat[4] = {1.0, 0.0, 0.0, 0.0};
        mjtNum ee_quat[4] = {1.0, 0.0, 0.0, 0.0};
        mjtNum inverse_ee_quat[4] = {};
        mjtNum ee_to_hold_quat[4] = {};
        mju_mat2Quat(hold_quat, hold_mat);
        mju_mat2Quat(ee_quat, ee_mat);
        QuatConjugate(ee_quat, inverse_ee_quat);
        QuatMultiply(
            inverse_ee_quat, hold_quat, ee_to_hold_quat);

        const mjtNum world_offset[3] = {
            hold_pos[0] - ee_pos[0],
            hold_pos[1] - ee_pos[1],
            hold_pos[2] - ee_pos[2],
        };
        mjtNum ee_to_hold_offset[3] = {};
        RotateVectorByQuat(
            inverse_ee_quat, world_offset, ee_to_hold_offset);

        mjtNum desired_hold_quat[4] = {};
        if (!NormalizeQuaternion(hold_pose, desired_hold_quat)) {
            return hold_pose;
        }
        mjtNum inverse_ee_to_hold_quat[4] = {};
        mjtNum desired_ee_quat[4] = {};
        QuatConjugate(
            ee_to_hold_quat, inverse_ee_to_hold_quat);
        QuatMultiply(
            desired_hold_quat, inverse_ee_to_hold_quat,
            desired_ee_quat);

        mjtNum desired_world_offset[3] = {};
        RotateVectorByQuat(
            desired_ee_quat, ee_to_hold_offset,
            desired_world_offset);

        Pose3D solver_pose = hold_pose;
        solver_pose.x -= static_cast<float>(desired_world_offset[0]);
        solver_pose.y -= static_cast<float>(desired_world_offset[1]);
        solver_pose.z -= static_cast<float>(desired_world_offset[2]);
        solver_pose.qw = static_cast<float>(desired_ee_quat[0]);
        solver_pose.qx = static_cast<float>(desired_ee_quat[1]);
        solver_pose.qy = static_cast<float>(desired_ee_quat[2]);
        solver_pose.qz = static_cast<float>(desired_ee_quat[3]);
        return solver_pose;
    }

    bool SolvePositionIK(const Pose3D& target,
                        const ExecutorConfig::MujocoConfig& config,
                        bool constrain_orientation,
                        std::vector<float>* solution,
                        std::string* detail) {
        if (!IsFinitePose(target)) {
            if (detail) *detail = "target pose contains non-finite values";
            return false;
        }

        std::vector<mjtNum> saved_qpos(data->qpos, data->qpos + model->nq);
        std::vector<mjtNum> saved_qvel(data->qvel, data->qvel + model->nv);
        std::vector<mjtNum> saved_ctrl(data->ctrl, data->ctrl + model->nu);
        mjtNum target_quat[4] = {1.0, 0.0, 0.0, 0.0};
        const bool use_orientation =
            constrain_orientation && NormalizeQuaternion(target, target_quat);
        constexpr double kOrientationWeight = 0.20;
        constexpr double kOrientationToleranceRad = 0.10;

        std::vector<double> jacp(3 * model->nv, 0.0);
        std::vector<double> jacr(3 * model->nv, 0.0);
        std::vector<float> best_joints = CurrentJoints();
        double best_error_norm = std::numeric_limits<double>::infinity();
        double best_orientation_error =
            std::numeric_limits<double>::infinity();
        constexpr std::array<std::array<double, 3>, 7> kSeedOffsets = {{
            {{0.0, 0.0, 0.0}},
            {{0.75, 0.0, 0.0}},
            {{-0.75, 0.0, 0.0}},
            {{0.0, 0.55, -0.55}},
            {{0.0, -0.55, 0.55}},
            {{1.40, 0.45, -0.45}},
            {{-1.40, 0.45, -0.45}},
        }};

        for (const auto& seed_offset : kSeedOffsets) {
            std::copy(saved_qpos.begin(), saved_qpos.end(), data->qpos);
            std::copy(saved_qvel.begin(), saved_qvel.end(), data->qvel);
            std::copy(saved_ctrl.begin(), saved_ctrl.end(), data->ctrl);
            for (size_t joint = 0;
                    joint < std::min<size_t>(3, qpos_addr.size()); ++joint) {
                data->qpos[qpos_addr[joint]] += seed_offset[joint];
                ClampJoint(static_cast<int>(joint));
            }

            for (int iter = 0; iter < config.ik_iterations; ++iter) {
            ForwardWithAttachments();
            const mjtNum* pos = data->site_xpos + 3 * ee_site_id;
            const mjtNum* mat = data->site_xmat + 9 * ee_site_id;
            mjtNum current_quat[4] = {1.0, 0.0, 0.0, 0.0};
            mju_mat2Quat(current_quat, mat);
            mjtNum inv_current_quat[4] = {};
            mjtNum error_quat[4] = {};
            QuatConjugate(current_quat, inv_current_quat);
            QuatMultiply(target_quat, inv_current_quat, error_quat);
            if (error_quat[0] < 0.0) {
                for (double& value : error_quat) {
                    value = -value;
                }
            }
            const double orientation_error[3] = {
                2.0 * error_quat[1],
                2.0 * error_quat[2],
                2.0 * error_quat[3],
            };
            const double orientation_error_norm = std::sqrt(
                orientation_error[0] * orientation_error[0] +
                orientation_error[1] * orientation_error[1] +
                orientation_error[2] * orientation_error[2]);
            const double err[3] = {
                static_cast<double>(target.x) - pos[0],
                static_cast<double>(target.y) - pos[1],
                static_cast<double>(target.z) - pos[2],
            };
            const double err_norm = std::sqrt(
                err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
            const double previous_best_score = best_error_norm +
                kOrientationWeight * best_orientation_error;
            const double score = err_norm +
                kOrientationWeight * orientation_error_norm;
            if (score < previous_best_score) {
                best_error_norm = err_norm;
                best_orientation_error = orientation_error_norm;
                best_joints = CurrentJoints();
            }
            if (err_norm <= config.ik_position_tolerance_m &&
                (!use_orientation ||
                    orientation_error_norm <= kOrientationToleranceRad)) {
                if (solution) *solution = CurrentJoints();
                std::copy(saved_qpos.begin(), saved_qpos.end(), data->qpos);
                std::copy(saved_qvel.begin(), saved_qvel.end(), data->qvel);
                std::copy(saved_ctrl.begin(), saved_ctrl.end(), data->ctrl);
                ForwardWithAttachments();
                return true;
            }

            std::fill(jacp.begin(), jacp.end(), 0.0);
            std::fill(jacr.begin(), jacr.end(), 0.0);
            mj_jacSite(model, data, jacp.data(), jacr.data(), ee_site_id);

            const size_t dof_count = qvel_addr.size();
            std::vector<double> normal(dof_count * dof_count, 0.0);
            std::vector<double> rhs(dof_count, 0.0);
            const double lambda2 =
                static_cast<double>(config.ik_damping) * config.ik_damping;
            const double task_error[6] = {
                err[0],
                err[1],
                err[2],
                use_orientation
                    ? kOrientationWeight * orientation_error[0] : 0.0,
                use_orientation
                    ? kOrientationWeight * orientation_error[1] : 0.0,
                use_orientation
                    ? kOrientationWeight * orientation_error[2] : 0.0,
            };
            for (size_t j = 0; j < dof_count; ++j) {
                const int col_j = qvel_addr[j];
                const double jac_j[6] = {
                    jacp[col_j],
                    jacp[model->nv + col_j],
                    jacp[2 * model->nv + col_j],
                    use_orientation ? kOrientationWeight * jacr[col_j] : 0.0,
                    use_orientation
                        ? kOrientationWeight * jacr[model->nv + col_j] : 0.0,
                    use_orientation
                        ? kOrientationWeight * jacr[2 * model->nv + col_j]
                        : 0.0,
                };
                for (int row = 0; row < 6; ++row) {
                    rhs[j] += jac_j[row] * task_error[row];
                }
                for (size_t k = 0; k < dof_count; ++k) {
                    const int col_k = qvel_addr[k];
                    const double jac_k[6] = {
                        jacp[col_k],
                        jacp[model->nv + col_k],
                        jacp[2 * model->nv + col_k],
                        use_orientation
                            ? kOrientationWeight * jacr[col_k] : 0.0,
                        use_orientation
                            ? kOrientationWeight * jacr[model->nv + col_k]
                            : 0.0,
                        use_orientation
                            ? kOrientationWeight * jacr[2 * model->nv + col_k]
                            : 0.0,
                    };
                    for (int row = 0; row < 6; ++row) {
                        normal[j * dof_count + k] +=
                            jac_j[row] * jac_k[row];
                    }
                }
                normal[j * dof_count + j] += lambda2;
            }

            std::vector<double> dq;
            if (!SolveLinearSystem(normal, rhs, &dq)) {
                if (detail) *detail = "singular damped least-squares system";
                break;
            }

            const double scale = std::clamp(
                static_cast<double>(config.ik_step_scale), 0.05, 1.0);
            constexpr double kMaximumJointUpdateRad = 0.15;
            for (size_t j = 0; j < qpos_addr.size(); ++j) {
                const double update = std::clamp(
                    scale * dq[j], -kMaximumJointUpdateRad,
                    kMaximumJointUpdateRad);
                data->qpos[qpos_addr[j]] += update;
                ClampJoint(static_cast<int>(j));
            }
            }
        }

        if (detail) {
            *detail = "pose IK did not converge; error_m=" +
                std::to_string(best_error_norm) +
                " orientation_error_rad=" +
                std::to_string(best_orientation_error);
        }
        if (solution) *solution = best_joints;
        std::copy(saved_qpos.begin(), saved_qpos.end(), data->qpos);
        std::copy(saved_qvel.begin(), saved_qvel.end(), data->qvel);
        std::copy(saved_ctrl.begin(), saved_ctrl.end(), data->ctrl);
        ForwardWithAttachments();
        return false;
    }
};

MujocoGraspExecutor::MujocoGraspExecutor(const ExecutorConfig& config)
    : GraspExecutor(config), impl_(std::make_unique<Impl>()) {}

MujocoGraspExecutor::~MujocoGraspExecutor() = default;

void MujocoGraspExecutor::SetTargetLabel(const std::string& target_label) {
    impl_->target_label = target_label;
    impl_->last_pre_grasp_valid = false;
    RecordResult(GraspResult::SUCCESS, "set_target_label");
}

void MujocoGraspExecutor::ResetScene() {
    impl_->ResetTaskScene(config_);
    impl_->target_label.clear();
    impl_->last_pre_grasp_valid = false;
    RecordResult(GraspResult::SUCCESS, "reset_scene");
}

void MujocoGraspExecutor::SetVisualStepCallback(
    std::function<void()> callback) {
    impl_->visual_step_callback = std::move(callback);
}

void MujocoGraspExecutor::RecordResult(GraspResult result,
                                        const std::string& action,
                                        const std::string& detail) {
    diagnostics_.last_result = result;
    diagnostics_.last_action = action;
    diagnostics_.last_detail = detail;
}

bool MujocoGraspExecutor::Init() {
    std::string error;
    if (!impl_->Load(config_, &error)) {
        std::cerr << "[MujocoExecutor] init failed: " << error << std::endl;
        RecordResult(GraspResult::MOVE_FAILED, "init", error);
        return false;
    }
    if (config_.home_joints.size() == impl_->joint_ids.size()) {
        impl_->SetJointPositionsDirect(config_.home_joints);
    }
    SetGripperCtrl(config_.mujoco.gripper_open_ctrl, "init_open_gripper");
    std::cout << "[MujocoExecutor] Initialized: xml="
                << config_.mujoco.xml_path
                << " joints=" << impl_->joint_ids.size()
                << " site=" << config_.mujoco.end_effector_site
                << std::endl;
    RecordResult(GraspResult::SUCCESS, "init");
    return true;
}

GraspResult MujocoGraspExecutor::MoveToJointsSim(
    const std::vector<float>& joints,
    const char* action,
    bool allow_target_contact) {
    if (joints.size() != impl_->joint_ids.size()) {
        const std::string detail = "joint target size mismatch";
        RecordResult(GraspResult::OUT_OF_RANGE, action, detail);
        return GraspResult::OUT_OF_RANGE;
    }

    const std::vector<float> start = impl_->CurrentJoints();
    float max_delta = 0.0f;
    for (size_t i = 0; i < joints.size(); ++i) {
        max_delta = std::max(max_delta, std::abs(joints[i] - start[i]));
    }
    const double timestep = impl_->model->opt.timestep;
    const double speed = std::max(0.10, static_cast<double>(config_.move_speed));
    const double duration = std::max(0.25, max_delta / speed);
    const int interpolation_steps = std::clamp(
        static_cast<int>(std::ceil(duration / timestep)),
        20, std::max(20, config_.mujoco.max_motion_steps));
    std::vector<float> interpolated(joints.size(), 0.0f);
    for (int step = 1; step <= interpolation_steps; ++step) {
        const float alpha = static_cast<float>(step) /
            static_cast<float>(interpolation_steps);
        for (size_t i = 0; i < joints.size(); ++i) {
            interpolated[i] = start[i] + alpha * (joints[i] - start[i]);
        }
        impl_->StepWithTargets(interpolated);
        std::string detail;
        if (impl_->HasUnsafeCollision(allow_target_contact, &detail)) {
            RecordResult(GraspResult::MOVE_FAILED, action, detail);
            return GraspResult::MOVE_FAILED;
        }
        impl_->MaybeRenderVisualStep(step);
    }

    int stable_steps = 0;
    const int settle_steps = std::max(1, config_.mujoco.settle_steps);
    for (int i = 0; i < settle_steps; ++i) {
        impl_->StepWithTargets(joints);
        std::string detail;
        if (impl_->HasUnsafeCollision(allow_target_contact, &detail)) {
            RecordResult(GraspResult::MOVE_FAILED, action, detail);
            return GraspResult::MOVE_FAILED;
        }
        if (impl_->JointError(joints) <= config_.mujoco.joint_tolerance_rad) {
            ++stable_steps;
            if (stable_steps >= 10) break;
        } else {
            stable_steps = 0;
        }
        impl_->MaybeRenderVisualStep(interpolation_steps + i);
    }
    const double final_error = impl_->JointError(joints);
    if (final_error > config_.mujoco.joint_tolerance_rad) {
        const std::string detail = "joint motion did not settle; error_rad=" +
            std::to_string(final_error) + " joint_errors=[" +
            impl_->JointErrorDetail(joints) + "]";
        RecordResult(GraspResult::TIMEOUT, action, detail);
        return GraspResult::TIMEOUT;
    }
    RecordResult(GraspResult::SUCCESS, action);
    return GraspResult::SUCCESS;
}

GraspResult MujocoGraspExecutor::MoveToPoseSim(const Pose3D& pose,
                                                const char* action,
                                                bool constrain_orientation,
                                                bool allow_target_contact) {
    const Pose3D solver_pose = impl_->SolverPoseForHoldPose(pose);
    Pose3D compensated_pose = solver_pose;
    constexpr int kMaximumTrackingCorrections = 8;
    double position_error = std::numeric_limits<double>::infinity();
    for (int attempt = 0; attempt < kMaximumTrackingCorrections; ++attempt) {
        std::vector<float> joints;
        std::string detail;
        if (!impl_->SolvePositionIK(
                compensated_pose, config_.mujoco, constrain_orientation,
                &joints, &detail)) {
            RecordResult(GraspResult::IK_FAILED, action, detail);
            return GraspResult::IK_FAILED;
        }
        const GraspResult move_result = MoveToJointsSim(
            joints, action, allow_target_contact);
        if (move_result != GraspResult::SUCCESS) {
            return move_result;
        }
        Pose3D actual_pose;
        if (!impl_->GetPose(actual_pose)) {
            RecordResult(
                GraspResult::MOVE_FAILED, action,
                "failed to read MuJoCo end-effector pose");
            return GraspResult::MOVE_FAILED;
        }
        position_error = std::sqrt(
            std::pow(static_cast<double>(solver_pose.x - actual_pose.x), 2) +
            std::pow(static_cast<double>(solver_pose.y - actual_pose.y), 2) +
            std::pow(static_cast<double>(solver_pose.z - actual_pose.z), 2));
        if (position_error <=
            config_.mujoco.cartesian_tracking_tolerance_m) {
            return GraspResult::SUCCESS;
        }
        compensated_pose.x += solver_pose.x - actual_pose.x;
        compensated_pose.y += solver_pose.y - actual_pose.y;
        compensated_pose.z += solver_pose.z - actual_pose.z;
    }
    const std::string detail = "Cartesian motion did not settle; error_m=" +
        std::to_string(position_error) + " tolerance_m=" +
        std::to_string(config_.mujoco.cartesian_tracking_tolerance_m);
    RecordResult(GraspResult::TIMEOUT, action, detail);
    return GraspResult::TIMEOUT;
}

GraspResult MujocoGraspExecutor::SetGripperCtrl(float ctrl,
                                                const char* action) {
    if (impl_->gripper_actuator_id < 0) {
        RecordResult(GraspResult::MOVE_FAILED, action,
                    "missing gripper actuator");
        return GraspResult::MOVE_FAILED;
    }
    for (int i = 0; i < config_.mujoco.settle_steps; ++i) {
        impl_->data->ctrl[impl_->gripper_actuator_id] = ctrl;
        mj_step(impl_->model, impl_->data);
        impl_->ForwardWithAttachments();
        impl_->MaybeRenderVisualStep(i);
    }
    RecordResult(GraspResult::SUCCESS, action);
    return GraspResult::SUCCESS;
}

GraspResult MujocoGraspExecutor::MoveToObserve() {
    return MoveToJointsSim(config_.observe_joints, "move_to_observe");
}

GraspResult MujocoGraspExecutor::MoveToSideObserve() {
    return MoveToJointsSim(config_.side_ready_joints, "move_to_side_observe");
}

GraspResult MujocoGraspExecutor::MoveToHome() {
    return MoveToJointsSim(config_.home_joints, "move_to_home");
}

GraspResult MujocoGraspExecutor::MoveToPreGrasp(
    const Pose3D& pre_grasp_pose, float grasp_yaw_rad,
    bool use_top_constraints) {
    if (!use_top_constraints && !impl_->validated_pre_grasp_path.empty()) {
        for (const std::vector<float>& joints :
                impl_->validated_pre_grasp_path) {
            const GraspResult result = MoveToJointsSim(
                joints, "move_to_pre_grasp", false);
            if (result != GraspResult::SUCCESS) {
                return result;
            }
        }
        impl_->last_pre_grasp_pose = pre_grasp_pose;
        impl_->last_pre_grasp_valid = true;
        return GraspResult::SUCCESS;
    }
    const Pose3D target_pose = use_top_constraints
        ? AlignOpeningAxisYaw(pre_grasp_pose, grasp_yaw_rad)
        : pre_grasp_pose;
    const GraspResult result =
        MoveToPoseSim(target_pose, "move_to_pre_grasp");
    if (result == GraspResult::SUCCESS) {
        impl_->last_pre_grasp_pose = target_pose;
        impl_->last_pre_grasp_valid = true;
    }
    return result;
}

GraspResult MujocoGraspExecutor::OpenGripperForGrasp(float minimum_opening) {
    (void)minimum_opening;
    return SetGripperCtrl(config_.mujoco.gripper_open_ctrl, "open_gripper");
}

GraspResult MujocoGraspExecutor::MoveToGrasp(
    const Pose3D& grasp_pose, float grasp_yaw_rad,
    bool use_top_constraints) {
    if (!use_top_constraints && !impl_->validated_grasp_path.empty()) {
        for (const std::vector<float>& joints : impl_->validated_grasp_path) {
            const GraspResult result = MoveToJointsSim(
                joints, "move_to_grasp", true);
            if (result != GraspResult::SUCCESS) {
                return result;
            }
        }
        return GraspResult::SUCCESS;
    }
    const Pose3D target_pose = use_top_constraints
        ? AlignOpeningAxisYaw(grasp_pose, grasp_yaw_rad)
        : grasp_pose;
    return MoveToPoseSim(target_pose, "move_to_grasp", true, true);
}

GraspResult MujocoGraspExecutor::CloseGripperAndCheck() {
    Pose3D pose_before_close{};
    const bool have_pose_before_close =
        impl_->GetHoldPose(pose_before_close);
    std::string detail;
    if (!impl_->CloseGripperOnContact(config_, &detail)) {
        Pose3D pose_after_close{};
        if (have_pose_before_close &&
            impl_->GetHoldPose(pose_after_close)) {
            detail += " tcp_before_close=[" +
                std::to_string(pose_before_close.x) + "," +
                std::to_string(pose_before_close.y) + "," +
                std::to_string(pose_before_close.z) + "]" +
                " tcp_close_delta=[" +
                std::to_string(pose_after_close.x - pose_before_close.x) +
                "," +
                std::to_string(pose_after_close.y - pose_before_close.y) +
                "," +
                std::to_string(pose_after_close.z - pose_before_close.z) +
                "]";
        }
        const std::string empty_detail = detail;
        std::cout << "[MujocoExecutor] empty grasp: "
                << empty_detail << std::endl;
        SetGripperCtrl(
            config_.mujoco.gripper_open_ctrl, "reopen_after_empty_grasp");
        std::string recovery_detail;
        if (impl_->last_pre_grasp_valid) {
            const GraspResult recovery_result = MoveToPoseSim(
                impl_->last_pre_grasp_pose,
                "retract_after_empty_grasp", true, true);
            recovery_detail = "; retract_after_empty=" + std::string(
                recovery_result == GraspResult::SUCCESS
                    ? "success" : "failed");
        }
        RecordResult(GraspResult::EMPTY, "close_gripper_and_check",
                    empty_detail + recovery_detail);
        return GraspResult::EMPTY;
    }
    RecordResult(GraspResult::SUCCESS, "close_gripper_and_check", detail);
    return GraspResult::SUCCESS;
}

GraspResult MujocoGraspExecutor::LiftFromGrasp(
    const Pose3D& retreat_pose, const Pose3D& lift_pose,
    float grasp_yaw_rad, bool use_top_constraints) {
    if (!use_top_constraints && !impl_->validated_lift_path.empty()) {
        for (const std::vector<float>& joints : impl_->validated_lift_path) {
            const GraspResult result = MoveToJointsSim(
                joints, "lift_from_grasp", true);
            if (result != GraspResult::SUCCESS) {
                return result;
            }
        }
        if (!impl_->HasVerifiedPhysicalHold()) {
            const std::string detail =
                "target lost physical pad contact during lift";
            RecordResult(GraspResult::EMPTY, "lift_from_grasp", detail);
            return GraspResult::EMPTY;
        }
        return GraspResult::SUCCESS;
    }
    const Pose3D retreat_target = AlignOpeningAxisYaw(
        retreat_pose, grasp_yaw_rad);
    const Pose3D lift_target = AlignOpeningAxisYaw(
        lift_pose, grasp_yaw_rad);
    GraspResult result = MoveToPoseSim(
        retreat_target, "retreat_from_grasp", true, true);
    if (result != GraspResult::SUCCESS) {
        return result;
    }
    result = MoveToPoseSim(lift_target, "lift_from_grasp", true, true);
    if (result != GraspResult::SUCCESS) return result;
    if (!impl_->HasVerifiedPhysicalHold()) {
        const std::string detail = "target lost physical pad contact during lift";
        RecordResult(GraspResult::EMPTY, "lift_from_grasp", detail);
        return GraspResult::EMPTY;
    }
    return GraspResult::SUCCESS;
}

GraspResult MujocoGraspExecutor::ValidateGraspPoses(
    const Pose3D& pre_grasp_pose,
    const Pose3D& grasp_pose,
    const Pose3D& retreat_pose,
    const Pose3D& lift_pose,
    float entry_clearance_z_m,
    float grasp_yaw_rad,
    bool use_top_constraints,
    int timeout_ms,
    std::string* detail) {
    (void)timeout_ms;
    impl_->validated_pre_grasp_path.clear();
    impl_->validated_grasp_path.clear();
    impl_->validated_lift_path.clear();
    const std::vector<mjtNum> saved_qpos(
        impl_->data->qpos, impl_->data->qpos + impl_->model->nq);
    const std::vector<mjtNum> saved_qvel(
        impl_->data->qvel, impl_->data->qvel + impl_->model->nv);
    const std::vector<mjtNum> saved_ctrl(
        impl_->data->ctrl, impl_->data->ctrl + impl_->model->nu);
    const auto restore_state = [&]() {
        std::copy(saved_qpos.begin(), saved_qpos.end(), impl_->data->qpos);
        std::copy(saved_qvel.begin(), saved_qvel.end(), impl_->data->qvel);
        std::copy(saved_ctrl.begin(), saved_ctrl.end(), impl_->data->ctrl);
        impl_->ForwardWithAttachments();
    };

    std::vector<float> start_joints = impl_->CurrentJoints();
    std::vector<float> joints;
    std::string error;
    struct ValidationWaypoint {
        Pose3D pose;
        int execution_stage;
        bool allow_target_contact;
        std::vector<float> joint_target;
    };
    std::vector<ValidationWaypoint> waypoints;
    if (use_top_constraints) {
        waypoints = {
            {AlignOpeningAxisYaw(pre_grasp_pose, grasp_yaw_rad),
                0, false, {}},
            {AlignOpeningAxisYaw(grasp_pose, grasp_yaw_rad),
                1, true, {}},
            {AlignOpeningAxisYaw(retreat_pose, grasp_yaw_rad),
                2, true, {}},
            {AlignOpeningAxisYaw(lift_pose, grasp_yaw_rad),
                2, true, {}},
        };
    } else {
        if (!std::isfinite(entry_clearance_z_m) ||
            entry_clearance_z_m < pre_grasp_pose.z) {
            restore_state();
            error = "side entry clearance height is invalid";
            if (detail) *detail = error;
            RecordResult(
                GraspResult::OUT_OF_RANGE, "validate_grasp_poses", error);
            return GraspResult::OUT_OF_RANGE;
        }
        Pose3D elevated_pre_grasp = pre_grasp_pose;
        elevated_pre_grasp.z = entry_clearance_z_m;
        const std::vector<float>& staging_source =
            config_.side_ready_joints.size() == start_joints.size()
            ? config_.side_ready_joints
            : config_.home_joints;
        std::vector<float> aligned_staging_joints = staging_source;
        if (!aligned_staging_joints.empty() &&
            impl_->robot_root_body_id >= 0) {
            const mjtNum* root_position =
                impl_->data->xpos + 3 * impl_->robot_root_body_id;
            aligned_staging_joints[0] = std::atan2(
                pre_grasp_pose.y - static_cast<float>(root_position[1]),
                pre_grasp_pose.x - static_cast<float>(root_position[0]));
        }
        waypoints.push_back(
            {Pose3D{}, 0, false, staging_source});
        waypoints.push_back(
            {Pose3D{}, 0, false, aligned_staging_joints});
        waypoints.push_back(
            {elevated_pre_grasp, 0, false, {}});

        constexpr int kSideDescentSegments = 6;
        for (int segment = 1; segment <= kSideDescentSegments; ++segment) {
            const float alpha = static_cast<float>(segment) /
                static_cast<float>(kSideDescentSegments);
            Pose3D pose = pre_grasp_pose;
            pose.z = elevated_pre_grasp.z +
                alpha * (pre_grasp_pose.z - elevated_pre_grasp.z);
            waypoints.push_back({pose, 0, false, {}});
        }

        constexpr int kSideApproachSegments = 5;
        for (int segment = 1; segment <= kSideApproachSegments; ++segment) {
            const float alpha = static_cast<float>(segment) /
                static_cast<float>(kSideApproachSegments);
            Pose3D pose = grasp_pose;
            pose.x = pre_grasp_pose.x +
                alpha * (grasp_pose.x - pre_grasp_pose.x);
            pose.y = pre_grasp_pose.y +
                alpha * (grasp_pose.y - pre_grasp_pose.y);
            pose.z = pre_grasp_pose.z +
                alpha * (grasp_pose.z - pre_grasp_pose.z);
            waypoints.push_back({pose, 1, true, {}});
        }

        constexpr int kSideLiftSegments = 4;
        for (int segment = 1; segment <= kSideLiftSegments; ++segment) {
            const float alpha = static_cast<float>(segment) /
                static_cast<float>(kSideLiftSegments);
            Pose3D pose = retreat_pose;
            pose.x = grasp_pose.x +
                alpha * (retreat_pose.x - grasp_pose.x);
            pose.y = grasp_pose.y +
                alpha * (retreat_pose.y - grasp_pose.y);
            pose.z = grasp_pose.z +
                alpha * (retreat_pose.z - grasp_pose.z);
            waypoints.push_back({pose, 2, true, {}});
        }
        for (int segment = 1; segment <= kSideLiftSegments; ++segment) {
            const float alpha = static_cast<float>(segment) /
                static_cast<float>(kSideLiftSegments);
            Pose3D pose = lift_pose;
            pose.x = retreat_pose.x +
                alpha * (lift_pose.x - retreat_pose.x);
            pose.y = retreat_pose.y +
                alpha * (lift_pose.y - retreat_pose.y);
            pose.z = retreat_pose.z +
                alpha * (lift_pose.z - retreat_pose.z);
            waypoints.push_back({pose, 2, true, {}});
        }
    }
    for (size_t pose_index = 0; pose_index < waypoints.size();
            ++pose_index) {
        if (!waypoints[pose_index].joint_target.empty()) {
            joints = waypoints[pose_index].joint_target;
            if (joints.size() != start_joints.size()) {
                restore_state();
                error = "side staging joint count does not match model";
                if (detail) *detail = error;
                RecordResult(
                    GraspResult::OUT_OF_RANGE,
                    "validate_grasp_poses", error);
                return GraspResult::OUT_OF_RANGE;
            }
        } else {
            const Pose3D solver_pose =
                impl_->SolverPoseForHoldPose(waypoints[pose_index].pose);
            if (!impl_->SolvePositionIK(
                    solver_pose, config_.mujoco, true, &joints, &error)) {
                restore_state();
                if (detail) *detail = error;
                RecordResult(
                    GraspResult::IK_FAILED, "validate_grasp_poses", error);
                return GraspResult::IK_FAILED;
            }
        }

        constexpr int kPathSamples = 24;
        for (int sample = 1; sample <= kPathSamples; ++sample) {
            const float alpha = static_cast<float>(sample) /
                static_cast<float>(kPathSamples);
            std::vector<float> sample_joints(joints.size(), 0.0f);
            for (size_t joint = 0; joint < joints.size(); ++joint) {
                sample_joints[joint] = start_joints[joint] +
                    alpha * (joints[joint] - start_joints[joint]);
            }
            impl_->SetJointPositionsDirect(sample_joints);
            std::string collision_detail;
            if (impl_->HasUnsafeCollision(
                    waypoints[pose_index].allow_target_contact,
                    &collision_detail)) {
                restore_state();
                error = "path " + std::to_string(pose_index + 1) + "/" +
                    std::to_string(waypoints.size()) + " " +
                    "sample " + std::to_string(sample) + "/" +
                    std::to_string(kPathSamples) + ": " + collision_detail;
                if (detail) *detail = error;
                RecordResult(
                    GraspResult::MOVE_FAILED, "validate_grasp_poses", error);
                return GraspResult::MOVE_FAILED;
            }
        }
        start_joints = joints;
        impl_->SetJointPositionsDirect(start_joints);
        if (!use_top_constraints) {
            if (waypoints[pose_index].execution_stage == 0) {
                impl_->validated_pre_grasp_path.push_back(joints);
            } else if (waypoints[pose_index].execution_stage == 1) {
                impl_->validated_grasp_path.push_back(joints);
            } else {
                impl_->validated_lift_path.push_back(joints);
            }
        }
    }
    restore_state();
    if (detail) detail->clear();
    RecordResult(GraspResult::SUCCESS, "validate_grasp_poses");
    return GraspResult::SUCCESS;
}

void MujocoGraspExecutor::SetSupportPlane(
    const SupportPlane& support_plane) {
    (void)support_plane;
}

GraspResult MujocoGraspExecutor::MoveToPlace() {
    const auto move_place_pose = [this](
            const Pose3D& pose, const char* action) {
        GraspResult result = MoveToPoseSim(pose, action, true, true);
        if (result != GraspResult::IK_FAILED &&
            result != GraspResult::TIMEOUT) {
            return result;
        }
        std::cout << "[MujocoExecutor] " << action
                << ": preserving wrist orientation is unreachable or "
                << "cannot settle under load; "
                << "retrying with free wrist orientation" << std::endl;
        return MoveToPoseSim(pose, action, false, true);
    };

    Pose3D drop_pose;
    if (!impl_->GetDropZonePose(&drop_pose)) {
        if (impl_->held_body_id >= 0) {
            RecordResult(
                GraspResult::OUT_OF_RANGE, "move_to_place",
                "no collision-free placement slot in drop zone");
            return GraspResult::OUT_OF_RANGE;
        }
        return MoveToJointsSim(config_.place_joints, "move_to_place");
    }
    Pose3D transit_pose;
    if (!impl_->GetHoldPose(transit_pose)) {
        RecordResult(
            GraspResult::MOVE_FAILED, "move_to_place_transit",
            "end-effector pose is unavailable");
        return GraspResult::MOVE_FAILED;
    }
    transit_pose.z = static_cast<float>(std::max(
        static_cast<double>(transit_pose.z),
        static_cast<double>(drop_pose.z) +
            kMujocoPlaceTransitClearance));
    GraspResult result = move_place_pose(
        transit_pose, "move_to_place_transit");
    if (result != GraspResult::SUCCESS) {
        return result;
    }

    if (!impl_->GetDropZonePose(&drop_pose)) {
        if (impl_->held_body_id >= 0) {
            RecordResult(
                GraspResult::OUT_OF_RANGE, "move_to_place",
                "placement slot became unavailable");
            return GraspResult::OUT_OF_RANGE;
        }
        return MoveToJointsSim(config_.place_joints, "move_to_place");
    }
    Pose3D hover_pose = drop_pose;
    hover_pose.z = transit_pose.z;
    result = move_place_pose(hover_pose, "move_above_place");
    if (result != GraspResult::SUCCESS) {
        return result;
    }

    Pose3D descent_start;
    if (!impl_->GetHoldPose(descent_start)) {
        RecordResult(
            GraspResult::MOVE_FAILED, "move_to_place",
            "end-effector pose is unavailable before descent");
        return GraspResult::MOVE_FAILED;
    }
    constexpr int kPlaceDescentSegments = 4;
    for (int segment = 1; segment <= kPlaceDescentSegments; ++segment) {
        const float alpha = static_cast<float>(segment) /
            static_cast<float>(kPlaceDescentSegments);
        Pose3D descent_pose = drop_pose;
        descent_pose.z = descent_start.z +
            alpha * (drop_pose.z - descent_start.z);
        result = move_place_pose(
            descent_pose, "move_to_place_descent");
        if (result != GraspResult::SUCCESS) {
            return result;
        }
    }

    result = GraspResult::MOVE_FAILED;
    std::string detail;
    for (int attempt = 0; attempt < kMujocoPlaceCorrectionAttempts; ++attempt) {
        if (!impl_->GetDropZonePose(&drop_pose)) {
            RecordResult(
                GraspResult::OUT_OF_RANGE, "move_to_place",
                "placement slot became unavailable during correction");
            return GraspResult::OUT_OF_RANGE;
        }
        result = move_place_pose(drop_pose, "move_to_place");
        if (result != GraspResult::SUCCESS) {
            return result;
        }
        if (impl_->HeldObjectInsideDropZoneXY(&detail)) {
            RecordResult(GraspResult::SUCCESS, "move_to_place", detail);
            return GraspResult::SUCCESS;
        }
        std::cout << "[MujocoExecutor] place correction "
                << (attempt + 1) << "/"
                << kMujocoPlaceCorrectionAttempts << ": "
                << detail << std::endl;
    }
    RecordResult(result, "move_to_place", detail);
    return result;
}

GraspResult MujocoGraspExecutor::ReleaseObject() {
    bool inside_before_release = false;
    const std::string before_release =
        impl_->DescribeTargetDropStatus(&inside_before_release);
    impl_->ReleaseHeldObject(0);
    GraspResult result = SetGripperCtrl(
        config_.mujoco.gripper_open_ctrl, "release_object");
    bool inside_zone = false;
    const std::string detail = impl_->DescribeTargetDropStatus(&inside_zone);
    if (result == GraspResult::SUCCESS && !inside_zone) {
        result = GraspResult::MOVE_FAILED;
    }
    RecordResult(result, "release_object", detail);
    std::cout << "[MujocoExecutor] release_object: before={"
            << before_release << "} after={" << detail << "}" << std::endl;
    return result;
}

GraspResult MujocoGraspExecutor::CloseGripper() {
    return SetGripperCtrl(config_.mujoco.gripper_close_ctrl, "close_gripper");
}

GraspResult MujocoGraspExecutor::ExecuteGrasp(
    const Pose3D& grasp_pose,
    const Pose3D& pre_grasp_pose,
    float grasp_yaw_rad) {
    GraspResult result = MoveToPreGrasp(pre_grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;
    result = OpenGripperForGrasp();
    if (result != GraspResult::SUCCESS) return result;
    result = MoveToGrasp(grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;
    return CloseGripperAndCheck();
}

GraspResult MujocoGraspExecutor::ExecutePlace() {
    GraspResult result = MoveToPlace();
    if (result != GraspResult::SUCCESS) return result;
    return ReleaseObject();
}

void MujocoGraspExecutor::EmergencyStop() {
    impl_->HoldCurrentState();
    RecordResult(GraspResult::SUCCESS, "emergency_stop",
                "motion stopped at current state");
}

bool MujocoGraspExecutor::GetCurrentPose(Pose3D& pose) {
    return impl_->GetHoldPose(pose);
}

void MujocoGraspExecutor::Tick(float dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0f) {
        return;
    }
    const int steps = std::max(
        1, static_cast<int>(std::round(
                dt_s / std::max(kMinFiniteDepth, impl_->model->opt.timestep))));
    for (int i = 0; i < steps; ++i) {
        mj_step(impl_->model, impl_->data);
        impl_->ForwardWithAttachments();
    }
}

}  // namespace perceptive_grasp
