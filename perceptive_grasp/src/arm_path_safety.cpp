/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file arm_path_safety.cpp
 * @brief URDF-based arm clearance checks for support surfaces.
 */

#include "arm_path_safety.h"

#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace perceptive_grasp {
namespace {

constexpr double kMinimumVectorNorm = 1e-9;

struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Transform {
    Vector3 translation;
    Quaternion rotation;
};

Vector3 Add(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vector3 Scale(const Vector3& vector, double scale) {
    return {vector.x * scale, vector.y * scale, vector.z * scale};
}

double Dot(const Vector3& lhs, const Vector3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double Norm(const Vector3& vector) {
    return std::sqrt(Dot(vector, vector));
}

Vector3 Normalize(const Vector3& vector) {
    const double norm = Norm(vector);
    if (norm < kMinimumVectorNorm) return {};
    return Scale(vector, 1.0 / norm);
}

Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs) {
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

Quaternion Normalize(const Quaternion& quaternion) {
    const double norm = std::sqrt(
        quaternion.w * quaternion.w + quaternion.x * quaternion.x +
        quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    if (norm < kMinimumVectorNorm) return {};
    return {
        quaternion.w / norm,
        quaternion.x / norm,
        quaternion.y / norm,
        quaternion.z / norm,
    };
}

Vector3 Rotate(const Quaternion& quaternion, const Vector3& vector) {
    const Quaternion normalized = Normalize(quaternion);
    const Quaternion pure{0.0, vector.x, vector.y, vector.z};
    const Quaternion inverse{
        normalized.w, -normalized.x, -normalized.y, -normalized.z};
    const Quaternion rotated = Multiply(Multiply(normalized, pure), inverse);
    return {rotated.x, rotated.y, rotated.z};
}

Transform Compose(const Transform& parent, const Transform& child) {
    return {
        Add(parent.translation, Rotate(parent.rotation, child.translation)),
        Normalize(Multiply(parent.rotation, child.rotation)),
    };
}

Transform FromUrdfPose(const urdf::Pose& pose) {
    return {
        {pose.position.x, pose.position.y, pose.position.z},
        {pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z},
    };
}

Transform JointMotion(const urdf::Joint& joint, double position) {
    Transform motion;
    const Vector3 axis = Normalize(
        Vector3{joint.axis.x, joint.axis.y, joint.axis.z});
    if (joint.type == urdf::Joint::REVOLUTE ||
        joint.type == urdf::Joint::CONTINUOUS) {
        const double half_angle = position * 0.5;
        const double sine = std::sin(half_angle);
        motion.rotation = Normalize({
            std::cos(half_angle), axis.x * sine, axis.y * sine, axis.z * sine});
    } else if (joint.type == urdf::Joint::PRISMATIC) {
        motion.translation = Scale(axis, position);
    }
    return motion;
}

double ProjectedRadius(const urdf::Geometry& geometry,
                        const Transform& collision_transform,
                        const Vector3& plane_normal) {
    if (geometry.type == urdf::Geometry::SPHERE) {
        const auto& sphere = static_cast<const urdf::Sphere&>(geometry);
        return sphere.radius;
    }

    if (geometry.type == urdf::Geometry::BOX) {
        const auto& box = static_cast<const urdf::Box&>(geometry);
        const Vector3 axis_x = Rotate(collision_transform.rotation, {1.0, 0.0, 0.0});
        const Vector3 axis_y = Rotate(collision_transform.rotation, {0.0, 1.0, 0.0});
        const Vector3 axis_z = Rotate(collision_transform.rotation, {0.0, 0.0, 1.0});
        return 0.5 * (
            std::fabs(Dot(plane_normal, axis_x)) * box.dim.x +
            std::fabs(Dot(plane_normal, axis_y)) * box.dim.y +
            std::fabs(Dot(plane_normal, axis_z)) * box.dim.z);
    }

    if (geometry.type == urdf::Geometry::CYLINDER) {
        const auto& cylinder = static_cast<const urdf::Cylinder&>(geometry);
        const Vector3 cylinder_axis = Rotate(
            collision_transform.rotation, {0.0, 0.0, 1.0});
        const double axial_projection = std::clamp(
            std::fabs(Dot(plane_normal, cylinder_axis)), 0.0, 1.0);
        const double radial_projection = std::sqrt(
            std::max(0.0, 1.0 - axial_projection * axial_projection));
        return 0.5 * cylinder.length * axial_projection +
            cylinder.radius * radial_projection;
    }

    return std::numeric_limits<double>::infinity();
}

std::string ClearanceDetail(const std::string& link_name,
                            double clearance_m,
                            double required_clearance_m) {
    std::ostringstream stream;
    stream << "link " << link_name << " clearance=" << clearance_m
            << "m below required=" << required_clearance_m << "m";
    return stream.str();
}

}  // namespace

struct ArmPathSafety::Impl {
    struct ChainEntry {
        urdf::JointConstSharedPtr joint;
        urdf::LinkConstSharedPtr child_link;
        int movable_joint_index = -1;
    };

    urdf::ModelInterfaceSharedPtr model;
    urdf::LinkConstSharedPtr base;
    std::vector<ChainEntry> chain;
    int movable_joint_count = 0;

    ArmPathSafetyResult CheckConfiguration(
        const std::vector<float>& joints,
        const SupportPlane& support_plane,
        float required_clearance_m) const {
        ArmPathSafetyResult result;
        if (!support_plane.valid) {
            result.detail = "support surface is unavailable";
            return result;
        }
        if (static_cast<int>(joints.size()) < movable_joint_count) {
            result.detail = "joint vector is shorter than the URDF chain";
            return result;
        }

        const Vector3 plane_normal = Normalize(Vector3{
            support_plane.normal_x,
            support_plane.normal_y,
            support_plane.normal_z});
        if (Norm(plane_normal) < kMinimumVectorNorm) {
            result.detail = "support surface normal is invalid";
            return result;
        }

        result.safe = true;
        result.minimum_clearance_m = std::numeric_limits<float>::infinity();
        Transform link_transform;
        bool collision_geometry_found = false;

        const auto check_link = [&](const urdf::LinkConstSharedPtr& link,
                                    const Transform& transform,
                                    ArmPathSafetyResult* output) {
            if (!link) return;
            const auto& collisions = link->collision_array;
            for (const urdf::CollisionSharedPtr& collision : collisions) {
                if (!collision || !collision->geometry) continue;
                collision_geometry_found = true;
                const Transform collision_transform = Compose(
                    transform, FromUrdfPose(collision->origin));
                const double radius = ProjectedRadius(
                    *collision->geometry, collision_transform, plane_normal);
                if (!std::isfinite(radius)) {
                    output->safe = false;
                    output->detail = "unsupported mesh collision on link " +
                        link->name;
                    return;
                }
                if (support_plane.bounds_valid) {
                    const double radius_x = ProjectedRadius(
                        *collision->geometry, collision_transform,
                        Vector3{1.0, 0.0, 0.0});
                    const double radius_y = ProjectedRadius(
                        *collision->geometry, collision_transform,
                        Vector3{0.0, 1.0, 0.0});
                    const double center_x =
                        collision_transform.translation.x;
                    const double center_y =
                        collision_transform.translation.y;
                    const bool overlaps_x =
                        center_x + radius_x >= support_plane.min_x &&
                        center_x - radius_x <= support_plane.max_x;
                    const bool overlaps_y =
                        center_y + radius_y >= support_plane.min_y &&
                        center_y - radius_y <= support_plane.max_y;
                    if (!overlaps_x || !overlaps_y) continue;
                }
                const double clearance = Dot(
                    plane_normal, collision_transform.translation) +
                    support_plane.d - radius;
                if (clearance < output->minimum_clearance_m) {
                    output->minimum_clearance_m = static_cast<float>(clearance);
                    output->link_name = link->name;
                }
            }
        };

        // The fixed base mount may intentionally touch the chassis support.
        // Only links downstream of a movable arm joint can sweep into it.
        for (const ChainEntry& entry : chain) {
            link_transform = Compose(
                link_transform,
                FromUrdfPose(entry.joint->parent_to_joint_origin_transform));
            if (entry.movable_joint_index >= 0) {
                link_transform = Compose(
                    link_transform,
                    JointMotion(
                        *entry.joint, joints[entry.movable_joint_index]));
            }
            check_link(entry.child_link, link_transform, &result);
            if (!result.safe) return result;
        }

        if (!std::isfinite(result.minimum_clearance_m)) {
            if (!collision_geometry_found) {
                result.safe = false;
                result.detail = "URDF chain has no collision geometry";
            }
            return result;
        }
        if (result.minimum_clearance_m + 1e-6f < required_clearance_m) {
            result.safe = false;
            result.detail = ClearanceDetail(
                result.link_name,
                result.minimum_clearance_m,
                required_clearance_m);
        }
        return result;
    }
};

ArmPathSafety::ArmPathSafety() : impl_(std::make_unique<Impl>()) {}

ArmPathSafety::~ArmPathSafety() = default;

bool ArmPathSafety::Init(const std::string& urdf_path,
                            const std::string& base_link,
                            const std::string& tip_link,
                            std::string* error) {
    impl_->model = urdf::parseURDFFile(urdf_path);
    if (!impl_->model) {
        if (error) *error = "failed to parse URDF: " + urdf_path;
        return false;
    }
    impl_->base = impl_->model->getLink(base_link);
    urdf::LinkConstSharedPtr current = impl_->model->getLink(tip_link);
    if (!impl_->base || !current) {
        if (error) *error = "URDF base or tip link is missing";
        return false;
    }

    std::vector<Impl::ChainEntry> reversed_chain;
    while (current && current->name != base_link) {
        const urdf::JointConstSharedPtr joint = current->parent_joint;
        if (!joint) {
            if (error) *error = "tip link is not a descendant of base link";
            return false;
        }
        reversed_chain.push_back({joint, current, -1});
        current = impl_->model->getLink(joint->parent_link_name);
    }
    if (!current) {
        if (error) *error = "tip link is not a descendant of base link";
        return false;
    }

    impl_->chain.assign(reversed_chain.rbegin(), reversed_chain.rend());
    impl_->movable_joint_count = 0;
    for (Impl::ChainEntry& entry : impl_->chain) {
        if (entry.joint->type == urdf::Joint::REVOLUTE ||
            entry.joint->type == urdf::Joint::CONTINUOUS ||
            entry.joint->type == urdf::Joint::PRISMATIC) {
            entry.movable_joint_index = impl_->movable_joint_count++;
        } else if (entry.joint->type != urdf::Joint::FIXED) {
            if (error) {
                *error = "unsupported joint type in arm chain: " +
                    entry.joint->name;
            }
            return false;
        }
    }
    if (impl_->movable_joint_count <= 0) {
        if (error) *error = "URDF arm chain has no movable joints";
        return false;
    }
    if (error) error->clear();
    return true;
}

ArmPathSafetyResult ArmPathSafety::CheckConfiguration(
    const std::vector<float>& joints,
    const SupportPlane& support_plane,
    float required_clearance_m) const {
    if (!impl_ || !impl_->model) {
        ArmPathSafetyResult result;
        result.detail = "arm path safety is not initialized";
        return result;
    }
    return impl_->CheckConfiguration(
        joints, support_plane, required_clearance_m);
}

ArmPathSafetyResult ArmPathSafety::CheckPath(
    const std::vector<float>& start_joints,
    const std::vector<float>& target_joints,
    const SupportPlane& support_plane,
    float required_clearance_m,
    float maximum_joint_step_rad) const {
    ArmPathSafetyResult result;
    if (start_joints.size() != target_joints.size() || start_joints.empty()) {
        result.detail = "path joint vectors have different sizes";
        return result;
    }
    if (maximum_joint_step_rad <= 0.0f) {
        result.detail = "maximum joint interpolation step is invalid";
        return result;
    }

    float maximum_delta = 0.0f;
    for (size_t index = 0; index < start_joints.size(); ++index) {
        maximum_delta = std::max(
            maximum_delta,
            std::fabs(target_joints[index] - start_joints[index]));
    }
    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(
                maximum_delta / maximum_joint_step_rad)));
    result.safe = true;
    result.minimum_clearance_m = std::numeric_limits<float>::infinity();
    std::vector<float> sample(start_joints.size());
    for (int sample_index = 0; sample_index <= sample_count; ++sample_index) {
        const float progress = static_cast<float>(sample_index) /
            static_cast<float>(sample_count);
        for (size_t joint = 0; joint < sample.size(); ++joint) {
            sample[joint] = start_joints[joint] +
                progress * (target_joints[joint] - start_joints[joint]);
        }
        ArmPathSafetyResult configuration = CheckConfiguration(
            sample, support_plane, required_clearance_m);
        if (configuration.minimum_clearance_m < result.minimum_clearance_m) {
            result.minimum_clearance_m = configuration.minimum_clearance_m;
            result.link_name = configuration.link_name;
            result.sample_index = sample_index;
        }
        if (!configuration.safe) {
            configuration.sample_index = sample_index;
            configuration.detail = "path sample " +
                std::to_string(sample_index) + "/" +
                std::to_string(sample_count) + ": " +
                configuration.detail;
            return configuration;
        }
    }
    return result;
}

}  // namespace perceptive_grasp
