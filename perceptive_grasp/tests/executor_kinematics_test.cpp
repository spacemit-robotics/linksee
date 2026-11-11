// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/** @file executor_kinematics_test.cpp
 * @brief Offline SO101 model compatibility and seeded/unseeded public FK/IK regression.
 */
#include "executor_kinematics.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using perceptive_grasp::ExecutorKinematics;
using perceptive_grasp::PlanningInverseOptions;
using perceptive_grasp::PlanningJoints;
using perceptive_grasp::PlanningPose;

namespace {
struct Reference {
    const char* name;
    std::array<double, 5> joints;
    PlanningPose pose;
    bool inverse_case = true;
};

// Recorded from the original calibration FK, independently cross-checked against
// the public URDF FK (maximum matrix-entry error 2.23e-16). No legacy library is linked.
const Reference kReferences[] = {
    {"home", {1.546, -1.765, 1.400, 1.093, 0.186},
        {0.04113966530568148, -0.1523663327558409, 0.06359298033431676,
            0.2725150566193417, 0.5908798217509171, 0.6953758651058546, -0.3050393194190636}, false},
    {"top_observe", {1.544, -0.067, 0.101, 1.382, 0.188},
        {0.04277606929542394, -0.2026157243511039, 0.072877870072402,
            0.0515170335987785, 0.644068801074437, 0.7610495099230845, -0.05766297059251085}},
    {"side_observe", {1.550, 0.021, 1.400, -1.700, -0.036},
        {0.04412727830741761, -0.2412154514846095, 0.1322095496846513,
            0.5936511432082151, 0.4389902324520424, 0.4118096958169239, -0.5341148475847247}, false},
    {"place", {-1.500, -0.122, 0.101, 1.383, 0.330},
        {0.055901412061057, 0.2041969594290715, 0.08244331847948524,
            -0.08545706787704897, 0.7732557298734588, -0.6254717621719655, -0.05964679784546784}},
    {"top_seed", {0, -0.3, 0.7, 1.3, 0},
        {0.1606427013253253, -1.014456542950936e-05, 0.02779141547330972,
            -0.06453329270760208, 0.02428345970810883, 0.9976188193042304, -0.001568147618389416}},
    {"home_ik", {1.546, -1.72, 1.4, 1.093, 0.186},
        {0.04107746727063918, -0.1498591156293858, 0.05815951319195949,
            0.2587685053477599, 0.5966892479382491, 0.7022126024504405, -0.2896519687396455}},
    {"side_ik", {1.55, 0.021, 1.4, -1.64, -0.036},
        {0.04416670293799722, -0.243110326383857, 0.1228352931041826,
            0.5799621936225414, 0.456262454993164, 0.4280123749517912, -0.5217986517999716}},
};

double PositionError(const PlanningPose& first, const PlanningPose& second) {
    return std::sqrt(std::pow(first.x - second.x, 2) + std::pow(first.y - second.y, 2) +
        std::pow(first.z - second.z, 2));
}

double RotationError(const PlanningPose& first, const PlanningPose& second) {
    const double dot = first.qw * second.qw + first.qx * second.qx +
        first.qy * second.qy + first.qz * second.qz;
    return 2 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
}

void CheckReference(ExecutorKinematics& solver, const Reference& reference) {
    PlanningJoints joints;
    joints.count = 5;
    std::copy(reference.joints.begin(), reference.joints.end(), joints.q);
    PlanningPose measured{};
    assert(solver.Forward(joints, &measured) == 0);
    assert(PositionError(measured, reference.pose) < 1e-10);
    assert(RotationError(measured, reference.pose) < 1e-7);
    if (!reference.inverse_case) {
        // Recorded measured HOME/SIDE positions exceed the nominal URDF bounds.
        // They remain valid FK observations, never permission to return an out-of-limit IK goal.
        PlanningJoints solution;
        PlanningInverseOptions options{0.5, 1e-5, 0.85};
        if (solver.Inverse(reference.pose, &joints, &options, &solution) == 0) {
            double lower[5], upper[5];
            assert(solver.Limits(lower, upper) == 0);
            for (int index = 0; index < 5; ++index)
                assert(solution.q[index] >= lower[index] && solution.q[index] <= upper[index]);
        }
        std::cout << reference.name << " measured FK compatible; bounded IK required\n";
        return;
    }
    // Perturbed seeds exercise iteration, not only the already-solved early return.
    for (int index = 0; index < 5; ++index) joints.q[index] += 0.005;
    for (double weight : {1.0, 0.85}) {
        PlanningInverseOptions options{0.5, 1e-5, weight};
        PlanningJoints solution;
        assert(solver.Inverse(reference.pose, &joints, &options, &solution) == 0);
        assert(solver.Forward(solution, &measured) == 0);
        const double position = PositionError(measured, reference.pose);
        const double rotation = RotationError(measured, reference.pose);
        assert(position < 2e-5);
        if (weight < 1) assert(rotation < 1e-4);
        std::cout << reference.name << " weight=" << weight << " position_error_m=" <<
            position << " rotation_error_rad=" << rotation << '\n';
    }
}
}  // namespace

int main() {
    ExecutorKinematics solver(PERCEPTIVE_GRASP_TEST_MODEL, "pinocchio");
    assert(solver.Count() == 5);  // The moving jaw is outside the selected TCP chain.
    for (const auto& reference : kReferences) CheckReference(solver, reference);

    PlanningJoints zero;
    zero.count = 5;
    PlanningPose target{};
    assert(solver.Forward(zero, &target) == 0);
    PlanningJoints solved;
    assert(solver.Inverse(target, nullptr, nullptr, &solved) == 0);
    assert(solved.count == 5);
    target.x = 10;
    assert(solver.Inverse(target, &zero, nullptr, &solved) != 0);
    zero.count = 6;
    assert(solver.Forward(zero, &target) != 0);
    assert(solver.Inverse(target, &zero, nullptr, &solved) != 0);
    zero.count = 5;
    zero.q[0] = std::numeric_limits<double>::quiet_NaN();
    assert(solver.Forward(zero, &target) != 0);
    bool rejected = false;
    try { ExecutorKinematics invalid(PERCEPTIVE_GRASP_TEST_MODEL, "pinocchio", {"wrong_joint"}); }
    catch (const std::exception&) { rejected = true; }
    assert(rejected);
    std::cout << "executor kinematics migration passed\n";
}
