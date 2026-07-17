/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file stereo_geometry_test.cpp
 * @brief Unit tests for stereo calibration geometry helpers.
 */

#include "stereo_geometry.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool Near(double actual, double expected) {
    return std::abs(actual - expected) < 1e-9;
}

}  // namespace

int main() {
    perceptive_grasp::PinholeIntrinsics source;
    source.width = 1920;
    source.height = 1200;
    source.fx = 1200.0;
    source.fy = 1200.0;
    source.cx = 960.0;
    source.cy = 600.0;

    perceptive_grasp::PinholeIntrinsics output;
    std::string error;
    if (!perceptive_grasp::ScaleIntrinsicsWithLetterbox(
            source, 320, 256, output, error)) {
        std::cerr << error << std::endl;
        return 1;
    }

    // 1920x1200 scales to 320x200, leaving 28 rows above and below.
    if (output.width != 320 || output.height != 256 ||
        !Near(output.fx, 200.0) || !Near(output.fy, 200.0) ||
        !Near(output.cx, 160.0) || !Near(output.cy, 128.0)) {
        std::cerr << "unexpected letterboxed intrinsics" << std::endl;
        return 1;
    }

    if (perceptive_grasp::ScaleIntrinsicsWithLetterbox(
            source, 0, 256, output, error)) {
        std::cerr << "invalid output size was accepted" << std::endl;
        return 1;
    }

    std::cout << "stereo_geometry_test passed" << std::endl;
    return 0;
}
