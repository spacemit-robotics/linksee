/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file stereo_geometry.h
* @brief Stereo calibration and output-intrinsics helpers.
*/

#ifndef STEREO_GEOMETRY_H
#define STEREO_GEOMETRY_H

#include <string>

namespace perceptive_grasp {

/** Pinhole intrinsics for a rectified image. */
struct PinholeIntrinsics {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

/** Load rectified logical-left intrinsics from a stereo calibration file. */
bool LoadRectifiedLeftIntrinsics(const std::string& calibration_json,
                                PinholeIntrinsics& intrinsics,
                                std::string& error);

/** Scale rectified intrinsics to a letterboxed output image. */
bool ScaleIntrinsicsWithLetterbox(const PinholeIntrinsics& source,
                                int output_width,
                                int output_height,
                                PinholeIntrinsics& output,
                                std::string& error);

}  // namespace perceptive_grasp

#endif  // STEREO_GEOMETRY_H
