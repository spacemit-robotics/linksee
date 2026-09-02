/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_geometry.cpp
 * @brief Lightweight mask-guided 3D object geometry and grasp candidates.
 */

#include "grasp_geometry.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace perceptive_grasp {

namespace {

constexpr float kEpsilon = 1e-6f;
constexpr float kMinimumGripperClearanceM = 0.001f;
constexpr float kForegroundDepthMarginMm = 30.0f;
// A wider range allows the background visible through hollow objects to
// dominate the fitted cloud. Graspable tabletop objects fit within 120 mm
// along the camera ray; larger depth gaps belong to the support/background.
constexpr float kMaximumObjectDepthSpanMm = 120.0f;
constexpr float kSilhouetteDepthSpanMm = 45.0f;
constexpr float kSupportDepthBeforeObjectMm = 100.0f;
constexpr float kSupportDepthBehindObjectMm = 500.0f;
constexpr float kLockedSideHysteresisRatio = 0.85f;

bool WithinDimensionTolerance(float reference,
                                float current,
                                float absolute_tolerance,
                                float relative_tolerance) {
    const float tolerance = std::max(
        absolute_tolerance,
        relative_tolerance * std::max(reference, current));
    return std::fabs(reference - current) <= tolerance;
}

cv::Point3f Add(const cv::Point3f& a, const cv::Point3f& b) {
    return cv::Point3f(a.x + b.x, a.y + b.y, a.z + b.z);
}

cv::Point3f Subtract(const cv::Point3f& a, const cv::Point3f& b) {
    return cv::Point3f(a.x - b.x, a.y - b.y, a.z - b.z);
}

cv::Point3f Scale(const cv::Point3f& point, float scale) {
    return cv::Point3f(point.x * scale, point.y * scale, point.z * scale);
}

float Dot(const cv::Point3f& a, const cv::Point3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

cv::Point3f Cross(const cv::Point3f& a, const cv::Point3f& b) {
    return cv::Point3f(a.y * b.z - a.z * b.y,
                        a.z * b.x - a.x * b.z,
                        a.x * b.y - a.y * b.x);
}

float Norm(const cv::Point3f& point) {
    return std::sqrt(Dot(point, point));
}

cv::Point3f Normalize(const cv::Point3f& point) {
    const float norm = Norm(point);
    if (norm < kEpsilon) return cv::Point3f();
    return Scale(point, 1.0f / norm);
}

float Clamp(float value, float lower, float upper) {
    return std::max(lower, std::min(value, upper));
}

float HeightToFootprintRatio(const ObjectGeometry3D& geometry) {
    return geometry.height_m /
        std::max(geometry.length_m, kEpsilon);
}

bool IsClearlyHorizontal(const ObjectGeometry3D& geometry,
                            const GraspGeometryConfig& config) {
    return geometry.valid &&
        HeightToFootprintRatio(geometry) <
            config.side_min_height_width_ratio *
                kLockedSideHysteresisRatio;
}

float Percentile(std::vector<float> values, float quantile) {
    if (values.empty()) return NAN;
    quantile = Clamp(quantile, 0.0f, 1.0f);
    const size_t index = static_cast<size_t>(
        std::lround(quantile * static_cast<float>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

bool IsFinitePoint(const cv::Point3f& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z);
}

float MedianDepthAround(const cv::Mat& depth,
                        const cv::Point2f& center,
                        int radius) {
    const int center_x = std::clamp(
        static_cast<int>(std::lround(center.x)), 0, depth.cols - 1);
    const int center_y = std::clamp(
        static_cast<int>(std::lround(center.y)), 0, depth.rows - 1);
    std::vector<float> values;
    for (int y = std::max(0, center_y - radius);
        y <= std::min(depth.rows - 1, center_y + radius); ++y) {
        for (int x = std::max(0, center_x - radius);
            x <= std::min(depth.cols - 1, center_x + radius); ++x) {
            const uint16_t value = depth.at<uint16_t>(y, x);
            if (value != 0) values.push_back(value);
        }
    }
    return Percentile(values, 0.50f);
}

float MedianMaskBoundaryDepth(const cv::Mat& depth,
    const cv::Mat& mask) {
    cv::Mat interior;
    cv::erode(mask, interior,
        cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::Mat boundary;
    cv::subtract(mask, interior, boundary);

    std::vector<float> values;
    values.reserve(static_cast<size_t>(cv::countNonZero(boundary)));
    for (int y = 0; y < boundary.rows; ++y) {
        const uint8_t* boundary_row = boundary.ptr<uint8_t>(y);
        const uint16_t* depth_row = depth.ptr<uint16_t>(y);
        for (int x = 0; x < boundary.cols; ++x) {
            if (boundary_row[x] != 0 && depth_row[x] != 0) {
                values.push_back(static_cast<float>(depth_row[x]));
            }
        }
    }
    if (values.size() < 16) return NAN;
    return Percentile(values, 0.50f);
}

cv::Mat NormalizeTargetMask(const DetectionTarget& target,
                            const cv::Size& image_size) {
    cv::Mat mask;
    if (!target.mask.empty()) {
        cv::Mat single_channel;
        if (target.mask.channels() == 1) {
            single_channel = target.mask;
        } else {
            cv::cvtColor(target.mask, single_channel, cv::COLOR_BGR2GRAY);
        }

        double max_value = 0.0;
        cv::minMaxLoc(single_channel, nullptr, &max_value);
        const double scale = max_value <= 1.0 ? 255.0 : 1.0;
        single_channel.convertTo(mask, CV_8UC1, scale);
        if (mask.size() != image_size) {
            cv::resize(mask, mask, image_size, 0.0, 0.0, cv::INTER_NEAREST);
        }
        cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);
    }

    if (mask.empty() || cv::countNonZero(mask) < 16) mask.release();
    return mask;
}

int AdaptiveStride(const cv::Mat& mask, const GraspGeometryConfig& config) {
    const int point_count = std::max(1, cv::countNonZero(mask));
    const float ratio = static_cast<float>(point_count) /
        static_cast<float>(std::max(1, config.max_object_points));
    const int capacity_stride = static_cast<int>(std::ceil(std::sqrt(ratio)));
    return std::max(1, std::max(config.sample_stride, capacity_stride));
}

std::vector<cv::Point> SamplePixels(const cv::Mat& mask, int stride) {
    std::vector<cv::Point> pixels;
    for (int y = 0; y < mask.rows; y += stride) {
        const uint8_t* row = mask.ptr<uint8_t>(y);
        for (int x = 0; x < mask.cols; x += stride) {
            if (row[x] != 0) pixels.emplace_back(x, y);
        }
    }
    return pixels;
}

bool DeprojectPixels(const std::vector<cv::Point>& pixels,
                    const cv::Mat& depth,
                    const StereoCamera& camera,
                    const GraspPlanner& coordinate_planner,
                    float min_depth_mm,
                    float max_depth_mm,
                    std::vector<cv::Point3f>& points) {
    points.clear();
    points.reserve(pixels.size());
    for (const cv::Point& pixel : pixels) {
        const uint16_t depth_mm = depth.at<uint16_t>(pixel.y, pixel.x);
        if (depth_mm == 0 || depth_mm < min_depth_mm ||
            depth_mm > max_depth_mm) {
            continue;
        }
        float camera_point[3] = {};
        if (!camera.Deproject(pixel.x, pixel.y, depth_mm, camera_point)) {
            continue;
        }
        float base_point[3] = {};
        coordinate_planner.CameraToBase(camera_point, base_point);
        cv::Point3f point(base_point[0], base_point[1], base_point[2]);
        if (IsFinitePoint(point)) points.push_back(point);
    }
    return !points.empty();
}

void SetTableBounds(const std::vector<cv::Point3f>& points,
                    TablePlane& table) {
    if (points.empty()) return;
    std::vector<float> values_x;
    std::vector<float> values_y;
    values_x.reserve(points.size());
    values_y.reserve(points.size());
    for (const cv::Point3f& point : points) {
        if (!IsFinitePoint(point)) continue;
        values_x.push_back(point.x);
        values_y.push_back(point.y);
    }
    if (values_x.size() < 8) return;
    table.min_x = Percentile(values_x, 0.02f);
    table.max_x = Percentile(values_x, 0.98f);
    table.min_y = Percentile(values_y, 0.02f);
    table.max_y = Percentile(values_y, 0.98f);
    table.bounds_valid = table.min_x < table.max_x &&
        table.min_y < table.max_y;
}

bool FitTablePlane(const std::vector<cv::Point3f>& points,
                    const GraspGeometryConfig& config,
                    TablePlane& table) {
    if (points.size() < 30) return false;

    const float min_normal_z = std::cos(
        config.max_table_tilt_deg * static_cast<float>(CV_PI) / 180.0f);
    std::mt19937 random(20260722);
    std::uniform_int_distribution<size_t> index(0, points.size() - 1);
    int best_count = 0;
    cv::Point3f best_normal;
    float best_d = 0.0f;

    for (int iteration = 0; iteration < config.plane_ransac_iterations;
        ++iteration) {
        const cv::Point3f& p0 = points[index(random)];
        const cv::Point3f& p1 = points[index(random)];
        const cv::Point3f& p2 = points[index(random)];
        cv::Point3f normal = Normalize(
            Cross(Subtract(p1, p0), Subtract(p2, p0)));
        if (Norm(normal) < kEpsilon) continue;
        if (normal.z < 0.0f) normal = Scale(normal, -1.0f);
        if (normal.z < min_normal_z) continue;

        const float d = -Dot(normal, p0);
        int inlier_count = 0;
        for (const cv::Point3f& point : points) {
            if (std::fabs(Dot(normal, point) + d) <=
                config.plane_distance_threshold_m) {
                ++inlier_count;
            }
        }
        if (inlier_count > best_count) {
            best_count = inlier_count;
            best_normal = normal;
            best_d = d;
        }
    }

    const int required_inliers = std::max(20, static_cast<int>(points.size() / 8));
    if (best_count < required_inliers) return false;

    std::vector<cv::Point3f> inliers;
    inliers.reserve(best_count);
    for (const cv::Point3f& point : points) {
        if (std::fabs(Dot(best_normal, point) + best_d) <=
            config.plane_distance_threshold_m) {
            inliers.push_back(point);
        }
    }

    cv::Mat samples(static_cast<int>(inliers.size()), 3, CV_32F);
    for (int row = 0; row < samples.rows; ++row) {
        samples.at<float>(row, 0) = inliers[row].x;
        samples.at<float>(row, 1) = inliers[row].y;
        samples.at<float>(row, 2) = inliers[row].z;
    }
    cv::PCA pca(samples, cv::Mat(), cv::PCA::DATA_AS_ROW);
    cv::Point3f normal(pca.eigenvectors.at<float>(2, 0),
                        pca.eigenvectors.at<float>(2, 1),
                        pca.eigenvectors.at<float>(2, 2));
    normal = Normalize(normal);
    if (normal.z < 0.0f) normal = Scale(normal, -1.0f);
    if (normal.z < min_normal_z) return false;

    const cv::Point3f center(pca.mean.at<float>(0, 0),
                            pca.mean.at<float>(0, 1),
                            pca.mean.at<float>(0, 2));
    table.normal = normal;
    table.d = -Dot(normal, center);
    table.inlier_count = static_cast<int>(inliers.size());
    SetTableBounds(inliers, table);
    return true;
}

bool FitHorizontalSupportPlane(const std::vector<cv::Point3f>& points,
                                const GraspGeometryConfig& config,
                                TablePlane& table) {
    if (points.size() < 30) return false;
    std::vector<float> heights;
    heights.reserve(points.size());
    for (const cv::Point3f& point : points) {
        if (IsFinitePoint(point)) heights.push_back(point.z);
    }
    if (heights.size() < 30) return false;

    const float table_z = Percentile(heights, 0.50f);
    const float threshold = 2.0f * config.plane_distance_threshold_m;
    std::vector<cv::Point3f> inliers;
    inliers.reserve(points.size());
    for (const cv::Point3f& point : points) {
        if (std::fabs(point.z - table_z) <= threshold) {
            inliers.push_back(point);
        }
    }
    const int required_inliers = std::max(
        20, static_cast<int>(heights.size() / 10));
    if (static_cast<int>(inliers.size()) < required_inliers) return false;

    table.normal = cv::Point3f(0.0f, 0.0f, 1.0f);
    table.d = -table_z;
    table.inlier_count = static_cast<int>(inliers.size());
    SetTableBounds(inliers, table);
    return true;
}

void PlaneBasis(const cv::Point3f& normal,
                cv::Point3f& axis_u,
                cv::Point3f& axis_v) {
    axis_u = Subtract(cv::Point3f(1.0f, 0.0f, 0.0f),
                        Scale(normal, normal.x));
    if (Norm(axis_u) < 0.2f) {
        axis_u = Subtract(cv::Point3f(0.0f, 1.0f, 0.0f),
                            Scale(normal, normal.y));
    }
    axis_u = Normalize(axis_u);
    axis_v = Normalize(Cross(normal, axis_u));
}

bool BaseCameraRay(const StereoCamera& camera,
                    const GraspPlanner& coordinate_planner,
                    int pixel_x,
                    int pixel_y,
                    cv::Point3f& origin,
                    cv::Point3f& direction) {
    float camera_origin[3] = {0.0f, 0.0f, 0.0f};
    float camera_point[3] = {0.0f, 0.0f, 0.0f};
    if (!camera.Deproject(
            pixel_x, pixel_y, 1000, camera_point)) {
        return false;
    }
    float base_origin[3] = {0.0f, 0.0f, 0.0f};
    float base_point[3] = {0.0f, 0.0f, 0.0f};
    coordinate_planner.CameraToBase(camera_origin, base_origin);
    coordinate_planner.CameraToBase(camera_point, base_point);
    origin = cv::Point3f(
        base_origin[0], base_origin[1], base_origin[2]);
    direction = Normalize(Subtract(
        cv::Point3f(base_point[0], base_point[1], base_point[2]),
        origin));
    return Norm(direction) >= kEpsilon;
}

bool IntersectRayWithPlane(const cv::Point3f& origin,
                            const cv::Point3f& direction,
                            const TablePlane& plane,
                            cv::Point3f& intersection) {
    const float denominator = Dot(plane.normal, direction);
    if (std::fabs(denominator) < kEpsilon) return false;
    const float distance =
        -(Dot(plane.normal, origin) + plane.d) / denominator;
    if (distance <= 0.0f) return false;
    intersection = Add(origin, Scale(direction, distance));
    return IsFinitePoint(intersection);
}

bool IntersectRayWithVerticalLine(
    const cv::Point3f& origin,
    const cv::Point3f& direction,
    const cv::Point3f& table_point,
    const cv::Point3f& table_normal,
    float& height,
    float& residual) {
    const cv::Point3f offset = Subtract(table_point, origin);
    const float direction_normal = Dot(direction, table_normal);
    const float determinant =
        1.0f - direction_normal * direction_normal;
    if (determinant < 1e-4f) return false;

    const float rhs_direction = Dot(direction, offset);
    const float rhs_normal = -Dot(table_normal, offset);
    const float ray_distance =
        (rhs_direction + direction_normal * rhs_normal) / determinant;
    height =
        (direction_normal * rhs_direction + rhs_normal) / determinant;
    if (ray_distance <= 0.0f || height <= 0.0f) return false;

    const cv::Point3f ray_point =
        Add(origin, Scale(direction, ray_distance));
    const cv::Point3f vertical_point =
        Add(table_point, Scale(table_normal, height));
    residual = Norm(Subtract(ray_point, vertical_point));
    return std::isfinite(height) && std::isfinite(residual);
}

bool MaskRowCenter(const cv::Mat& mask,
                    int first_y,
                    int last_y,
                    int& center_x) {
    std::vector<float> values;
    for (int y = std::max(0, first_y);
        y <= std::min(mask.rows - 1, last_y); ++y) {
        const uint8_t* row = mask.ptr<uint8_t>(y);
        for (int x = 0; x < mask.cols; ++x) {
            if (row[x] != 0) values.push_back(static_cast<float>(x));
        }
    }
    if (values.empty()) return false;
    center_x = static_cast<int>(std::lround(Percentile(values, 0.50f)));
    return true;
}

bool MaskRowExtent(const cv::Mat& mask,
                    int center_y,
                    int radius,
                    int& left_x,
                    int& right_x) {
    std::vector<float> values;
    for (int y = std::max(0, center_y - radius);
        y <= std::min(mask.rows - 1, center_y + radius); ++y) {
        const uint8_t* row = mask.ptr<uint8_t>(y);
        for (int x = 0; x < mask.cols; ++x) {
            if (row[x] != 0) values.push_back(static_cast<float>(x));
        }
    }
    if (values.size() < 8) return false;
    left_x = static_cast<int>(std::lround(Percentile(values, 0.02f)));
    right_x = static_cast<int>(std::lround(Percentile(values, 0.98f)));
    return right_x - left_x >= 4;
}

bool EstimateUprightContainerFromSilhouette(
    const cv::Mat& target_mask,
    const std::vector<cv::Point3f>& support_points,
    const StereoCamera& camera,
    const GraspPlanner& coordinate_planner,
    const GraspGeometryConfig& config,
    ObjectGeometry3D& geometry,
    std::vector<cv::Point3f>& object_points,
    std::string& error) {
    TablePlane table;
    if (!FitTablePlane(support_points, config, table) &&
        !FitHorizontalSupportPlane(support_points, config, table)) {
        error = "support table plane was not found";
        return false;
    }

    const std::vector<cv::Point> mask_pixels = SamplePixels(target_mask, 1);
    if (mask_pixels.size() < 32) {
        error = "container silhouette is too small";
        return false;
    }
    std::vector<float> mask_rows;
    mask_rows.reserve(mask_pixels.size());
    for (const cv::Point& pixel : mask_pixels) {
        mask_rows.push_back(static_cast<float>(pixel.y));
    }
    const int top_y = static_cast<int>(
        std::lround(Percentile(mask_rows, 0.02f)));
    const int bottom_y = static_cast<int>(
        std::lround(Percentile(mask_rows, 0.98f)));
    if (bottom_y - top_y < 20) {
        error = "container silhouette height is too small";
        return false;
    }

    int bottom_x = 0;
    int top_x = 0;
    const int row_band = std::max(2, (bottom_y - top_y) / 30);
    if (!MaskRowCenter(
            target_mask, bottom_y - row_band, bottom_y, bottom_x) ||
        !MaskRowCenter(
            target_mask, top_y, top_y + row_band, top_x)) {
        error = "container silhouette endpoints are unavailable";
        return false;
    }

    cv::Point3f bottom_origin;
    cv::Point3f bottom_direction;
    cv::Point3f top_origin;
    cv::Point3f top_direction;
    if (!BaseCameraRay(
            camera, coordinate_planner, bottom_x, bottom_y,
            bottom_origin, bottom_direction) ||
        !BaseCameraRay(
            camera, coordinate_planner, top_x, top_y,
            top_origin, top_direction)) {
        error = "container silhouette rays could not be deprojected";
        return false;
    }

    cv::Point3f table_center;
    if (!IntersectRayWithPlane(
            bottom_origin, bottom_direction, table, table_center)) {
        error = "container bottom ray does not intersect the table";
        return false;
    }
    float height_m = 0.0f;
    float vertical_residual_m = 0.0f;
    if (!IntersectRayWithVerticalLine(
            top_origin, top_direction, table_center, table.normal,
            height_m, vertical_residual_m) ||
        height_m < 0.030f ||
        height_m > config.object_max_height_m ||
        vertical_residual_m > 0.050f) {
        std::ostringstream diagnostic;
        diagnostic << "container height could not be recovered"
                    << " (height=" << height_m
                    << "m, residual=" << vertical_residual_m << "m)";
        error = diagnostic.str();
        return false;
    }

    constexpr float kWidthHeightRatio = 0.75f;
    const int width_y = static_cast<int>(std::lround(
        bottom_y - kWidthHeightRatio *
            static_cast<float>(bottom_y - top_y)));
    int left_x = 0;
    int right_x = 0;
    if (!MaskRowExtent(target_mask, width_y, row_band, left_x, right_x)) {
        error = "container silhouette width is unavailable";
        return false;
    }
    if (!config.side_single_sided_gripper) {
        const int center_x = static_cast<int>(std::lround(
            static_cast<float>(bottom_x) + kWidthHeightRatio *
                static_cast<float>(top_x - bottom_x)));
        const int symmetric_half_width = std::min(
            center_x - left_x, right_x - center_x);
        if (symmetric_half_width < 2) {
            error = "container silhouette body width is unavailable";
            return false;
        }
        // Simulation uses a symmetric parallel gripper. Ignore a handle or
        // spout that extends only one side of the rendered silhouette.
        left_x = center_x - symmetric_half_width;
        right_x = center_x + symmetric_half_width;
    }
    cv::Point3f left_origin;
    cv::Point3f left_direction;
    cv::Point3f right_origin;
    cv::Point3f right_direction;
    if (!BaseCameraRay(
            camera, coordinate_planner, left_x, width_y,
            left_origin, left_direction) ||
        !BaseCameraRay(
            camera, coordinate_planner, right_x, width_y,
            right_origin, right_direction)) {
        error = "container width rays could not be deprojected";
        return false;
    }

    TablePlane width_plane = table;
    const cv::Point3f width_center = Add(
        table_center,
        Scale(table.normal, height_m * kWidthHeightRatio));
    width_plane.d = -Dot(width_plane.normal, width_center);
    cv::Point3f left_point;
    cv::Point3f right_point;
    if (!IntersectRayWithPlane(
            left_origin, left_direction, width_plane, left_point) ||
        !IntersectRayWithPlane(
            right_origin, right_direction, width_plane, right_point)) {
        error = "container width rays do not intersect the height plane";
        return false;
    }
    const float width_m = Norm(Subtract(right_point, left_point));
    if (width_m < 0.025f || width_m > 0.150f) {
        std::ostringstream diagnostic;
        diagnostic << "container width is outside the safe range"
                    << " (width=" << width_m << "m)";
        error = diagnostic.str();
        return false;
    }

    cv::Point3f minor_axis = Normalize(Subtract(right_point, left_point));
    minor_axis = Normalize(Subtract(
        minor_axis, Scale(table.normal, Dot(minor_axis, table.normal))));
    cv::Point3f major_axis = Normalize(Cross(minor_axis, table.normal));
    if (major_axis.x < 0.0f) major_axis = Scale(major_axis, -1.0f);
    if (minor_axis.y < 0.0f) minor_axis = Scale(minor_axis, -1.0f);

    geometry = ObjectGeometry3D{};
    geometry.valid = true;
    geometry.table = table;
    geometry.table_center = table_center;
    geometry.center = Add(
        table_center, Scale(table.normal, height_m * 0.5f));
    geometry.major_axis = major_axis;
    geometry.minor_axis = minor_axis;
    geometry.length_m = width_m;
    geometry.width_m = width_m;
    geometry.height_m = height_m;

    object_points.clear();
    constexpr int kCircleSamples = 24;
    constexpr int kHeightSamples = 4;
    const float radius_m = width_m * 0.5f;
    for (int height_index = 1;
        height_index <= kHeightSamples; ++height_index) {
        const float sample_height = height_m *
            static_cast<float>(height_index) /
            static_cast<float>(kHeightSamples);
        for (int angle_index = 0;
            angle_index < kCircleSamples; ++angle_index) {
            const float angle = 2.0f * static_cast<float>(CV_PI) *
                static_cast<float>(angle_index) /
                static_cast<float>(kCircleSamples);
            cv::Point3f point = Add(
                table_center, Scale(table.normal, sample_height));
            point = Add(
                point, Scale(major_axis, radius_m * std::cos(angle)));
            point = Add(
                point, Scale(minor_axis, radius_m * std::sin(angle)));
            object_points.push_back(point);
        }
    }
    geometry.source_point_count = static_cast<int>(mask_pixels.size());
    geometry.object_point_count = static_cast<int>(object_points.size());
    error.clear();
    return true;
}

Pose3D PoseAt(const cv::Point3f& point,
                float qw,
                float qx,
                float qy,
                float qz) {
    return Pose3D{point.x, point.y, point.z, qw, qx, qy, qz};
}

void QuaternionFromAxes(const cv::Point3f& axis_x,
                        const cv::Point3f& axis_y,
                        const cv::Point3f& axis_z,
                        float& qw,
                        float& qx,
                        float& qy,
                        float& qz) {
    const float r00 = axis_x.x;
    const float r01 = axis_y.x;
    const float r02 = axis_z.x;
    const float r10 = axis_x.y;
    const float r11 = axis_y.y;
    const float r12 = axis_z.y;
    const float r20 = axis_x.z;
    const float r21 = axis_y.z;
    const float r22 = axis_z.z;
    const float trace = r00 + r11 + r22;

    if (trace > 0.0f) {
        const float scale = std::sqrt(trace + 1.0f) * 2.0f;
        qw = 0.25f * scale;
        qx = (r21 - r12) / scale;
        qy = (r02 - r20) / scale;
        qz = (r10 - r01) / scale;
    } else if (r00 > r11 && r00 > r22) {
        const float scale = std::sqrt(1.0f + r00 - r11 - r22) * 2.0f;
        qw = (r21 - r12) / scale;
        qx = 0.25f * scale;
        qy = (r01 + r10) / scale;
        qz = (r02 + r20) / scale;
    } else if (r11 > r22) {
        const float scale = std::sqrt(1.0f + r11 - r00 - r22) * 2.0f;
        qw = (r02 - r20) / scale;
        qx = (r01 + r10) / scale;
        qy = 0.25f * scale;
        qz = (r12 + r21) / scale;
    } else {
        const float scale = std::sqrt(1.0f + r22 - r00 - r11) * 2.0f;
        qw = (r10 - r01) / scale;
        qx = (r02 + r20) / scale;
        qy = (r12 + r21) / scale;
        qz = 0.25f * scale;
    }

    const float norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > kEpsilon) {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }
}

float WidthAlong(const std::vector<cv::Point3f>& points,
                const cv::Point3f& axis) {
    if (points.empty()) return 0.0f;
    std::vector<float> projections;
    projections.reserve(points.size());
    for (const cv::Point3f& point : points) {
        projections.push_back(Dot(point, axis));
    }
    return std::max(0.0f,
                    Percentile(projections, 0.98f) -
                    Percentile(projections, 0.02f));
}

float FullExtentAlong(const std::vector<cv::Point3f>& points,
                        const cv::Point3f& axis) {
    if (points.empty()) return 0.0f;
    float minimum = Dot(points.front(), axis);
    float maximum = minimum;
    for (const cv::Point3f& point : points) {
        const float projection = Dot(point, axis);
        minimum = std::min(minimum, projection);
        maximum = std::max(maximum, projection);
    }
    return std::max(0.0f, maximum - minimum);
}

struct TopGraspSection {
    cv::Point3f table_center;
    cv::Point3f opening_axis;
    float width_m = 0.0f;
    float height_m = 0.0f;
    bool localized = false;
};

TopGraspSection FindTopGraspSection(
    const ObjectGeometry3D& geometry,
    const std::vector<cv::Point3f>& object_points,
    const GraspGeometryConfig& config) {
    TopGraspSection best{
        geometry.table_center,
        geometry.minor_axis,
        geometry.width_m,
        geometry.height_m,
        false};
    if (geometry.height_m >= config.side_min_height_m ||
        geometry.length_m /
            std::max(geometry.width_m, kEpsilon) < 1.15f ||
        object_points.size() < 40) {
        return best;
    }

    std::vector<float> major_values;
    major_values.reserve(object_points.size());
    for (const cv::Point3f& point : object_points) {
        major_values.push_back(Dot(point, geometry.major_axis));
    }
    const float major_min = Percentile(major_values, 0.10f);
    const float major_max = Percentile(major_values, 0.90f);
    const float major_span = major_max - major_min;
    if (major_span < 0.04f) return best;

    const float radius = Clamp(geometry.width_m * 0.45f, 0.030f, 0.045f);
    float best_cost = std::numeric_limits<float>::max();
    constexpr float fractions[] = {0.25f, 0.375f, 0.50f, 0.625f, 0.75f};
    for (const float fraction : fractions) {
        const float seed_major = major_min + fraction * major_span;
        std::vector<float> nearby_minor;
        for (const cv::Point3f& point : object_points) {
            if (std::fabs(Dot(point, geometry.major_axis) - seed_major) <=
                radius * 0.35f) {
                nearby_minor.push_back(Dot(point, geometry.minor_axis));
            }
        }
        if (nearby_minor.size() < 12) continue;
        const float seed_minor = Percentile(nearby_minor, 0.50f);

        std::vector<cv::Point3f> local_points;
        local_points.reserve(object_points.size());
        for (const cv::Point3f& point : object_points) {
            const float delta_major =
                Dot(point, geometry.major_axis) - seed_major;
            const float delta_minor =
                Dot(point, geometry.minor_axis) - seed_minor;
            if (delta_major * delta_major + delta_minor * delta_minor <=
                radius * radius) {
                local_points.push_back(point);
            }
        }
        if (local_points.size() < 30) continue;

        cv::Mat samples(static_cast<int>(local_points.size()), 2, CV_32F);
        for (int row = 0; row < samples.rows; ++row) {
            samples.at<float>(row, 0) =
                Dot(local_points[row], geometry.major_axis);
            samples.at<float>(row, 1) =
                Dot(local_points[row], geometry.minor_axis);
        }
        cv::PCA pca(samples, cv::Mat(), cv::PCA::DATA_AS_ROW);
        const float secondary_variance = pca.eigenvalues.at<float>(1, 0);
        if (secondary_variance < kEpsilon ||
            pca.eigenvalues.at<float>(0, 0) / secondary_variance < 1.5f) {
            continue;
        }
        cv::Point3f tangent = Normalize(Add(
            Scale(geometry.major_axis, pca.eigenvectors.at<float>(0, 0)),
            Scale(geometry.minor_axis, pca.eigenvectors.at<float>(0, 1))));
        cv::Point3f opening = Normalize(Cross(geometry.table.normal, tangent));
        if (Norm(tangent) < kEpsilon || Norm(opening) < kEpsilon) continue;
        if (std::fabs(Dot(tangent, geometry.major_axis)) > 0.97f) continue;
        if (opening.y < 0.0f ||
            (std::fabs(opening.y) < kEpsilon && opening.x < 0.0f)) {
            opening = Scale(opening, -1.0f);
        }

        std::vector<float> tangent_values;
        std::vector<float> opening_values;
        std::vector<float> heights;
        tangent_values.reserve(local_points.size());
        opening_values.reserve(local_points.size());
        heights.reserve(local_points.size());
        for (const cv::Point3f& point : local_points) {
            tangent_values.push_back(Dot(point, tangent));
            opening_values.push_back(Dot(point, opening));
            heights.push_back(
                Dot(geometry.table.normal, point) + geometry.table.d);
        }
        const float tangent_extent =
            Percentile(tangent_values, 0.95f) -
            Percentile(tangent_values, 0.05f);
        const float local_width =
            Percentile(opening_values, 0.95f) -
            Percentile(opening_values, 0.05f);
        if (tangent_extent < 0.025f || local_width < 0.008f) continue;

        const float center_tangent = Percentile(tangent_values, 0.50f);
        const float center_opening = Percentile(opening_values, 0.50f);
        const cv::Point3f table_center = Add(
            Add(Scale(tangent, center_tangent),
                Scale(opening, center_opening)),
            Scale(geometry.table.normal, -geometry.table.d));
        const float height = Percentile(heights, 0.90f);
        const float center_penalty = 0.01f * std::fabs(fraction - 0.50f);
        const float cost = local_width + center_penalty;
        if (cost < best_cost) {
            best = TopGraspSection{
                table_center, opening, local_width, height, true};
            best_cost = cost;
        }
    }

    if (best.localized && best.width_m + 0.005f >= geometry.width_m) {
        return TopGraspSection{
            geometry.table_center,
            geometry.minor_axis,
            geometry.width_m,
            geometry.height_m,
            false};
    }
    return best;
}

void Reject(GraspCandidate& candidate, const std::string& reason) {
    candidate.geometry_valid = false;
    candidate.rejection_reason = reason;
}

GraspCandidate MakeSideCandidate(
    const ObjectGeometry3D& geometry,
    const std::vector<cv::Point3f>& object_points,
    const cv::Point3f& requested_approach,
    const cv::Point3f& requested_opening,
    const GraspGeometryConfig& config,
    float score) {
    GraspCandidate candidate;
    candidate.strategy = GraspStrategy::SIDE;
    candidate.score = score;

    cv::Point3f approach = Normalize(
        cv::Point3f(requested_approach.x, requested_approach.y, 0.0f));
    cv::Point3f opening = requested_opening;
    opening.z = 0.0f;
    opening = Subtract(opening, Scale(approach, Dot(opening, approach)));
    opening = Normalize(opening);
    if (Norm(approach) < kEpsilon || Norm(opening) < kEpsilon) {
        Reject(candidate, "invalid side approach axes");
        return candidate;
    }
    if (opening.y < 0.0f ||
        (std::fabs(opening.y) < kEpsilon && opening.x < 0.0f)) {
        opening = Scale(opening, -1.0f);
    }
    candidate.approach_axis = approach;
    candidate.opening_axis = opening;
    const float object_width_m = WidthAlong(object_points, opening);
    candidate.required_width_m = object_width_m +
        2.0f * config.footprint_padding_m;

    const float minimum_center_height = config.table_clearance_m +
        config.finger_half_height_m;
    const float maximum_center_height = geometry.height_m -
        config.finger_half_height_m;
    if (maximum_center_height <= minimum_center_height) {
        Reject(candidate, "object is too short for side finger clearance");
        return candidate;
    }

    const float center_height = Clamp(
        geometry.height_m * config.side_grasp_height_ratio,
        minimum_center_height, maximum_center_height);
    cv::Point3f grasp_point = Add(
        geometry.table_center, Scale(geometry.table.normal, center_height));
    cv::Point3f visible_surface_axis = Normalize(
        cv::Point3f(geometry.center.x, geometry.center.y, 0.0f));
    if (Norm(visible_surface_axis) >= kEpsilon) {
        grasp_point = Add(
            grasp_point,
            Scale(
                visible_surface_axis,
                config.side_visible_surface_offset_m));
    }
    const float fixed_jaw_offset_m = config.side_gripper_offset_m +
        (config.side_single_sided_gripper ? 0.5f * object_width_m : 0.0f);
    grasp_point = Add(grasp_point,
                        Scale(opening, fixed_jaw_offset_m));

    const cv::Point3f axis_z = approach;
    const cv::Point3f axis_y = opening;
    const cv::Point3f axis_x = Normalize(Cross(axis_y, axis_z));
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    QuaternionFromAxes(axis_x, axis_y, axis_z, qw, qx, qy, qz);

    grasp_point = Add(
        grasp_point,
        Scale(approach, config.side_grasp_forward_offset_m));
    candidate.grasp_pose = PoseAt(grasp_point, qw, qx, qy, qz);
    const cv::Point3f pre_grasp_point = Subtract(
        grasp_point, Scale(approach, config.side_approach_distance_m));
    candidate.pre_grasp_pose = PoseAt(pre_grasp_point, qw, qx, qy, qz);
    const cv::Point3f entry_clearance_point = Add(
        geometry.table_center,
        Scale(
            geometry.table.normal,
            geometry.height_m + config.finger_half_height_m +
                config.side_entry_clearance_m));
    candidate.entry_clearance_z_m = entry_clearance_point.z;
    const cv::Point3f retreat_point = Add(
        grasp_point,
        Scale(geometry.table.normal, config.side_initial_lift_m));
    candidate.retreat_pose = PoseAt(
        retreat_point, qw, qx, qy, qz);
    const float lift_retreat_distance_m = std::max(
        config.side_approach_distance_m, config.side_lift_retreat_m);
    cv::Point3f lift_point = Subtract(
        grasp_point, Scale(approach, lift_retreat_distance_m));
    lift_point = Add(
        lift_point,
        Scale(geometry.table.normal, config.side_initial_lift_m));
    candidate.lift_pose = PoseAt(lift_point, qw, qx, qy, qz);

    if (object_width_m + kMinimumGripperClearanceM >
        config.gripper_max_width_m) {
        std::ostringstream reason;
        reason << "required side opening exceeds gripper width"
                << " (object=" << object_width_m
                << "m, limit=" << config.gripper_max_width_m << "m)";
        Reject(candidate, reason.str());
    } else {
        candidate.required_width_m = std::min(
            candidate.required_width_m, config.gripper_max_width_m);
        candidate.geometry_valid = true;
    }
    return candidate;
}

float PoseWorkspaceMargin(
    const Pose3D& pose,
    const GraspPlannerConfig& planner_config) {
    const WorkspaceLimits& workspace = planner_config.workspace;
    return std::min({
        pose.x - workspace.x_min,
        workspace.x_max - pose.x,
        pose.y - workspace.y_min,
        workspace.y_max - pose.y,
        pose.z - workspace.z_min,
        workspace.z_max - pose.z});
}

void ApplyCandidateQualityScore(
    GraspCandidate& candidate,
    const ObjectGeometry3D& geometry,
    const GraspGeometryConfig& config,
    const GraspPlannerConfig& planner_config) {
    candidate.width_margin_m = std::max(
        0.0f, config.gripper_max_width_m - candidate.required_width_m);
    candidate.depth_quality = Clamp(
        static_cast<float>(geometry.object_point_count) /
            static_cast<float>(std::max(
                config.min_object_points * 4, 1)),
        0.0f, 1.0f);
    candidate.path_clearance_m =
        candidate.strategy == GraspStrategy::TOP
        ? std::max(
            0.0f, candidate.pre_grasp_pose.z - candidate.grasp_pose.z)
        : std::max(
            0.0f,
            candidate.entry_clearance_z_m - candidate.pre_grasp_pose.z);
    candidate.workspace_margin_m = std::min({
        PoseWorkspaceMargin(candidate.pre_grasp_pose, planner_config),
        PoseWorkspaceMargin(candidate.grasp_pose, planner_config),
        PoseWorkspaceMargin(candidate.retreat_pose, planner_config),
        PoseWorkspaceMargin(candidate.lift_pose, planner_config)});

    const float normalized_width_margin = Clamp(
        candidate.width_margin_m /
            std::max(config.gripper_max_width_m, kEpsilon),
        0.0f, 1.0f);
    const float normalized_path_clearance = Clamp(
        candidate.path_clearance_m / 0.05f, 0.0f, 1.0f);
    const float normalized_workspace_margin = Clamp(
        candidate.workspace_margin_m / 0.05f, 0.0f, 1.0f);
    candidate.score +=
        0.12f * normalized_width_margin +
        0.08f * candidate.depth_quality +
        0.05f * normalized_path_clearance +
        0.05f * normalized_workspace_margin;
}

}  // namespace

const char* GraspStrategyName(GraspStrategy strategy) {
    switch (strategy) {
        case GraspStrategy::TOP: return "top";
        case GraspStrategy::SIDE: return "side";
    }
    return "unknown";
}

bool AreObjectGeometriesConsistent(const ObjectGeometry3D& reference,
                                    const ObjectGeometry3D& current,
                                    std::string* reason) {
    const float center_distance = cv::norm(reference.center - current.center);
    const bool center_consistent = center_distance <= 0.010f;
    std::string dimension_reason;
    const bool dimensions_consistent = AreObjectDimensionsConsistent(
        reference, current, &dimension_reason);
    const bool consistent = center_consistent && dimensions_consistent;
    if (reason) {
        std::ostringstream message;
        message << "center_delta=" << center_distance << "m "
                << dimension_reason;
        *reason = message.str();
    }
    return consistent;
}

bool AreObjectDimensionsConsistent(const ObjectGeometry3D& reference,
                                    const ObjectGeometry3D& current,
                                    std::string* reason) {
    const bool length_consistent = WithinDimensionTolerance(
        reference.length_m, current.length_m, 0.025f, 0.35f);
    const bool width_consistent = WithinDimensionTolerance(
        reference.width_m, current.width_m, 0.020f, 0.40f);
    const bool height_consistent = WithinDimensionTolerance(
        reference.height_m, current.height_m, 0.020f, 0.30f);
    const bool consistent = reference.valid && current.valid &&
        length_consistent && width_consistent && height_consistent;
    if (reason) {
        std::ostringstream message;
        message << "reference_dims=[" << reference.length_m << ","
                << reference.width_m << "," << reference.height_m
                << "]m current_dims=[" << current.length_m << ","
                << current.width_m << "," << current.height_m << "]m";
        *reason = message.str();
    }
    return consistent;
}

GraspGeometryPlanner::GraspGeometryPlanner(
    const GraspGeometryConfig& config,
    const GraspPlannerConfig& planner_config)
    : config_(config), planner_config_(planner_config) {}

bool GraspGeometryPlanner::Plan(
    const cv::Mat& depth,
    const DetectionTarget& target,
    const StereoCamera& camera,
    const GraspPlanner& coordinate_planner,
    GraspGeometryResult& result,
    std::optional<GraspStrategy> locked_strategy) const {
    const auto started_at = std::chrono::steady_clock::now();
    result = GraspGeometryResult{};
    if (depth.empty() || depth.type() != CV_16UC1) {
        result.error = "aligned metric depth must be CV_16UC1";
        return false;
    }

    cv::Mat target_mask = NormalizeTargetMask(target, depth.size());
    if (target_mask.empty()) {
        result.error = "target segmentation mask is unavailable";
        return false;
    }
    cv::Mat eroded_mask;
    cv::erode(target_mask, eroded_mask,
                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    if (cv::countNonZero(eroded_mask) < 16) eroded_mask = target_mask;

    const float silhouette_width = std::max(1.0f, target.x2 - target.x1);
    const float silhouette_height = std::max(1.0f, target.y2 - target.y1);
    const std::vector<cv::Point> target_pixels = SamplePixels(
        target_mask, AdaptiveStride(target_mask, config_));
    std::vector<float> target_depths;
    target_depths.reserve(target_pixels.size());
    for (const cv::Point& pixel : target_pixels) {
        const uint16_t depth_mm = depth.at<uint16_t>(pixel.y, pixel.x);
        if (depth_mm != 0) target_depths.push_back(depth_mm);
    }
    result.center_depth_mm = MedianDepthAround(depth, target.center, 4);
    // The inside edge of the segmentation mask is a more stable foreground
    // reference than a low global percentile. A small nearby occluder (for
    // example the robot wrist entering one side of the mask) can dominate the
    // 5th/25th percentiles even though the object center and most of its
    // silhouette remain at the correct depth. The boundary median also keeps
    // the intended behavior for hollow objects, where the center sees through
    // the opening but the rim carries the object depth.
    const float boundary_depth_mm =
        MedianMaskBoundaryDepth(depth, target_mask);
    const bool silhouette_depth_pattern =
        target_depths.size() <
            static_cast<size_t>(config_.min_object_points) ||
        (std::isfinite(result.center_depth_mm) &&
        std::isfinite(boundary_depth_mm) &&
        result.center_depth_mm - boundary_depth_mm >=
            kForegroundDepthMarginMm);
    const bool upright_silhouette =
        silhouette_height >= 1.05f * silhouette_width &&
        silhouette_depth_pattern;

    // Open or transparent upright targets are identified from image shape and
    // depth discontinuity. No object class is used for this decision.
    const cv::Mat& object_mask =
        upright_silhouette ? target_mask : eroded_mask;
    const int stride = upright_silhouette
        ? 1
        : AdaptiveStride(object_mask, config_);
    const std::vector<cv::Point> object_pixels =
        SamplePixels(object_mask, stride);
    std::vector<float> object_depths;
    object_depths.reserve(object_pixels.size());
    for (const cv::Point& pixel : object_pixels) {
        const uint16_t depth_mm = depth.at<uint16_t>(pixel.y, pixel.x);
        if (depth_mm != 0) object_depths.push_back(depth_mm);
    }
    const int x1 = std::clamp(
        static_cast<int>(std::floor(target.x1)), 0, depth.cols);
    const int y1 = std::clamp(
        static_cast<int>(std::floor(target.y1)), 0, depth.rows);
    const int x2 = std::clamp(
        static_cast<int>(std::ceil(target.x2)), 0, depth.cols);
    const int y2 = std::clamp(
        static_cast<int>(std::ceil(target.y2)), 0, depth.rows);
    const int margin = std::max(12, std::max(x2 - x1, y2 - y1) / 3);
    const cv::Rect support_roi(
        std::max(0, x1 - margin), std::max(0, y1 - margin),
        std::min(depth.cols, x2 + margin) - std::max(0, x1 - margin),
        std::min(depth.rows, y2 + margin) - std::max(0, y1 - margin));
    cv::Mat support_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
    if (support_roi.width > 0 && support_roi.height > 0) {
        support_mask(support_roi).setTo(255);
    }
    cv::Mat dilated_target;
    cv::dilate(target_mask, dilated_target,
                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));
    support_mask.setTo(0, dilated_target);
    const std::vector<cv::Point> support_pixels =
        SamplePixels(support_mask, std::max(2, stride * 2));

    std::string silhouette_error;
    if (upright_silhouette) {
        std::vector<cv::Point3f> support_points;
        DeprojectPixels(
            support_pixels, depth, camera, coordinate_planner,
            1.0f, 20000.0f, support_points);

        ObjectGeometry3D silhouette_geometry;
        std::vector<cv::Point3f> silhouette_points;
        if (EstimateUprightContainerFromSilhouette(
                target_mask, support_points, camera, coordinate_planner,
                config_, silhouette_geometry, silhouette_points,
                silhouette_error)) {
            ObjectGeometry3D depth_geometry;
            std::vector<cv::Point3f> depth_points;
            std::string depth_geometry_error;
            std::vector<cv::Point3f> raw_depth_points;
            bool depth_geometry_valid = false;
            if (!target_depths.empty()) {
                const float depth_low = std::max(
                    1.0f, Percentile(target_depths, 0.01f) -
                        kForegroundDepthMarginMm);
                const float depth_high =
                    Percentile(target_depths, 0.99f) +
                        kForegroundDepthMarginMm;
                DeprojectPixels(
                    target_pixels, depth, camera, coordinate_planner,
                    depth_low, depth_high, raw_depth_points);
                depth_geometry_valid = EstimateObjectGeometry(
                    raw_depth_points, support_points, config_,
                    depth_geometry, &depth_points,
                    depth_geometry_error);
            }

            if (depth_geometry_valid &&
                IsClearlyHorizontal(depth_geometry, config_)) {
                result.geometry = depth_geometry;
                result.object_points = std::move(depth_points);
            } else {
                result.geometry = silhouette_geometry;
                result.object_points = std::move(silhouette_points);
            }
            result.candidates = GenerateCandidates(
                result.geometry, result.object_points, config_,
                planner_config_, locked_strategy);
            result.elapsed_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - started_at)
                    .count();
            return !result.candidates.empty();
        }
        result.geometry = ObjectGeometry3D{};
        result.object_points.clear();
    }

    if (object_depths.size() <
        static_cast<size_t>(config_.min_object_points)) {
        result.error = "too few valid depth samples in target mask";
        if (!silhouette_error.empty()) {
            result.error +=
                "; silhouette fallback failed: " + silhouette_error;
        }
        result.elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started_at)
                .count();
        return false;
    }

    std::vector<float> foreground_candidates;
    if (std::isfinite(boundary_depth_mm)) {
        foreground_candidates.push_back(boundary_depth_mm);
    }
    if (std::isfinite(result.center_depth_mm)) {
        const bool duplicate = std::any_of(
            foreground_candidates.begin(), foreground_candidates.end(),
            [center_depth = result.center_depth_mm](float existing) {
                return std::fabs(existing - center_depth) <
                    kForegroundDepthMarginMm;
            });
        if (!duplicate) {
            foreground_candidates.push_back(result.center_depth_mm);
        }
    }
    for (const float quantile : {
            0.005f, 0.01f, 0.02f, 0.05f, 0.10f, 0.25f, 0.50f,
            0.75f, 0.90f, 0.95f, 0.98f}) {
        const float candidate = Percentile(object_depths, quantile);
        const bool duplicate = std::any_of(
            foreground_candidates.begin(), foreground_candidates.end(),
            [candidate](float existing) {
                return std::fabs(existing - candidate) <
                    kForegroundDepthMarginMm;
            });
        if (!duplicate) foreground_candidates.push_back(candidate);
    }

    const float foreground_reference_depth_mm =
        std::isfinite(boundary_depth_mm)
            ? boundary_depth_mm
            : Percentile(
                object_depths, upright_silhouette ? 0.01f : 0.25f);
    float best_reference_error_mm = std::numeric_limits<float>::max();
    float best_geometry_volume = -1.0f;
    int best_object_point_count = -1;
    std::string last_geometry_error;
    std::vector<std::string> cluster_diagnostics;
    for (const float foreground_depth : foreground_candidates) {
        const float depth_low = std::max(
            1.0f, foreground_depth - kForegroundDepthMarginMm);
        const float maximum_depth_span = upright_silhouette
            ? kSilhouetteDepthSpanMm
            : kMaximumObjectDepthSpanMm;
        const float depth_high = foreground_depth + maximum_depth_span;
        std::vector<cv::Point3f> raw_object_points;
        DeprojectPixels(object_pixels, depth, camera, coordinate_planner,
                        depth_low, depth_high, raw_object_points);
        if (raw_object_points.size() <
            static_cast<size_t>(config_.min_object_points)) {
            std::ostringstream diagnostic;
            diagnostic << std::lround(foreground_depth) << "mm: "
                        << raw_object_points.size() << " object points";
            cluster_diagnostics.push_back(diagnostic.str());
            continue;
        }

        std::vector<cv::Point3f> support_points;
        const float support_depth_low = std::max(
            1.0f, foreground_depth - kSupportDepthBeforeObjectMm);
        const float support_depth_high =
            foreground_depth + kSupportDepthBehindObjectMm;
        DeprojectPixels(support_pixels, depth, camera, coordinate_planner,
                        support_depth_low, support_depth_high, support_points);

        ObjectGeometry3D candidate_geometry;
        std::vector<cv::Point3f> candidate_points;
        std::string candidate_error;
        if (!EstimateObjectGeometry(
                raw_object_points, support_points, config_,
                candidate_geometry, &candidate_points, candidate_error)) {
            if (candidate_geometry.table.inlier_count >
                result.geometry.table.inlier_count) {
                result.geometry.table = candidate_geometry.table;
            }
            last_geometry_error = candidate_error;
            std::ostringstream diagnostic;
            diagnostic << std::lround(foreground_depth) << "mm: "
                        << candidate_error << " (object="
                        << raw_object_points.size() << ", support="
                        << support_points.size() << ")";
            cluster_diagnostics.push_back(diagnostic.str());
            continue;
        }
        const float reference_error_mm = std::fabs(
            foreground_depth - foreground_reference_depth_mm);
        const float geometry_volume = candidate_geometry.length_m *
            candidate_geometry.width_m * candidate_geometry.height_m;
        const bool better_candidate = best_object_point_count < 0 ||
            reference_error_mm < best_reference_error_mm - 1.0f ||
            (std::fabs(reference_error_mm - best_reference_error_mm) <=
                1.0f &&
                (geometry_volume > best_geometry_volume + 1e-8f ||
                (std::fabs(geometry_volume - best_geometry_volume) <=
                    1e-8f && candidate_geometry.object_point_count >
                    best_object_point_count)));
        if (!better_candidate) continue;

        best_reference_error_mm = reference_error_mm;
        best_geometry_volume = geometry_volume;
        best_object_point_count = candidate_geometry.object_point_count;
        result.geometry = candidate_geometry;
        result.object_points = std::move(candidate_points);
        result.foreground_depth_mm = foreground_depth;
    }

    if (best_object_point_count < 0) {
        result.error = "no valid foreground depth cluster";
        if (!last_geometry_error.empty()) {
            result.error += ": " + last_geometry_error;
        }
        if (!cluster_diagnostics.empty()) {
            result.error += "; candidates={";
            for (size_t index = 0; index < cluster_diagnostics.size();
                ++index) {
                if (index != 0) result.error += "; ";
                result.error += cluster_diagnostics[index];
            }
            result.error += "}";
        }
        result.elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started_at)
                .count();
        return false;
    }
    result.candidates = GenerateCandidates(
        result.geometry, result.object_points, config_, planner_config_,
        locked_strategy);
    result.elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started_at)
            .count();
    return !result.candidates.empty();
}

bool GraspGeometryPlanner::EstimateObjectGeometry(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point3f>& support_points,
    const GraspGeometryConfig& config,
    ObjectGeometry3D& geometry,
    std::vector<cv::Point3f>* filtered_object_points,
    std::string& error) {
    geometry = ObjectGeometry3D{};
    geometry.source_point_count = static_cast<int>(object_points.size());
    if (object_points.size() < static_cast<size_t>(config.min_object_points)) {
        error = "too few object points";
        return false;
    }
    if (!FitTablePlane(support_points, config, geometry.table) &&
        !FitHorizontalSupportPlane(support_points, config, geometry.table)) {
        error = "support table plane was not found";
        return false;
    }

    std::vector<cv::Point3f> filtered;
    std::vector<float> heights;
    const auto filter_object_points = [&]() {
        filtered.clear();
        heights.clear();
        filtered.reserve(object_points.size());
        heights.reserve(object_points.size());
        for (const cv::Point3f& point : object_points) {
            const float height =
                Dot(geometry.table.normal, point) + geometry.table.d;
            if (height >= config.table_clearance_m &&
                height <= config.object_max_height_m) {
                filtered.push_back(point);
                heights.push_back(height);
            }
        }
    };
    filter_object_points();
    if (filtered.size() < static_cast<size_t>(config.min_object_points)) {
        TablePlane horizontal_table;
        if (FitHorizontalSupportPlane(
                support_points, config, horizontal_table)) {
            geometry.table = horizontal_table;
            filter_object_points();
        }
    }
    if (filtered.size() < static_cast<size_t>(config.min_object_points)) {
        std::vector<float> raw_heights;
        raw_heights.reserve(object_points.size());
        for (const cv::Point3f& point : object_points) {
            raw_heights.push_back(
                Dot(geometry.table.normal, point) + geometry.table.d);
        }
        std::ostringstream diagnostic;
        diagnostic
            << "target mask does not contain enough points above the table"
            << " (filtered=" << filtered.size() << "/"
            << object_points.size()
            << ", height_p05=" << Percentile(raw_heights, 0.05f)
            << "m, height_p50=" << Percentile(raw_heights, 0.50f)
            << "m, height_p95=" << Percentile(raw_heights, 0.95f)
            << "m, table_normal=[" << geometry.table.normal.x << ","
            << geometry.table.normal.y << "," << geometry.table.normal.z
            << "], table_d=" << geometry.table.d << ")";
        error = diagnostic.str();
        return false;
    }

    const float object_height = Percentile(heights, 0.95f);
    if (!std::isfinite(object_height) ||
        object_height < config.object_min_height_m) {
        error = "estimated object height is below the safe minimum";
        return false;
    }

    cv::Point3f axis_u;
    cv::Point3f axis_v;
    PlaneBasis(geometry.table.normal, axis_u, axis_v);
    std::vector<cv::Point2f> projected;
    std::vector<float> projected_u;
    std::vector<float> projected_v;
    projected.reserve(filtered.size());
    projected_u.reserve(filtered.size());
    projected_v.reserve(filtered.size());
    for (const cv::Point3f& point : filtered) {
        const float u = Dot(point, axis_u);
        const float v = Dot(point, axis_v);
        projected.emplace_back(u, v);
        projected_u.push_back(u);
        projected_v.push_back(v);
    }

    const float u_min = Percentile(projected_u, 0.01f);
    const float u_max = Percentile(projected_u, 0.99f);
    const float v_min = Percentile(projected_v, 0.01f);
    const float v_max = Percentile(projected_v, 0.99f);
    std::vector<cv::Point2f> robust_projected;
    robust_projected.reserve(projected.size());
    for (const cv::Point2f& point : projected) {
        if (point.x >= u_min && point.x <= u_max &&
            point.y >= v_min && point.y <= v_max) {
            robust_projected.push_back(point);
        }
    }
    if (robust_projected.size() < 8) {
        error = "object footprint is degenerate";
        return false;
    }

    const cv::RotatedRect rectangle = cv::minAreaRect(robust_projected);
    const float angle = rectangle.angle * static_cast<float>(CV_PI) / 180.0f;
    const cv::Point2f width_axis(std::cos(angle), std::sin(angle));
    const cv::Point2f height_axis(-std::sin(angle), std::cos(angle));
    cv::Point2f major_axis_2d;
    cv::Point2f minor_axis_2d;
    if (rectangle.size.width >= rectangle.size.height) {
        geometry.length_m = rectangle.size.width;
        geometry.width_m = rectangle.size.height;
        major_axis_2d = width_axis;
        minor_axis_2d = height_axis;
    } else {
        geometry.length_m = rectangle.size.height;
        geometry.width_m = rectangle.size.width;
        major_axis_2d = height_axis;
        minor_axis_2d = width_axis;
    }
    geometry.height_m = object_height;
    geometry.major_axis = Normalize(Add(
        Scale(axis_u, major_axis_2d.x), Scale(axis_v, major_axis_2d.y)));
    geometry.minor_axis = Normalize(Add(
        Scale(axis_u, minor_axis_2d.x), Scale(axis_v, minor_axis_2d.y)));
    if (geometry.major_axis.x < 0.0f) {
        geometry.major_axis = Scale(geometry.major_axis, -1.0f);
    }
    if (geometry.minor_axis.y < 0.0f) {
        geometry.minor_axis = Scale(geometry.minor_axis, -1.0f);
    }

    geometry.table_center = Add(
        Add(Scale(axis_u, rectangle.center.x),
            Scale(axis_v, rectangle.center.y)),
        Scale(geometry.table.normal, -geometry.table.d));
    geometry.center = Add(
        geometry.table_center,
        Scale(geometry.table.normal, geometry.height_m * 0.5f));
    geometry.object_point_count = static_cast<int>(filtered.size());
    geometry.valid = true;
    if (filtered_object_points) *filtered_object_points = std::move(filtered);
    error.clear();
    return true;
}

std::vector<GraspCandidate> GraspGeometryPlanner::GenerateCandidates(
    const ObjectGeometry3D& geometry,
    const std::vector<cv::Point3f>& object_points,
    const GraspGeometryConfig& config,
    const GraspPlannerConfig& planner_config,
    std::optional<GraspStrategy> locked_strategy) {
    std::vector<GraspCandidate> candidates;
    if (!geometry.valid) return candidates;
    const bool allow_top = config.strategy == "auto" ||
        config.strategy == "top";
    const bool allow_side = config.strategy == "auto" ||
        config.strategy == "side";
    const float height_footprint_ratio =
        HeightToFootprintRatio(geometry);
    const bool preferred_side_shape =
        geometry.height_m >= config.side_min_height_m &&
        height_footprint_ratio >=
            config.side_min_height_width_ratio;
    const bool locked_side_shape =
        locked_strategy == GraspStrategy::SIDE &&
        geometry.height_m >=
            config.side_min_height_m * kLockedSideHysteresisRatio &&
        height_footprint_ratio >=
            config.side_min_height_width_ratio *
                kLockedSideHysteresisRatio;
    const bool side_shape = preferred_side_shape || locked_side_shape;
    const bool geometry_requires_side = config.strategy == "auto" &&
        preferred_side_shape;

    if (allow_top) {
        const TopGraspSection section =
            FindTopGraspSection(geometry, object_points, config);
        GraspCandidate top;
        top.strategy = GraspStrategy::TOP;
        top.approach_axis = cv::Point3f(0.0f, 0.0f, -1.0f);
        top.opening_axis = section.opening_axis;
        top.opening_axis.z = 0.0f;
        top.opening_axis = Normalize(top.opening_axis);
        const float object_width_m = section.width_m;
        top.required_width_m = object_width_m +
            2.0f * config.footprint_padding_m;
        const float low_object_bonus = 0.20f * Clamp(
            (config.side_min_height_m - geometry.height_m) /
                std::max(config.side_min_height_m, kEpsilon),
            0.0f, 1.0f);
        top.score = 0.75f + low_object_bonus +
            (section.localized ? 0.02f : 0.0f);

        cv::Point3f grasp_point = Add(
            section.table_center,
            Scale(geometry.table.normal, section.height_m));
        grasp_point.z = std::max(
            section.table_center.z + config.table_clearance_m,
            grasp_point.z - planner_config.grasp_depth);
        grasp_point.z = std::max(
            grasp_point.z, planner_config.workspace.z_min);
        grasp_point = Add(
            grasp_point,
            Scale(top.opening_axis, planner_config.gripper_offset));
        top.grasp_pose = PoseAt(grasp_point, 0.0f, 0.0f, 1.0f, 0.0f);
        top.pre_grasp_pose = top.grasp_pose;
        top.pre_grasp_pose.z += planner_config.approach_height;
        top.retreat_pose = top.pre_grasp_pose;
        top.lift_pose = top.pre_grasp_pose;
        top.grasp_yaw_rad = std::atan2(
            -top.opening_axis.x, top.opening_axis.y);
        while (top.grasp_yaw_rad < 0.0f) {
            top.grasp_yaw_rad += static_cast<float>(CV_PI);
        }
        while (top.grasp_yaw_rad >= static_cast<float>(CV_PI)) {
            top.grasp_yaw_rad -= static_cast<float>(CV_PI);
        }

        if (geometry_requires_side) {
            Reject(top, "3D geometry requires side grasp");
        } else if (object_width_m + kMinimumGripperClearanceM >
            config.gripper_max_width_m) {
            Reject(top, "required top opening exceeds gripper width");
        } else {
            top.required_width_m = std::min(
                top.required_width_m, config.gripper_max_width_m);
            top.geometry_valid = true;
        }
        candidates.push_back(top);
    }

    if (allow_side) {
        if (!side_shape) {
            GraspCandidate side;
            side.strategy = GraspStrategy::SIDE;
            side.score = 0.0f;
            Reject(side, "object is not tall enough for side grasp");
            candidates.push_back(side);
        } else {
            cv::Point3f radial = Normalize(
                cv::Point3f(geometry.center.x, geometry.center.y, 0.0f));
            if (Norm(radial) < kEpsilon) {
                radial = cv::Point3f(1.0f, 0.0f, 0.0f);
            }
            cv::Point3f major = geometry.major_axis;
            major.z = 0.0f;
            major = Normalize(major);
            if (Dot(major, radial) < 0.0f) major = Scale(major, -1.0f);

            const float tall_bonus = 0.12f * Clamp(
                (height_footprint_ratio -
                    config.side_min_height_width_ratio) / 2.0f,
                0.0f, 1.0f);
            const float side_base_score =
                preferred_side_shape ? 0.88f : 0.65f;
            const cv::Point3f radial_opening(
                -radial.y, radial.x, 0.0f);
            candidates.push_back(MakeSideCandidate(
                geometry, object_points, radial, radial_opening,
                config, side_base_score + tall_bonus));

            // Keep one geometrically distinct approach so IK and path
            // validation can choose the safer reachable side.
            if (std::fabs(Dot(major, radial)) < 0.98f) {
                candidates.push_back(MakeSideCandidate(
                    geometry, object_points, major, geometry.minor_axis,
                    config, side_base_score - 0.04f + tall_bonus));
            }
        }
    }

    for (GraspCandidate& candidate : candidates) {
        if (candidate.geometry_valid) {
            ApplyCandidateQualityScore(
                candidate, geometry, config, planner_config);
        }
    }
    std::stable_sort(
        candidates.begin(), candidates.end(),
        [](const GraspCandidate& lhs, const GraspCandidate& rhs) {
            if (lhs.geometry_valid != rhs.geometry_valid) {
                return lhs.geometry_valid > rhs.geometry_valid;
            }
            return lhs.score > rhs.score;
        });
    return candidates;
}

}  // namespace perceptive_grasp
