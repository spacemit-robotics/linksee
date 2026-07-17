#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression checks for configurable stereo camera backends."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CMAKE = ROOT / "CMakeLists.txt"
CONFIG = ROOT / "config" / "grasp_pipeline.yaml"
DETECTOR_CONFIG = ROOT / "config" / "yolov8_seg.yaml"
MAIN_CPP = ROOT / "src" / "main.cpp"
PIPELINE_CPP = ROOT / "src" / "grasp_pipeline.cpp"
TARGET_CPP = ROOT / "src" / "target_detector.cpp"
TARGET_H = ROOT / "include" / "target_detector.h"
PIPELINE_H = ROOT / "include" / "grasp_pipeline.h"
STEREO_H = ROOT / "include" / "stereo_camera.h"
CAMERA_FACTORY_CPP = ROOT / "src" / "stereo_camera.cpp"
LAS2_CPP = ROOT / "src" / "las2_stereo_camera.cpp"
STEREO_GEOMETRY_CPP = ROOT / "src" / "stereo_geometry.cpp"
HAND_EYE_PY = ROOT / "scripts" / "calibrate_hand_eye.py"
DEBUG_VIEW_CPP = ROOT / "tools" / "debug_view.cpp"
DEBUG_LOCALIZE_CPP = ROOT / "tools" / "debug_localize.cpp"


class CameraBackendSourceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cmake = CMAKE.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")
        cls.detector_config = DETECTOR_CONFIG.read_text(encoding="utf-8")
        cls.main = MAIN_CPP.read_text(encoding="utf-8")
        cls.pipeline = PIPELINE_CPP.read_text(encoding="utf-8")
        cls.target = TARGET_CPP.read_text(encoding="utf-8")
        cls.target_h = TARGET_H.read_text(encoding="utf-8")
        cls.pipeline_h = PIPELINE_H.read_text(encoding="utf-8")
        cls.stereo_h = STEREO_H.read_text(encoding="utf-8")
        cls.camera_factory = CAMERA_FACTORY_CPP.read_text(encoding="utf-8")
        cls.las2_cpp = LAS2_CPP.read_text(encoding="utf-8")
        cls.stereo_geometry = STEREO_GEOMETRY_CPP.read_text(encoding="utf-8")
        cls.hand_eye = HAND_EYE_PY.read_text(encoding="utf-8")
        cls.debug_view = DEBUG_VIEW_CPP.read_text(encoding="utf-8")
        cls.debug_localize = DEBUG_LOCALIZE_CPP.read_text(encoding="utf-8")

    def test_config_exposes_stereo_camera_backends(self):
        self.assertIn("type:", self.config)
        self.assertIn("realsense:", self.config)
        self.assertIn("spacemit_las2:", self.config)
        self.assertIn("calib_path:", self.config)

    def test_loader_reads_only_selected_backend_settings(self):
        self.assertIn('cam["type"]', self.main)
        self.assertIn('cam["realsense"]', self.main)
        self.assertIn('realsense["motion_flush_frames"]', self.main)
        self.assertIn("motion_flush_frames: 16", self.config)
        self.assertIn("int motion_flush_frames = 16", self.stereo_h)
        self.assertIn('cam["spacemit_las2"]', self.main)
        self.assertIn('las2["model_path"]', self.main)
        self.assertIn('las2["core_count"]', self.main)
        self.assertIn('las2["core_affinity"]', self.main)
        self.assertIn("ResolveConfigPath(config_dir", self.main)
        self.assertIn("&cfg.camera.spacemit_las2.model_path", self.main)
        self.assertIn("&cfg.camera.spacemit_las2.calib_path", self.main)
        self.assertNotIn("camera.realsense configuration is required", self.main)
        self.assertIn(
            "camera.spacemit_las2 configuration is required", self.main
        )

    def test_release_config_exposes_runtime_inference_settings(self):
        self.assertIn("core_count: 1", self.config)
        self.assertIn('core_affinity: "8"', self.config)
        self.assertIn("min_m: 0.05", self.config)
        self.assertIn("max_m: 2.0", self.config)
        self.assertIn("int core_count = 1", self.stereo_h)
        self.assertIn('std::string core_affinity = "8"', self.stereo_h)
        self.assertIn("config_path: yolov8_seg.yaml", self.config)
        self.assertIn("target_labels: []", self.config)
        self.assertIn(
            'std::string config_path = "yolov8_seg.yaml"',
            self.target_h,
        )

    def test_backend_config_types_are_separate(self):
        self.assertIn("struct RealSenseCameraConfig", self.stereo_h)
        self.assertIn("struct SpacemitLas2CameraConfig", self.stereo_h)
        self.assertIn("RealSenseCameraConfig realsense", self.stereo_h)
        self.assertIn(
            "SpacemitLas2CameraConfig spacemit_las2", self.stereo_h
        )

    def test_pipeline_uses_camera_factory_interface(self):
        self.assertIn("StereoCameraConfig camera", self.pipeline_h)
        self.assertIn("std::unique_ptr<StereoCamera> camera_", self.pipeline_h)
        self.assertIn("CreateStereoCamera(config_.camera)", self.pipeline)
        self.assertNotIn("std::make_unique<DepthCamera>", self.pipeline)
        self.assertIn("FlushCameraAfterMotion", self.pipeline)

    def test_factory_preserves_realsense_and_exposes_las2(self):
        self.assertIn("class StereoCamera", self.stereo_h)
        self.assertIn("std::make_unique<DepthCamera>", self.camera_factory)
        self.assertIn("HAVE_REALSENSE_CAMERA", self.camera_factory)
        self.assertIn("UnsupportedRealSenseCamera", self.camera_factory)
        self.assertIn(
            'config.type == "spacemit_las2"', self.camera_factory
        )
        self.assertIn("UnsupportedSpacemitLas2Camera", self.camera_factory)

    def test_build_includes_camera_factory_and_optional_las2(self):
        self.assertIn("src/stereo_camera.cpp", self.cmake)
        self.assertIn("find_package(realsense2 QUIET", self.cmake)
        self.assertIn("if(HAVE_REALSENSE_CAMERA)", self.cmake)
        self.assertIn("ENABLE_LAS2_CAMERA", self.cmake)
        self.assertIn("LAS2_RUNTIME_DIR", self.cmake)
        self.assertIn('$ENV{HOME}/las2_runtime', self.cmake)
        self.assertIn("src/las2_stereo_camera.cpp", self.cmake)
        self.assertIn("src/stereo_geometry.cpp", self.cmake)
        self.assertIn("PATCHELF_EXECUTABLE", self.cmake)
        self.assertIn("_LAS2_PATCHED_LIBRARY", self.cmake)
        self.assertIn("--set-rpath", self.cmake)

    def test_public_binary_wraps_core_with_optional_debug_output(self):
        self.assertIn("add_executable(perceptive_grasp_core", self.cmake)
        self.assertIn("add_custom_target(perceptive_grasp ALL", self.cmake)
        self.assertIn("scripts/run_perceptive_grasp.py", self.cmake)

    def test_debug_view_uses_selected_stereo_camera_backend(self):
        self.assertIn("LoadDebugViewConfig", self.debug_view)
        self.assertIn("CreateStereoCamera(config.camera)", self.debug_view)
        self.assertIn("camera->GetFrames(color, depth)", self.debug_view)
        self.assertIn(
            'config.camera.type == "spacemit_las2"', self.debug_view
        )
        self.assertNotIn("librealsense2/rs.hpp", self.debug_view)
        self.assertNotIn("rs2::pipeline", self.debug_view)

    def test_debug_localize_uses_selected_stereo_camera_backend(self):
        self.assertIn("LoadCameraConfig", self.debug_localize)
        self.assertIn(
            "CreateStereoCamera(camera_config)", self.debug_localize
        )
        self.assertIn(
            "camera->GetFrames(color, depth)", self.debug_localize
        )
        self.assertIn("camera->Deproject", self.debug_localize)
        self.assertNotIn("librealsense2/rs.hpp", self.debug_localize)
        self.assertNotIn("rs2::pipeline", self.debug_localize)

    def test_debug_view_builds_with_any_camera_backend(self):
        self.assertIn(
            "if(HAVE_REALSENSE_CAMERA OR HAVE_LAS2_CAMERA)",
            self.cmake,
        )
        self.assertIn("src/stereo_camera.cpp", self.cmake)
        self.assertIn("src/las2_stereo_camera.cpp", self.cmake)
        self.assertIn("target_link_libraries(debug_view PRIVATE", self.cmake)

    def test_las2_backend_uses_aligned_rgb_depth_api(self):
        self.assertIn("las2::Camera", self.las2_cpp)
        self.assertIn("frame.rgb", self.las2_cpp)
        self.assertIn("frame.depth", self.las2_cpp)
        self.assertIn("frame.frame_id", self.las2_cpp)
        self.assertIn("LastFrameId", self.stereo_h)
        self.assertIn("cv::COLOR_RGB2BGR", self.las2_cpp)
        self.assertIn("settings.core_affinity.c_str()", self.las2_cpp)
        self.assertIn("stage=CAMERA_REFRESH", self.pipeline)
        self.assertIn("current_frame_id != previous_frame_id", self.pipeline)

    def test_las2_intrinsics_are_derived_from_calibration(self):
        self.assertIn("LoadRectifiedLeftIntrinsics", self.las2_cpp)
        self.assertIn("ScaleIntrinsicsWithLetterbox", self.las2_cpp)
        self.assertIn("cv::stereoRectify", self.stereo_geometry)

    def test_detection_stability_and_performance_are_observable(self):
        self.assertIn("stable_frames: 1", self.config)
        self.assertIn("int detect_stable_frames = 1", self.pipeline_h)
        self.assertIn('det["stable_frames"]', self.main)
        self.assertIn('module=" << module', self.pipeline)
        self.assertIn('"camera_warmup"', self.pipeline)
        self.assertIn('"detector_warmup"', self.pipeline)
        self.assertIn("Stereo camera and detector warmup complete", self.pipeline)
        self.assertIn('"[Timing] stage=DETECTING"', self.pipeline)
        self.assertIn("camera_ms=", self.pipeline)
        self.assertIn("camera_cpu_ms=", self.pipeline)
        self.assertIn("detector_ms=", self.pipeline)
        self.assertIn('"[Timing] stage=PLANNING"', self.pipeline)
        self.assertIn('"[Action] END stage="', self.pipeline)

    def test_pipeline_prints_stage_and_task_summaries(self):
        self.assertIn('"[Stage "', self.pipeline)
        self.assertIn("PIPELINE SUMMARY", self.pipeline)
        self.assertIn("initialization_ms=", self.pipeline)
        self.assertIn("task_ms=", self.pipeline)
        self.assertIn("end_to_end_ms=", self.pipeline)
        self.assertIn("stage_timings_", self.pipeline_h)

    def test_detector_uses_spacemit_execution_provider(self):
        self.assertIn("SpaceMITExecutionProvider", self.detector_config)
        self.assertNotIn("CPUExecutionProvider", self.detector_config)
        self.assertIn(
            'VisionService::Create(config_.config_path, "", false)',
            self.target,
        )

    def test_hand_eye_calibration_selects_configured_camera(self):
        self.assertIn("--camera-type", self.hand_eye)
        self.assertIn("_read_pipeline_camera_config", self.hand_eye)
        self.assertIn("_read_manipulator_device", self.hand_eye)
        self.assertIn("_resolve_camera_type", self.hand_eye)
        self.assertIn("Las2CalibrationCamera", self.hand_eye)

    def test_las2_hand_eye_uses_rectified_logical_left_rgb(self):
        self.assertIn('cv2.VideoWriter_fourcc(*"MJPG")', self.hand_eye)
        self.assertIn("cv2.stereoRectify", self.hand_eye)
        self.assertIn("cv2.initUndistortRectifyMap", self.hand_eye)
        self.assertIn("left_start = prefix + self.eye_width", self.hand_eye)


if __name__ == "__main__":
    unittest.main()
