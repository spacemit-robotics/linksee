#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression checks for isolated hardware, simulation and ASR profiles."""

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"


def _load(name):
    return yaml.safe_load((CONFIG / name).read_text(encoding="utf-8"))


class BackendProfilesTest(unittest.TestCase):
    def test_default_profile_keeps_validated_hardware_backends(self):
        config = _load("grasp_pipeline.yaml")
        self.assertEqual(config["camera"]["type"], "realsense")
        self.assertEqual(
            config["manipulator"].get("driver", "so101"), "so101"
        )
        self.assertEqual(
            config["manipulator"]["tip_link"], "gripper_frame_link"
        )
        self.assertTrue(config["mobile_base"]["enabled"])
        self.assertEqual(config["voice"]["asr"]["backend"], "sensevoice")
        self.assertEqual(config["grasp"]["top"]["verification_lift_m"], 0.0)
        self.assertEqual(config["grasp"]["top"]["minimum_grasp_height"], 0.0)

    def test_default_profile_matches_k3_validated_grasp_behavior(self):
        config = _load("grasp_pipeline.yaml")
        top = config["grasp"]["top"]
        self.assertEqual(top["approach_height"], 0.10)
        self.assertEqual(top["gripper_offset"], 0.0)
        self.assertEqual(top["grasp_point_x_ratio"], 1.0)
        self.assertEqual(top["gripper_open"], 0.6)
        self.assertFalse(top["safe_mask_interior"])
        self.assertFalse(top["support_plane_occlusion_recovery"])
        self.assertFalse(top["support_plane_height_anchor"])
        self.assertTrue(config["orientation"]["enabled"])
        self.assertTrue(config["manipulator"]["legacy_top_ik"])
        self.assertEqual(config["mobile_base"]["target_x"], 0.275)
        self.assertEqual(config["mobile_base"]["x_tolerance"], 0.035)
        self.assertEqual(config["mobile_base"]["min_cmd_duration_ms"], 350)
        self.assertEqual(config["mobile_base"]["max_align_attempts"], 6)
        self.assertEqual(config["mobile_base"]["max_total_travel_m"], 0.24)
        self.assertEqual(
            config["manipulator"]["joint_limits"][3],
            {"joint": 3, "min": -1.790, "max": 1.670},
        )

    def test_simulation_profiles_explicitly_opt_into_new_planning(self):
        for name in (
            "grasp_pipeline_mujoco_ur5e.yaml",
            "grasp_pipeline_remote_mujoco_ur5e.yaml",
        ):
            config = _load(name)
            top = config["grasp"]["top"]
            self.assertTrue(top["safe_mask_interior"], name)
            self.assertTrue(top["support_plane_occlusion_recovery"], name)
            self.assertTrue(top["support_plane_height_anchor"], name)
            self.assertFalse(config["manipulator"]["legacy_top_ik"], name)

    def test_local_mujoco_profile_is_explicit_and_hardware_independent(self):
        config = _load("grasp_pipeline_mujoco_ur5e.yaml")
        self.assertEqual(config["camera"]["type"], "mujoco")
        self.assertEqual(config["manipulator"]["driver"], "mujoco_ur5e")
        self.assertFalse(config["mobile_base"]["enabled"])
        self.assertNotIn("uart_device", config["manipulator"])

    def test_remote_mujoco_profile_uses_one_shared_endpoint(self):
        config = _load("grasp_pipeline_remote_mujoco_ur5e.yaml")
        self.assertEqual(config["camera"]["type"], "remote_mujoco")
        self.assertEqual(config["manipulator"]["driver"], "remote_mujoco")
        self.assertIn("host", config["remote_mujoco"])
        self.assertIn("port", config["remote_mujoco"])
        self.assertFalse(config["mobile_base"]["enabled"])

    def test_both_asr_backends_have_separate_configuration(self):
        config = _load("grasp_pipeline.yaml")["voice"]["asr"]
        self.assertIn("sensevoice", config)
        self.assertIn("qwen3_asr", config)
        self.assertIn("model_dir", config["sensevoice"])
        self.assertIn("endpoint", config["qwen3_asr"])


if __name__ == "__main__":
    unittest.main()
