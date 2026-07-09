#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression checks for grasp-depth pixel handling."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PIPELINE_CPP = ROOT / "src" / "grasp_pipeline.cpp"


class GraspDepthSourceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pipeline = PIPELINE_CPP.read_text(encoding="utf-8")

    def test_grasp_pixel_is_clamped_to_depth_image(self):
        self.assertIn("int ClampPixel", self.pipeline)
        self.assertIn("current_depth_.cols", self.pipeline)
        self.assertIn("current_depth_.rows", self.pipeline)
        self.assertIn("clamped from", self.pipeline)

    def test_depth_falls_back_to_target_center(self):
        self.assertIn("MedianDepthAtPixel", self.pipeline)
        self.assertIn("fallback to", self.pipeline)
        self.assertIn("target center", self.pipeline)
        self.assertIn("Target depth invalid at grasp pixel and center",
                      self.pipeline)


if __name__ == "__main__":
    unittest.main()
