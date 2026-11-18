#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Recorded calibration contracts without a physical camera or serial device."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location(
    "calibrate_hand_eye", ROOT / "scripts/calibrate_hand_eye.py"
)
calibration = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = calibration
SPEC.loader.exec_module(calibration)

MANIPULATION = next(
    (
        parent / "components/control/manipulation"
        for parent in ROOT.parents
        if (parent / "components/control/manipulation").is_dir()
    ),
    None,
)
if MANIPULATION is not None:
    sys.path.insert(0, str(MANIPULATION / "bindings/python"))
    import spacemit_manipulation

    for module in (
        "capabilities/calibration/python",
        "capabilities/calibration/cli",
        "capabilities/kinematics/python",
    ):
        spacemit_manipulation.__path__.append(str(MANIPULATION / module))

import hand_eye_collection as collection  # noqa: E402
from spacemit_manipulation.calibration_session import read_dataset  # noqa: E402
from spacemit_manipulation.sampling import FeedbackPort  # noqa: E402

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


class ExistingCalibrationContracts(unittest.TestCase):
    def test_existing_dataset_transform_and_units_roundtrip(self):
        with tempfile.TemporaryDirectory(prefix="grasp-calibration-") as directory:
            root = Path(directory)
            (root / "samples").mkdir()
            (root / "images").mkdir()
            base = calibration._make_transform([0.23, -0.11, 0.36], [0.4, -0.3, 0.2])
            camera = calibration._make_transform([0.02, 0.08, 0.31], [-0.2, 0.1, 0.3])
            calibration._json_dump(root / "manifest.json", {"camera": {"backend": "realsense"}})
            image = np.zeros((8, 8, 3), np.uint8)
            calibration._save_sample(root, 1, image, image, [0.1] * 5, base, camera, {})
            _, _, rotations, positions, boards, translations = calibration._load_dataset(root)
            np.testing.assert_allclose(calibration._T_from_rt(rotations[0], positions[0]), base)
            np.testing.assert_allclose(calibration._T_from_rt(boards[0], translations[0]), camera)

    def test_las2_keeps_rectified_logical_left_view(self):
        camera = calibration.Las2CalibrationCamera.__new__(calibration.Las2CalibrationCamera)
        camera.eye_width, camera.eye_height = 12, 8
        camera.map_x = camera.map_y = None
        frame = np.zeros((8, 184, 3), np.uint8)
        frame[:, 160:172] = 20
        frame[:, 172:] = 70
        camera.capture_device = Mock()
        camera.capture_device.read.return_value = (True, frame)
        with patch.object(calibration.cv2, "remap", side_effect=lambda image, *a, **kw: image):
            result = camera.capture()
        self.assertEqual(result.shape, (8, 12, 3))
        self.assertTrue(np.all(result == 70))

    def test_apply_preserves_other_backend_and_creates_backup(self):
        with tempfile.TemporaryDirectory(prefix="grasp-calibration-") as directory:
            path = Path(directory) / "pipeline.yaml"
            original = (
                "camera:\n  type: realsense\ncalibration:\n"
                "  spacemit_las2:\n    T_base_camera:\n"
                "      translation: [1, 2, 3]\n      rotation: [0, 0, 0]\n"
            )
            path.write_text(original)
            backup = calibration._apply_to_grasp_config(
                path, "realsense", [0.1, 0.2, 0.3], [0, 0, 0]
            )
            self.assertEqual(Path(backup).read_text(), original)
            result = calibration.yaml.safe_load(path.read_text())
            self.assertEqual(
                result["calibration"]["spacemit_las2"]["T_base_camera"]["translation"], [1, 2, 3]
            )
            self.assertEqual(
                result["calibration"]["realsense"]["T_base_camera"]["translation"], [0.1, 0.2, 0.3]
            )


class CollectionContracts(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="grasp-readonly-calibration-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.plan = {
            "schema_version": 3,
            "device": {"arm": {"joints": [{"name": name} for name in JOINT_NAMES]}},
        }
        self.model = {
            "urdf": str(ROOT / "urdf/so101.urdf"),
            "base": "base_link",
            "tip": "gripper_frame_link",
            "joints": JOINT_NAMES,
        }
        self.pipeline = {
            "manipulator": {
                "device_plan": "device.json",
                "model": "model.json",
                "calibration_resources": ["encoder.json"],
            },
            "camera": {"type": "realsense", "realsense": {"width": 640, "height": 480}},
        }
        (self.root / "device.json").write_text(json.dumps(self.plan))
        (self.root / "model.json").write_text(json.dumps(self.model))
        (self.root / "encoder.json").write_text('{"verified_test_fixture": true}')
        self.save_pipeline()
        self.args = SimpleNamespace(
            config=str(self.root / "pipeline.json"),
            device_plan=None,
            model=None,
            camera_type="auto",
            las2_calib_path=None,
            las2_video_device=None,
            calibration_resource=[],
            installation_id="table-one",
            num_poses=12,
            no_quality_gate=False,
            capture_settle_ms=0,
            board_type="aruco",
            aruco_dict="4x4_50",
            marker_length=0.04,
            marker_id=0,
            device_executable="manipulation-device",
            dataset_dir=str(self.root / "session"),
            output=None,
            save_images=None,
            confirm_joint_reference=False,
            min_relative_rotation_deg=0,
            min_poses=12,
            max_reproj_error=2.0,
            min_marker_size_px=40,
            min_distance=0.1,
            max_distance=0.4,
        )

    def save_pipeline(self):
        (self.root / "pipeline.json").write_text(json.dumps(self.pipeline))

    def test_capture_answer_accepts_enter_and_keeps_quit(self):
        self.assertEqual(collection.parse_capture_answer(""), "capture")  # Enter alone.
        self.assertEqual(collection.parse_capture_answer("   "), "capture")
        self.assertEqual(collection.parse_capture_answer("CAPTURE"), "capture")
        self.assertEqual(collection.parse_capture_answer("capture"), "capture")
        self.assertEqual(collection.parse_capture_answer(" q "), "quit")
        self.assertEqual(collection.parse_capture_answer("Q"), "quit")
        self.assertIsNone(collection.parse_capture_answer("x"))  # Still not a capture.

    def test_small_marker_is_a_warning_not_a_rejection(self):
        """A small but precise board still contributes: the far poses constrain depth."""
        args = SimpleNamespace(
            min_marker_size_px=40.0,
            min_distance=0.10,
            max_distance=0.40,
            max_reproj_error=2.0,
            min_relative_rotation_deg=0.0,
            min_poses=1,
            no_quality_gate=False,
        )
        info = {
            "marker_size_px": 20.2,
            "reprojection_error_px": 0.4,
            "distance_m": 0.28,
            "detected_frames": 3,
            "num_frames": 3,
        }
        accepted, quality = calibration.check_capture_quality(np.eye(4), np.eye(4), info, [], args)
        self.assertTrue(accepted)
        self.assertEqual(quality["reject_reasons"], [])
        self.assertTrue(any("marker small" in text for text in quality["warnings"]))
        # The size reading stays in the record, so the distribution is still inspectable.
        self.assertEqual(quality["marker_size_px"], 20.2)

    def test_reprojection_error_still_rejects_a_small_marker(self):
        args = SimpleNamespace(
            min_marker_size_px=40.0,
            min_distance=0.10,
            max_distance=0.40,
            max_reproj_error=2.0,
            min_relative_rotation_deg=0.0,
            min_poses=1,
            no_quality_gate=False,
        )
        info = {
            "marker_size_px": 20.2,
            "reprojection_error_px": 3.5,
            "distance_m": 0.28,
            "detected_frames": 3,
            "num_frames": 3,
        }
        accepted, quality = calibration.check_capture_quality(np.eye(4), np.eye(4), info, [], args)
        self.assertFalse(accepted)
        self.assertTrue(any("reprojection error" in text for text in quality["reject_reasons"]))

    def test_a_partially_seen_board_is_flagged_but_still_accepted(self):
        """Fewer corners still solve a pose, so warn: the residual it adds is the issue.

        Measured on one 15-pose set, 7-8 corner samples averaged 7 mm against 4 mm for a
        fully seen board, which alone decides whether the default gate passes.
        """
        args = SimpleNamespace(
            min_marker_size_px=40.0,
            min_distance=0.10,
            max_distance=0.60,
            max_reproj_error=2.0,
            min_relative_rotation_deg=0.0,
            min_poses=1,
            min_charuco_corners=9,
            no_quality_gate=False,
        )
        base = {
            "marker_size_px": 16.3,
            "reprojection_error_px": 0.08,
            "distance_m": 0.507,
            "detected_frames": 3,
            "num_frames": 3,
        }
        partial, quality = calibration.check_capture_quality(
            np.eye(4), np.eye(4), dict(base, max_charuco_corners=8), [], args
        )
        self.assertTrue(partial)
        self.assertEqual(quality["reject_reasons"], [])
        self.assertTrue(any("charuco corners 8" in text for text in quality["warnings"]))

        full, quality = calibration.check_capture_quality(
            np.eye(4), np.eye(4), dict(base, max_charuco_corners=12), [], args
        )
        self.assertTrue(full)
        self.assertFalse(any("charuco corners" in text for text in quality["warnings"]))

    def test_paths_are_relative_to_owner_and_joint_order_is_exact(self):
        plan, model, fk, _, _, files, _ = collection.load_resources(self.args)
        self.assertEqual(plan, self.root / "device.json")
        self.assertEqual(model, self.root / "model.json")
        self.assertIn(self.root / "encoder.json", files)
        pose = np.asarray(fk.forward([0.0] * 5)).reshape(4, 4)
        self.assertTrue(np.isfinite(pose).all())
        self.plan["device"]["arm"]["joints"].reverse()
        plan.write_text(json.dumps(self.plan))
        with self.assertRaisesRegex(ValueError, "joint order"):
            collection.load_resources(self.args)

    def test_missing_plan_does_not_fall_back_to_serial_or_model_guess(self):
        del self.pipeline["manipulator"]["device_plan"]
        self.save_pipeline()
        with patch.object(collection, "DeviceClient") as connect:
            with self.assertRaisesRegex(ValueError, "explicit"):
                collection.load_resources(self.args)
        connect.assert_not_called()

    def test_old_ticks_formula_is_not_applied_to_public_joint_feedback(self):
        _, _, model, _, _, _, _ = collection.load_resources(self.args)
        client = Mock()
        client.state.return_value = dict(
            arm_sample_id=1, moving=False, fault=False, joints_rad=[0.0] * 5, source_time_s=1.0
        )
        result = FeedbackPort(client, model).read_joints()
        self.assertEqual(result["joints_rad"], [0.0] * 5)
        self.assertEqual([call[0] for call in client.method_calls], ["state"])
        self.assertFalse(hasattr(calibration, "init_so101"))
        self.assertFalse(hasattr(calibration, "_ticks_to_rad"))

    def test_public_fk_matches_recorded_legacy_so101_poses(self):
        # Recorded from the pre-migration URDFForwardKinematics with the existing app URDF.
        _, _, model, _, _, _, _ = collection.load_resources(self.args)
        fixtures = [
            (
                [0.2, -0.3, 0.4, -0.5, 0.6],
                [
                    [0.221054622249, 0.369144579030, 0.902699913457, 0.339665268270],
                    [-0.579272040252, 0.794332167705, -0.182976257290, -0.065572880864],
                    [-0.784588272513, -0.482461073167, 0.389425930715, 0.280036947006],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            ),
            (
                [-0.4, 0.2, -0.7, 0.3, -0.2],
                [
                    [0.081517636966, -0.422486785730, 0.902695846200, 0.363992226175],
                    [0.301687184628, 0.873680761965, 0.381663161444, 0.139164619815],
                    [-0.949915637041, 0.241219489378, 0.198679240107, 0.314679328171],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            ),
        ]
        for joints, expected in fixtures:
            np.testing.assert_allclose(
                np.asarray(model.forward(joints)).reshape(4, 4), expected, atol=1e-11, rtol=0
            )

    def test_reference_false_is_never_overridden_and_unknown_is_explicit(self):
        client = Mock()
        client.state.return_value = {"joint_reference_verified": False}
        with self.assertRaisesRegex(ValueError, "uncalibrated"):
            collection.check_joint_reference(client, True)
        client.state.return_value = {"joint_reference_verified": None}
        with self.assertRaisesRegex(ValueError, "confirm-joint-reference"):
            collection.check_joint_reference(client, False)
        self.assertEqual(
            collection.check_joint_reference(client, True),
            {"driver_verified": None, "operator_confirmed": True},
        )
        client.state.return_value = {"joint_reference_verified": True}
        self.assertEqual(
            collection.check_joint_reference(client, False),
            {"driver_verified": True, "operator_confirmed": False},
        )

    def test_resource_changes_reject_mixed_collection(self):
        files = collection.load_resources(self.args)[-2]
        fingerprints = {path: collection.digest(path) for path in files}
        (self.root / "encoder.json").write_text('{"changed": true}')
        with self.assertRaisesRegex(ValueError, "changed"):
            collection.check_resources(fingerprints)

    def test_empty_calibration_resources_are_not_implicitly_trusted(self):
        self.pipeline["manipulator"]["calibration_resources"] = []
        self.save_pipeline()
        with self.assertRaisesRegex(ValueError, "calibration_resources"):
            collection.load_resources(self.args)

    def test_retargeted_calibration_symlink_is_detected(self):
        link = self.root / "encoder_link.json"
        link.symlink_to(self.root / "encoder.json")
        self.pipeline["manipulator"]["calibration_resources"] = [link.name]
        self.save_pipeline()
        files = collection.load_resources(self.args)[-2]
        fingerprints = {path: collection.digest(path) for path in files}
        alternate = self.root / "encoder_other.json"
        alternate.write_text('{"another_encoder_reference": true}')
        link.unlink()
        link.symlink_to(alternate)
        with self.assertRaisesRegex(ValueError, "resource changed"):
            collection.check_resources(fingerprints)

    def test_tool_attachment_must_not_change_between_samples(self):
        client = Mock()
        client.state.return_value = {"effector_feedback": "position", "effector_positions": [0.4]}
        self.assertEqual(collection.read_tool_position(client, [0.4]), [0.4])
        with self.assertRaisesRegex(ValueError, "target must stay rigid"):
            collection.read_tool_position(client, [0.2])
        client.state.return_value["effector_command_active"] = True
        with self.assertRaisesRegex(ValueError, "stationary"):
            collection.read_tool_position(client, [0.4])

    def test_marker_coordinates_and_legacy_quality_gate_are_preserved(self):
        transform = calibration._make_transform([0.02, -0.01, 0.25], [0.2, -0.1, 0.3])
        image = np.zeros((8, 8, 3), np.uint8)
        detector = collection.BoardDetector(self.args)
        with patch.object(
            calibration,
            "detect_board",
            return_value=(
                transform,
                image,
                {
                    "reprojection_error_px": 0.4,
                    "marker_size_px": 50,
                    "marker_id": 0,
                },
            ),
        ):
            observed, quality, _ = detector.detect(image, np.eye(3), np.zeros(5))
        np.testing.assert_allclose(observed, transform)
        self.assertEqual(quality["reprojection_rms_px"], 0.4)
        self.assertEqual(detector.cfg["marker_length_m"], 0.04)
        with patch.object(
            calibration,
            "detect_board",
            return_value=(
                transform,
                image,
                {
                    "reprojection_error_px": 20,
                    "marker_size_px": 50,
                    "marker_id": 0,
                },
            ),
        ):
            with self.assertRaisesRegex(ValueError, "reprojection"):
                detector.detect(image, np.eye(3), np.zeros(5))

    def test_marker_identity_is_not_silently_changed(self):
        detector = collection.BoardDetector(self.args)
        with patch.object(
            calibration,
            "detect_board",
            return_value=(
                np.eye(4),
                None,
                {
                    "marker_id": 1,
                    "reprojection_error_px": 0.1,
                    "marker_size_px": 60,
                },
            ),
        ):
            with self.assertRaisesRegex(ValueError, "fixed calibration target"):
                detector.detect(None, np.eye(3), np.zeros(5))
        self.args.marker_id = None
        with self.assertRaisesRegex(ValueError, "marker-id"):
            collection.BoardDetector(self.args)

    def test_faulted_or_moving_tool_and_invalid_joint_feedback_are_refused(self):
        _, _, model, _, _, _, _ = collection.load_resources(self.args)
        state = dict(
            arm_sample_id=1, moving=False, fault=False, joints_rad=[0.0] * 5, source_time_s=1.0
        )
        client = Mock()
        for field in ("latched", "effector_moving", "effector_command_active", "effector_fault"):
            client.state.return_value = dict(state, **{field: True})
            with self.assertRaisesRegex(RuntimeError, "stationary"):
                FeedbackPort(client, model).read_joints()
        for joints in ([0.0] * 4, [float("nan")] * 5):
            client.state.return_value = dict(state, joints_rad=joints)
            with self.assertRaises(RuntimeError):
                FeedbackPort(client, model).read_joints()

    def test_public_session_collection_has_readonly_handoffs_and_cleanup(self):
        self.pipeline["camera"]["realsense"].update(width=8, height=8, fps=30, serial="test-camera")
        self.save_pipeline()
        _, _, model, _, _, _, _ = collection.load_resources(self.args)
        random = np.random.default_rng(1801)
        targets = iter(random.uniform(-0.7, 0.7, (12, 5)))
        joints = [0.0] * 5
        base_camera = calibration._make_transform([0.3, -0.2, 0.5], [0.2, -0.3, 0.4])
        flange_board = calibration._make_transform([0.03, 0.02, 0.08], [-0.4, 0.2, 0.1])
        image = np.zeros((8, 8, 3), np.uint8)
        camera = Mock()
        camera.metadata = {
            "serial": "test-camera",
            "backend": "realsense",
            "width": 8,
            "height": 8,
            "fps": 30,
        }
        camera.camera_matrix, camera.dist_coeffs = np.eye(3), np.zeros(5)
        camera.capture.return_value = image
        client = Mock()
        counter = iter(range(1, 2000))
        client.state.side_effect = lambda **kw: dict(
            arm_sample_id=next(counter),
            joints_rad=list(joints),
            source_time_s=1.0,
            moving=False,
            fault=False,
            joint_reference_verified=True,
        )
        owner = Mock()

        def next_pose():
            joints[:] = next(targets)
            return client

        def board_observation(*unused):
            pose = np.asarray(model.forward(joints)).reshape(4, 4)
            transform = np.linalg.inv(base_camera) @ pose @ flange_board
            return (
                transform,
                {
                    "reprojection_rms_px": 0.2,
                    "reprojection_error_px": 0.2,
                    "marker_size_px": 50,
                    "distance_m": 0.25,
                },
                image,
            )

        owner.__enter__ = Mock(side_effect=next_pose)
        owner.__exit__ = Mock()
        with patch.object(collection.shutil, "which", return_value="test-device"), patch.object(
            calibration, "_create_pipeline_camera", return_value=camera
        ), patch.object(collection, "DeviceClient", return_value=owner) as factory, patch.object(
            collection.BoardDetector,
            "detect",
            side_effect=board_observation,
        ), patch("spacemit_manipulation.sampling.time.sleep"), patch(
            "builtins.input", return_value="CAPTURE"
        ), patch.object(collection, "solve_collection") as solve:
            collection.collect(self.args)
        self.assertEqual(factory.call_count, 12)
        self.assertTrue(
            all(call.kwargs == {"allow_motion": False} for call in factory.call_args_list)
        )
        self.assertTrue(all(call[0] == "state" for call in client.method_calls))
        self.assertEqual(owner.__exit__.call_count, 12)
        camera.stop.assert_called_once()
        solve.assert_not_called()
        manifest, samples = read_dataset(self.root / "session")
        self.assertEqual(len(samples), 12)
        self.assertEqual(manifest["session_identity"]["device"]["installation_id"], "table-one")
        np.testing.assert_allclose(manifest["T_flange_tcp"], np.eye(4))
        self.assertEqual(
            samples[0]["joint_reference"], {"driver_verified": True, "operator_confirmed": False}
        )
        self.assertFalse((self.root / "session/result.json").exists())
        self.args.apply_config = self.args.config
        collection.solve_collection(self.args, self.root / "session")
        report = json.loads((self.root / "session/result.json").read_text())
        np.testing.assert_allclose(report["T_base_camera"], base_camera, atol=1e-7)
        applied = calibration.yaml.safe_load(Path(self.args.config).read_text())
        exported = applied["calibration"]["realsense"]["T_base_camera"]
        np.testing.assert_allclose(
            calibration._make_transform(exported["translation"], exported["rotation"]),
            base_camera,
            atol=1e-5,
        )
        self.pipeline["camera"]["realsense"]["serial"] = "other-camera"
        self.save_pipeline()
        with self.assertRaisesRegex(ValueError, "camera serial"):
            collection.validate_apply_target(self.args, report, "realsense")
        self.pipeline["camera"]["realsense"].update(serial="test-camera", width=16)
        self.save_pipeline()
        with self.assertRaisesRegex(ValueError, "image profile"):
            collection.validate_apply_target(self.args, report, "realsense")
        (self.root / "encoder.json").write_text('{"changed_after_capture": true}')
        with self.assertRaisesRegex(ValueError, "joint calibration"):
            collection.validate_apply_target(self.args, report, "realsense")

    def test_abort_keeps_incomplete_marker_and_closes_camera(self):
        camera = Mock(
            metadata={"serial": "test-camera"}, camera_matrix=np.eye(3), dist_coeffs=np.zeros(5)
        )
        with patch.object(collection.shutil, "which", return_value="test-device"), patch.object(
            calibration, "_create_pipeline_camera", return_value=camera
        ), patch("builtins.input", return_value="q"), patch.object(
            collection, "DeviceClient"
        ) as factory:
            with self.assertRaisesRegex(RuntimeError, "incomplete"):
                collection.collect(self.args)
        factory.assert_not_called()
        camera.stop.assert_called_once()
        with self.assertRaisesRegex(ValueError, "incomplete"):
            read_dataset(self.root / "session")


if __name__ == "__main__":
    unittest.main()
