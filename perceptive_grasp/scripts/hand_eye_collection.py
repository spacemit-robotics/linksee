#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Application camera adapter for the public, read-only manipulation calibration session."""

from contextlib import ExitStack
import copy
import json
import math
from pathlib import Path
import shutil
import time

import numpy as np
import yaml

from spacemit_manipulation import DeviceClient
from spacemit_manipulation.calibration_session import (
    CalibrationSession,
    device_identity,
    digest,
    export_perceptive,
    session_identity,
    solve_session,
)
from spacemit_manipulation.charuco_sampler import write_image_new
from spacemit_manipulation.sampling import FeedbackPort, capture_sample
from spacemit_manipulation.urdf_fk import UrdfKinematics
from spacemit_manipulation.workbench import save_new

import calibrate_hand_eye as application


def resolve_file(value, directory, *, preserve_reference=False):
    """Resolve an explicit resource against its owning configuration."""
    if not isinstance(value, str) or not value.strip():
        raise ValueError("device_plan and model must be explicit file paths")
    path = directory / application.expand_config_path(value)
    path = path.absolute() if preserve_reference else path.resolve()
    if not path.is_file():
        raise FileNotFoundError(str(path))
    return path


def load_resources(args):
    """Validate model/plan order without contacting a device or inferring encoder zeroes."""
    pipeline_path = Path(args.config).resolve()
    pipeline = yaml.safe_load(pipeline_path.read_text())
    if not isinstance(pipeline, dict) or not isinstance(pipeline.get("manipulator"), dict):
        raise ValueError("pipeline requires manipulator.device_plan and manipulator.model")
    arm = pipeline["manipulator"]
    plan_path = resolve_file(args.device_plan or arm.get("device_plan"), pipeline_path.parent)
    model_path = resolve_file(args.model or arm.get("model"), pipeline_path.parent)
    model_config = json.loads(model_path.read_text())
    urdf_path = resolve_file(model_config.get("urdf"), model_path.parent)
    model = UrdfKinematics(
        urdf_path, model_config["base"], model_config["tip"], model_config["joints"]
    )
    plan = yaml.safe_load(plan_path.read_text())
    if not isinstance(plan, dict) or plan.get("schema_version") != 3:
        raise ValueError("calibration requires a schema_version: 3 device plan")
    device = plan["device"]
    configured_names = [joint["name"] for joint in device["arm"]["joints"]]
    if configured_names != model.names:
        raise ValueError("device plan joint order disagrees with kinematic model")
    if arm.get("tip_link", model_config["tip"]) != model_config["tip"]:
        raise ValueError("pipeline tip frame disagrees with calibration model")
    if arm.get("base_link", model_config["base"]) != model_config["base"]:
        raise ValueError("pipeline base frame disagrees with calibration model")
    if arm.get("urdf_path"):
        pipeline_urdf = resolve_file(arm["urdf_path"], pipeline_path.parent)
        if digest(pipeline_urdf) != digest(urdf_path):
            raise ValueError("pipeline and calibration must use the same URDF")
    files = [pipeline_path, plan_path, model_path, urdf_path]
    calibration_paths = arm.get("calibration_resources", [])
    if not isinstance(calibration_paths, list):
        raise ValueError("manipulator.calibration_resources must list calibration file paths")
    calibration_files = [
        # Drivers reopen calibration resources on every handoff. Keep their symbolic paths so a
        # retargeted symlink is detected instead of continuing to hash the old resolved target.
        resolve_file(value, pipeline_path.parent, preserve_reference=True)
        for value in calibration_paths + args.calibration_resource
    ]
    if not calibration_files and not args.confirm_joint_reference:
        raise ValueError("manipulator.calibration_resources must identify the joint calibration")
    files.extend(calibration_files)
    camera_config = copy.deepcopy(pipeline.get("camera", {}))
    backend = application._resolve_camera_type(camera_config, args.camera_type)
    if backend == "spacemit_las2":
        settings = camera_config.get(backend, {})
        calibration_path = resolve_file(
            args.las2_calib_path or settings.get("calib_path"), pipeline_path.parent
        )
        settings["calib_path"] = str(calibration_path)
        files.append(calibration_path)
        if not settings.get("serial"):
            raise ValueError("camera.spacemit_las2.serial must identify the installed camera")
    return plan_path, model_path, model, camera_config, backend, files, calibration_files


class BoardDetector:
    """Retain the application optical frame, physical board dimensions and detection gates."""

    def __init__(self, args):
        self.args = args
        self.cfg = {"type": args.board_type, "dictionary": args.aruco_dict}
        if args.board_type == "charuco":
            self.cfg.update(
                squares_x=args.charuco_squares_x,
                squares_y=args.charuco_squares_y,
                square_length_m=args.charuco_square_length,
                marker_length_m=args.charuco_marker_length,
            )
            if not (
                args.charuco_squares_x >= 2
                and args.charuco_squares_y >= 2
                and 0 < args.charuco_marker_length < args.charuco_square_length
            ):
                raise ValueError("invalid ChArUco board dimensions in metres")
        else:
            self.cfg["marker_length_m"] = args.marker_length
            if not math.isfinite(args.marker_length) or args.marker_length <= 0:
                raise ValueError("marker length must be positive metres")
            if args.marker_id is None or not 0 <= args.marker_id < 100:
                raise ValueError("ArUco collection requires --marker-id in the selected dictionary")
            dictionary_size = int(args.aruco_dict.rsplit("_", 1)[1])
            if args.marker_id >= dictionary_size:
                raise ValueError("--marker-id is outside the selected dictionary")
            self.cfg["marker_id"] = args.marker_id

    def detect(self, image, camera_matrix, dist_coeffs):
        transform, annotated, info = application.detect_board(
            image, camera_matrix, dist_coeffs, self.args, return_info=True
        )
        if transform is None:
            raise ValueError("board not detected: " + application._format_detect_info(info))
        if self.args.board_type == "aruco" and info.get("marker_id") != self.args.marker_id:
            raise ValueError("detected ArUco marker differs from the fixed calibration target")
        error, size = info.get("reprojection_error_px"), info.get("marker_size_px")
        if error is None or size is None or not np.isfinite([error, size]).all():
            raise ValueError("board detection has no finite reprojection/size evidence")
        info["distance_m"] = float(np.linalg.norm(transform[:3, 3]))
        accepted, quality = application.check_capture_quality(
            np.eye(4), transform, info, [], self.args
        )
        if not accepted:
            raise ValueError("; ".join(quality["reject_reasons"]))
        return transform, dict(info, reprojection_rms_px=float(error)), annotated


def camera_identity(camera, camera_config, backend):
    """Bind the actual image geometry and optical convention, including LAS2 rectification."""
    metadata = dict(camera.metadata)
    serial = metadata.get("serial") or camera_config.get(backend, {}).get("serial")
    if not isinstance(serial, str) or not serial.strip():
        raise ValueError("camera identity is required")
    metadata.update(
        serial=serial,
        frame="color_optical",
        backend=backend,
        camera_matrix=np.asarray(camera.camera_matrix).tolist(),
        dist_coeffs=np.asarray(camera.dist_coeffs).tolist(),
        crop="full_rectified_logical_left" if backend == "spacemit_las2" else "full_color",
    )
    # Bind file contents instead of machine-specific device and calibration paths.
    metadata.pop("video_device", None)
    metadata.pop("calib_path", None)
    if backend == "spacemit_las2":
        metadata["intrinsic_calibration_sha256"] = digest(camera_config[backend]["calib_path"])
    return metadata


def check_resources(expected):
    """Refuse mixed configuration/calibration sessions, including changes between handoffs."""
    if any(digest(path) != fingerprint for path, fingerprint in expected.items()):
        raise ValueError("calibration resource changed during collection; start a new session")


def check_joint_reference(client, operator_confirmed):
    """Separate driver evidence from an explicit one-session operator declaration."""
    state = client.state()
    reference = state.get("joint_reference_verified")
    if reference is False:
        raise ValueError("device joint reference is uncalibrated; use the device calibration tool")
    if reference is not None and type(reference) is not bool:
        raise ValueError("invalid joint reference evidence")
    if reference is None and not operator_confirmed:
        raise ValueError(
            "driver does not verify joint reference; confirm the installed model and encoder "
            "zeroes, then explicitly pass --confirm-joint-reference"
        )
    return {"driver_verified": reference, "operator_confirmed": reference is None}


def read_tool_position(client, expected=None):
    """Keep a measured tool's board attachment unchanged throughout the session."""
    state = client.state()
    if any(
        state.get(field, False)
        for field in (
            "effector_moving",
            "effector_command_active",
            "effector_fault",
        )
    ):
        raise ValueError("end effector must remain stationary and healthy during calibration")
    if state.get("effector_feedback") != "position":
        return None
    positions = state.get("effector_positions", [])
    if not positions or not np.isfinite(positions).all():
        raise ValueError("invalid end-effector position feedback")
    if expected is not None and (
        len(positions) != len(expected) or np.max(np.abs(np.asarray(positions) - expected)) > 0.001
    ):
        raise ValueError(
            "end-effector position changed; the target must stay rigid to the model tip"
        )
    return list(positions)


def collect(args):
    """Collect stationary samples with one read-only device owner per explicit handoff.

    The marker is rigid relative to the model tip. T_tip_marker is estimated by
    hand-eye solving; no TCP/marker offset or encoder correction is guessed here.
    """
    if not args.installation_id or not args.installation_id.strip():
        raise ValueError("--installation-id must identify this fixed installation")
    if not 12 <= args.num_poses <= 100:
        raise ValueError("--num-poses must be 12..100")
    if args.no_quality_gate:
        raise ValueError("shared collection does not support --no-quality-gate")
    if args.capture_settle_ms < 0:
        raise ValueError("--capture-settle-ms must be nonnegative")
    if args.output or getattr(args, "apply_config", None):
        raise ValueError("--output and --apply-config require --solve-only")
    detector = BoardDetector(args)
    plan_path, model_path, model, config, backend, files, calibration_files = load_resources(args)
    executable = shutil.which(args.device_executable)
    if executable is None:
        raise FileNotFoundError("manipulation-device not found; load the SDK environment")
    output = (
        Path(args.dataset_dir).resolve()
        if args.dataset_dir
        else (Path.cwd() / "hand_eye_datasets" / time.strftime("handeye_%Y%m%d_%H%M%S"))
    )
    if output.exists():
        raise FileExistsError(str(output))
    if args.output and Path(args.output).exists():
        raise FileExistsError(str(args.output))
    fingerprints = {path: digest(path) for path in files}
    with ExitStack() as resources:
        camera = application._create_pipeline_camera(config, backend, args.las2_video_device, None)
        resources.callback(camera.stop)
        identity = camera_identity(camera, config, backend)
        provenance = {
            "camera": identity,
            "board": detector.cfg,
            "resource_sha256": list(fingerprints.values()),
        }
        device = device_identity(plan_path, model_path, installation_id=args.installation_id)
        device["pipeline_sha256"] = fingerprints[Path(args.config).resolve()]
        device["calibration_resources_sha256"] = [fingerprints[path] for path in calibration_files]
        device["joint_reference_operator_confirmation"] = bool(args.confirm_joint_reference)
        session = CalibrationSession(output, session_identity(device, provenance), provenance)
        accepted_poses = []
        tool_position = None
        while len(session.samples) < args.num_poses:
            check_resources(fingerprints)
            answer = input(
                f"样本 {len(session.samples) + 1}/{args.num_poses}："
                "按现场规程调整并静止，退出其他控制程序后输入 CAPTURE；输入 q 结束："
            ).strip()
            if answer.lower() == "q":
                raise RuntimeError("collection incomplete; accepted samples retained")
            if answer != "CAPTURE":
                continue
            check_resources(fingerprints)
            # DeviceClient owns the same public resource lease while all frame brackets are read.
            # No enable/disable, target, recovery or direct serial operation is requested.
            with DeviceClient(executable, plan_path, allow_motion=False) as client:
                reference = check_joint_reference(client, args.confirm_joint_reference)
                if reference["driver_verified"] is True and not calibration_files:
                    raise ValueError(
                        "driver-verified calibration requires explicit calibration_resources"
                    )
                sample_tool_position = read_tool_position(client, tool_position)
                port = FeedbackPort(client, model)
                if args.capture_settle_ms:
                    time.sleep(args.capture_settle_ms / 1000.0)
                try:
                    record, image, annotated = capture_sample(port, camera, detector)
                except ValueError as error:
                    print(f"样本未收录：{error}")
                    continue
                check_resources(fingerprints)
                read_tool_position(client, sample_tool_position)
                if camera_identity(camera, config, backend) != identity:
                    raise ValueError("camera identity or intrinsics changed during collection")
                pose = np.asarray(record["T_base_flange"])
                accepted, quality = application.check_capture_quality(
                    pose,
                    np.asarray(record["T_camera_board"]),
                    record["quality"],
                    accepted_poses,
                    args,
                )
                if not accepted:
                    print("样本未收录：" + "; ".join(quality["reject_reasons"]))
                    continue
                prefix = output / f"sample_{len(session.samples):04d}"
                write_image_new(f"{prefix}.png", image)
                write_image_new(f"{prefix}_detected.png", annotated)
                record.update(
                    image=prefix.name + ".png",
                    visualization=prefix.name + "_detected.png",
                    joint_reference=reference,
                    effector_positions=sample_tool_position,
                )
                if args.save_images:
                    extra = Path(args.save_images).resolve()
                    extra.mkdir(parents=True, exist_ok=True)
                    write_image_new(extra / (prefix.name + "_detected.png"), annotated)
                session.append(record)
                tool_position = sample_tool_position
                accepted_poses.append(pose)
                print(f"已采集 {len(session.samples)}/{args.num_poses}")
        check_resources(fingerprints)
        session.finish()
    print(f"采集数据集：{output}")


def validate_apply_target(args, report, backend):
    """Check the destination model, calibration resources and camera without opening hardware."""
    target_args = copy.copy(args)
    target_args.config = args.apply_config
    target_args.device_plan = target_args.model = None
    target_args.calibration_resource = []
    target_args.camera_type = backend
    target_args.las2_calib_path = target_args.las2_video_device = None
    target_args.confirm_joint_reference = True
    plan, model, _, camera_config, _, _, calibration_files = load_resources(target_args)
    expected = report["session_identity"]["device"]
    actual = device_identity(plan, model, installation_id=args.installation_id)
    for field in (
        "device_plan_sha256",
        "model_sha256",
        "urdf_sha256",
        "base_frame",
        "flange_frame",
        "joints",
    ):
        if actual[field] != expected[field]:
            raise ValueError("destination device/model differs from the calibration session")
    if sorted(digest(path) for path in calibration_files) != sorted(
        expected.get("calibration_resources_sha256", [])
    ):
        raise ValueError("destination joint calibration differs from the calibration session")
    settings = camera_config.get(backend)
    if not isinstance(settings, dict):
        raise ValueError("destination has no matching camera profile")
    camera = report["camera"]
    serial = settings.get("serial")
    if serial is not None and str(serial) != camera["serial"]:
        raise ValueError("destination camera serial differs from the calibration session")
    if serial is None and digest(args.apply_config) != expected.get("pipeline_sha256"):
        raise ValueError(
            "changed destination must explicitly identify the calibrated camera serial"
        )
    if backend == "realsense":
        if any(
            int(settings.get(name, default)) != camera[name]
            for name, default in (("width", 640), ("height", 480), ("fps", 30))
        ):
            raise ValueError(
                "destination camera image profile differs from the calibration session"
            )
    elif digest(settings["calib_path"]) != camera.get("intrinsic_calibration_sha256"):
        raise ValueError("destination LAS2 intrinsics differ from the calibration session")


def solve_collection(args, directory):
    """Use the common solver and export only a quality-approved matching installation."""
    if not args.installation_id:
        raise ValueError("shared session export requires --installation-id")
    report = solve_session(directory)
    manifest = json.loads((Path(directory) / "manifest.json").read_text())
    installation = report["session_identity"]["device"]["installation_id"]
    if installation != args.installation_id:
        raise ValueError("dataset belongs to a different installation")
    backend = application._resolve_dataset_camera_type(manifest, args.camera_type)
    if args.apply_config:
        validate_apply_target(args, report, backend)
    destination = Path(args.output) if args.output else Path(directory) / "result.json"
    save_new(destination, report)
    fragment = Path(directory) / "perceptive_calibration.json"
    export_perceptive(report, fragment, backend, installation)
    if args.apply_config:
        transform = json.loads(fragment.read_text())["calibration"][backend]["T_base_camera"]
        application._apply_to_grasp_config(
            args.apply_config, backend, transform["translation"], transform["rotation"]
        )
    print(f"标定结果：{destination}")
    print(f"抓取外参：{fragment}")
