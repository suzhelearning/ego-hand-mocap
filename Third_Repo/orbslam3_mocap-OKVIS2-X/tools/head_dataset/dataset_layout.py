#!/usr/bin/env python3
"""Dataset layout helpers for head-v3 HDF5 and optional compact-v2 takes.

Head datasets are the public default.  Compact-v2 takes must be selected with
``--take``.  The module can materialize private compatibility views for older
ORB-SLAM utilities, but legacy EuRoC datasets are no longer accepted as input.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping, Sequence

import numpy as np
import yaml


COMPACT_SCHEMA = "quad_cam_dataset"
COMPACT_SCHEMA_VERSION = 2
HEAD_SCHEMA_VERSION = 3
CALIBRATION_SCHEMA = "quad_cam_streamer.calibration_full"
CALIBRATION_SCHEMA_VERSION = 2
ADAPTER_MARKER = ".orbslam3_euroc_adapter.json"
TAKE_ADAPTER_MARKER = ".orbslam3_take_adapter.json"

HEAD_CAMERA_LOGICAL_IDS = {
    "cam0": "RGB_left",
    "cam1": "RGB_right",
    "cam2": "IR_left",
    "cam3": "IR_right",
}

CAMERA_FIELDS = (
    "camera_id",
    "frame_index",
    "timestamp_ns",
    "pts_ns",
    "v4l2_sequence",
    "is_keyframe",
)
IMU_FIELDS = (
    "timestamp_ns",
    "imu_id",
    "gx_rad_s",
    "gy_rad_s",
    "gz_rad_s",
    "ax_m_s2",
    "ay_m_s2",
    "az_m_s2",
)


class DatasetLayoutError(ValueError):
    """Raised when a dataset does not satisfy the selected layout contract."""


def _run_h5dump_raw(h5_path: Path, dataset_path: str) -> bytes:
    """Read one HDF5 dataset through the system h5dump binary.

    The deployed Python environment has an h5py/numpy ABI mismatch, while the
    recorder image already ships the HDF5 command-line tools.  Native binary
    output avoids parsing h5dump's human-readable formatting and works for
    scalar strings as well as numeric arrays.
    """

    # Packaged Windows builds carry h5py; no external HDF5 tools are needed.
    # Retain h5dump on existing Linux deployments with their older NumPy ABI.
    if os.name == "nt" or os.environ.get("SLAM_H5_READER") == "h5py":
        try:
            import h5py
            with h5py.File(h5_path, "r") as handle:
                value = handle[dataset_path][()]
                if isinstance(value, bytes):
                    return value
                if isinstance(value, str):
                    return value.encode("utf-8")
                array = np.asarray(value)
                return array.astype(array.dtype.newbyteorder("="), copy=False).tobytes()
        except (OSError, KeyError, ImportError) as exc:
            raise DatasetLayoutError(f"failed to read {h5_path}:{dataset_path}: {exc}") from exc

    with tempfile.TemporaryDirectory(prefix="head-h5dump-") as directory:
        raw_path = Path(directory) / "data.bin"
        command = [
            "h5dump",
            "-d",
            dataset_path,
            "-b",
            "NATIVE",
            "-o",
            str(raw_path),
            "-O",
            os.devnull,
            str(h5_path),
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError as exc:
            raise DatasetLayoutError(
                "h5dump is required to read head datasets"
            ) from exc
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            raise DatasetLayoutError(
                f"failed to read {h5_path}:{dataset_path}: {detail}"
            )
        try:
            return raw_path.read_bytes()
        except OSError as exc:
            raise DatasetLayoutError(
                f"failed to collect {h5_path}:{dataset_path}: {exc}"
            ) from exc


def _h5_json_object(h5_path: Path, dataset_path: str, description: str) -> dict[str, Any]:
    try:
        text = _run_h5dump_raw(h5_path, dataset_path).rstrip(b"\0").decode("utf-8")
        value = json.loads(text)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise DatasetLayoutError(
            f"failed to parse {description} in {h5_path}:{dataset_path}: {exc}"
        ) from exc
    if not isinstance(value, dict):
        raise DatasetLayoutError(
            f"{description} root must be an object: {h5_path}:{dataset_path}"
        )
    return value


def _h5_array(h5_path: Path, dataset_path: str, dtype: Any) -> np.ndarray:
    raw = _run_h5dump_raw(h5_path, dataset_path)
    itemsize = np.dtype(dtype).itemsize
    if len(raw) % itemsize:
        raise DatasetLayoutError(
            f"invalid byte size for {h5_path}:{dataset_path}: {len(raw)}"
        )
    return np.frombuffer(raw, dtype=np.dtype(dtype).newbyteorder("=")).copy()


def _load_json_object(path: Path, description: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise DatasetLayoutError(f"{description} not found: {path}") from exc
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise DatasetLayoutError(f"failed to read {description} {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise DatasetLayoutError(f"{description} root must be an object: {path}")
    return value


def _require_file(path: Path, description: str) -> Path:
    if not path.is_file():
        raise DatasetLayoutError(f"{description} not found: {path}")
    return path


def _require_fields(
    actual: Sequence[str] | None,
    expected: Sequence[str],
    path: Path,
) -> None:
    if actual is None:
        raise DatasetLayoutError(f"CSV has no header: {path}")
    if tuple(actual) != tuple(expected):
        raise DatasetLayoutError(
            f"CSV header must be exactly {','.join(expected)}: {path}"
        )


def _finite_matrix(value: Any, size: int, label: str) -> np.ndarray:
    if isinstance(value, Mapping) and "data" in value:
        try:
            rows = int(value.get("rows", size))
            cols = int(value.get("cols", size))
        except (TypeError, ValueError, OverflowError) as exc:
            raise DatasetLayoutError(f"{label} has invalid matrix dimensions") from exc
        if rows != size or cols != size:
            raise DatasetLayoutError(f"{label} must be {size}x{size}")
        value = value["data"]
    try:
        matrix = np.asarray(value, dtype=float).reshape(size, size)
    except (TypeError, ValueError, OverflowError) as exc:
        raise DatasetLayoutError(
            f"{label} must contain {size * size} numeric values"
        ) from exc
    if not np.all(np.isfinite(matrix)):
        raise DatasetLayoutError(f"{label} contains non-finite values")
    return matrix


def _rigid_transform(value: Any, label: str) -> np.ndarray:
    matrix = _finite_matrix(value, 4, label)
    if not np.allclose(
        matrix[3], [0.0, 0.0, 0.0, 1.0], rtol=0.0, atol=1e-9
    ):
        raise DatasetLayoutError(f"{label} has an invalid homogeneous final row")
    rotation = matrix[:3, :3]
    determinant = float(np.linalg.det(rotation))
    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    if abs(determinant - 1.0) > 1e-3 or orthogonality_error > 1e-3:
        raise DatasetLayoutError(f"{label} rotation is not proper")
    return matrix


def _positive_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise DatasetLayoutError(f"{label} must be positive and finite")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise DatasetLayoutError(f"{label} must be positive and finite") from exc
    if not math.isfinite(number) or number <= 0.0:
        raise DatasetLayoutError(f"{label} must be positive and finite")
    return number


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise DatasetLayoutError(f"{label} must be finite")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise DatasetLayoutError(f"{label} must be finite") from exc
    if not math.isfinite(number):
        raise DatasetLayoutError(f"{label} must be finite")
    return number


def _positive_dimension(value: Any, label: str) -> int:
    number = _positive_number(value, label)
    dimension = int(number)
    if float(dimension) != number:
        raise DatasetLayoutError(f"{label} must be a positive integer")
    return dimension


def _runtime_gyro_bias_corrected(imu: Mapping[str, Any]) -> bool:
    six_face = imu.get("six_face")
    if not isinstance(six_face, Mapping) or six_face.get("status") not in {
        "valid",
        "measured",
        "calibrated",
    }:
        return False
    result = six_face.get("result")
    bias = result.get("bias") if isinstance(result, Mapping) else None
    scale = result.get("scale") if isinstance(result, Mapping) else None
    accel_bias = bias.get("accel") if isinstance(bias, Mapping) else None
    gyro_bias = bias.get("gyro") if isinstance(bias, Mapping) else None
    accel_scale = scale.get("accel") if isinstance(scale, Mapping) else None
    if not isinstance(accel_bias, list) or len(accel_bias) != 3:
        return False
    if not isinstance(gyro_bias, list) or len(gyro_bias) != 3:
        return False
    if not isinstance(accel_scale, list) or len(accel_scale) != 3:
        return False
    try:
        accel_values = [
            _finite_number(value, "six-face accelerometer bias")
            for value in accel_bias
        ]
        gyro_values = [
            _finite_number(value, "six-face gyro bias") for value in gyro_bias
        ]
        scale_values = [
            _positive_number(value, "six-face accelerometer scale")
            for value in accel_scale
        ]
    except DatasetLayoutError:
        return False
    return (
        len(accel_values) == 3
        and len(gyro_values) == 3
        and len(scale_values) == 3
    )


def _safe_dataset_file(
    root: Path,
    value: Any,
    label: str,
    source: str,
) -> Path:
    if not isinstance(value, str) or not value:
        raise DatasetLayoutError(f"{source}: {label} must be a relative file path")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise DatasetLayoutError(f"{source}: unsafe path for {label}: {value}")
    # Keep symlinks inside generated adapters valid: lexical normalization must
    # not follow the final symlink to the source dataset outside ``root``.
    path = Path(os.path.abspath(root / relative))
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise DatasetLayoutError(f"{source}: path escapes dataset for {label}: {value}") from exc
    return path


def _find_head_h5(root: Path) -> Path:
    preferred = root / f"{root.name}.h5"
    if preferred.is_file():
        return preferred
    candidates = sorted(path for path in root.glob("*.h5") if path.is_file())
    if not candidates:
        raise DatasetLayoutError(f"head HDF5 file not found: {root}/*.h5")
    if len(candidates) != 1:
        names = ", ".join(path.name for path in candidates)
        raise DatasetLayoutError(
            f"head dataset must contain exactly one .h5 file when {preferred.name} "
            f"is absent: {names}"
        )
    return candidates[0]


@dataclass(frozen=True)
class DatasetLayout:
    root: Path
    take: bool = False
    calibration_override: Path | None = None

    @classmethod
    def open(
        cls,
        root: str | os.PathLike[str],
        take: bool = False,
        calibration_file: str | os.PathLike[str] | None = None,
    ) -> "DatasetLayout":
        resolved = Path(root).expanduser().resolve()
        if not resolved.is_dir():
            raise DatasetLayoutError(f"dataset directory not found: {resolved}")
        override = (
            Path(calibration_file).expanduser().resolve()
            if calibration_file is not None
            else None
        )
        if override is not None:
            _require_file(override, "calibration override")
        layout = cls(resolved, bool(take), override)
        layout.load_manifest()
        return layout

    @property
    def head_h5(self) -> Path:
        if self.take:
            raise DatasetLayoutError("compact-v2 takes do not contain a head HDF5 file")
        return _find_head_h5(self.root)

    @property
    def format_name(self) -> str:
        return "take" if self.take else "head"

    @property
    def metadata_source(self) -> str:
        if self.take:
            return str(self.root / "dataset.json")
        return f"{self.head_h5}:/metadata/dataset.json"

    def load_manifest(self) -> dict[str, Any]:
        if self.take:
            manifest_path = self.root / "dataset.json"
            manifest = _load_json_object(manifest_path, "dataset.json")
            expected_version = COMPACT_SCHEMA_VERSION
        else:
            manifest_path = self.metadata_source
            manifest = _h5_json_object(
                self.head_h5,
                "/metadata/dataset.json",
                "embedded dataset.json",
            )
            expected_version = HEAD_SCHEMA_VERSION
        if manifest.get("schema") != COMPACT_SCHEMA:
            raise DatasetLayoutError(
                f"{manifest_path}: schema must be {COMPACT_SCHEMA!r}; "
                f"selected format is {self.format_name}"
            )
        version = manifest.get("schema_version")
        if type(version) is not int or version != expected_version:
            raise DatasetLayoutError(
                f"{manifest_path}: schema_version must be {expected_version}"
            )
        if manifest.get("complete") is not True:
            raise DatasetLayoutError(
                f"{manifest_path}: dataset is not finalized (complete must be true)"
            )
        clock = manifest.get("clock")
        if (
            not isinstance(clock, Mapping)
            or clock.get("domain") != "CLOCK_MONOTONIC"
        ):
            raise DatasetLayoutError(
                f"{manifest_path}: clock.domain must be CLOCK_MONOTONIC"
            )
        if (self.root / ".recording").exists():
            raise DatasetLayoutError(
                f"{self.root}: finalized processing refused while .recording exists"
            )
        return manifest

    def _camera_metadata(self, camera_id: str) -> Mapping[str, Any]:
        cameras = self.load_manifest().get("sensors", {}).get("cameras", [])
        if not isinstance(cameras, list):
            raise DatasetLayoutError(f"{self.metadata_source}: sensors.cameras is missing")
        for camera in cameras:
            if isinstance(camera, Mapping) and camera.get("id") == camera_id:
                return camera
        raise DatasetLayoutError(
            f"{self.metadata_source}: camera metadata not found for {camera_id}"
        )

    def _head_camera_logical_id(self, camera_id: str) -> str:
        camera = self._camera_metadata(camera_id)
        logical_id = camera.get("logical_id", HEAD_CAMERA_LOGICAL_IDS.get(camera_id))
        if not isinstance(logical_id, str) or not logical_id:
            raise DatasetLayoutError(
                f"{self.metadata_source}: logical_id missing for {camera_id}"
            )
        if "/" in logical_id or logical_id in {".", ".."}:
            raise DatasetLayoutError(
                f"{self.metadata_source}: invalid logical_id for {camera_id}: {logical_id}"
            )
        return logical_id

    def camera_video(self, camera_id: str) -> Path:
        manifest = self.load_manifest()
        files = manifest.get("files")
        videos = files.get("videos") if isinstance(files, Mapping) else None
        key = camera_id if self.take else self._head_camera_logical_id(camera_id)
        value = videos.get(key) if isinstance(videos, Mapping) else None
        if not isinstance(value, str) and self.take:
            # Early compact-v2 manifests did not always list the conventional
            # video paths explicitly.
            value = f"video/{camera_id}.mp4"
        if not isinstance(value, str) and not self.take:
            value = self._camera_metadata(camera_id).get("file")
        path = _safe_dataset_file(
            self.root,
            value,
            f"files.videos.{key}",
            self.metadata_source,
        )
        return path

    def camera_csv(self, camera_id: str | None = None) -> Path:
        if self.take:
            manifest = self.load_manifest()
            files = manifest.get("files")
            value = files.get("camera_frames") if isinstance(files, Mapping) else None
            return _safe_dataset_file(
                self.root,
                value or "data/camera_frames.csv",
                "files.camera_frames",
                self.metadata_source,
            )
        if not camera_id:
            raise DatasetLayoutError("camera_id is required for head frame data")
        logical_id = self._head_camera_logical_id(camera_id)
        return Path(f"{self.head_h5}:/sensor/camera/{logical_id}")

    def imu_csv(self, imu_id: str = "imu0") -> Path:
        if self.take:
            manifest = self.load_manifest()
            files = manifest.get("files")
            value = files.get("imu") if isinstance(files, Mapping) else None
            return _safe_dataset_file(
                self.root,
                value or "data/imu.csv",
                "files.imu",
                self.metadata_source,
            )
        if "/" in imu_id or imu_id in {".", ".."}:
            raise DatasetLayoutError(f"invalid IMU id: {imu_id}")
        return Path(f"{self.head_h5}:/sensor/imu/{imu_id}")

    def gripper_csv(self, imu_id: str = "imu0") -> Path:
        if not self.take:
            raise DatasetLayoutError("head datasets do not contain gripper CSV data")
        manifest = self.load_manifest()
        files = manifest.get("files")
        value = files.get("gripper") if isinstance(files, Mapping) else None
        return _safe_dataset_file(
            self.root,
            value or "data/gripper.csv",
            "files.gripper",
            self.metadata_source,
        )

    def camera_sensor_path(self, camera_id: str) -> Path:
        return self.calibration_path()

    def imu_sensor_path(self, imu_id: str = "imu0") -> Path:
        return self.calibration_path()

    def calibration_path(self) -> Path:
        if self.calibration_override is not None:
            return self.calibration_override
        if not self.take:
            return Path(f"{self.head_h5}:/calib/calibration_full.json")
        manifest = self.load_manifest()
        calibration = manifest.get("calibration")
        value = (
            calibration.get("embedded_file")
            if isinstance(calibration, Mapping)
            else None
        )
        return _safe_dataset_file(
            self.root,
            value or "calibration_full.json",
            "calibration.embedded_file",
            self.metadata_source,
        )

    def validate(
        self,
        cameras: Iterable[str] = (),
        imu_ids: Iterable[str] = (),
        require_calibration: bool = False,
    ) -> None:
        manifest = self.load_manifest()
        if require_calibration:
            if self.calibration_override is None:
                calibration = manifest.get("calibration")
                status = (
                    calibration.get("status")
                    if isinstance(calibration, Mapping)
                    else None
                )
                embedded_file = (
                    calibration.get("embedded_file")
                    if isinstance(calibration, Mapping)
                    else None
                )
                expected_embedded = (
                    "calibration_full.json"
                    if self.take
                    else f"{self.head_h5.name}:/calib/calibration_full.json"
                )
                if (
                    status not in {"present", "copied_unhashed"}
                    or embedded_file != expected_embedded
                ):
                    raise DatasetLayoutError(
                        f"{self.metadata_source}: calibration snapshot is not present"
                    )
            self.load_calibration()
        for camera_id in cameras:
            _require_file(self.camera_video(camera_id), f"{camera_id} MP4")
            if self.take:
                _require_file(self.camera_csv(camera_id), f"{camera_id} frame CSV")
            if sum(1 for _ in self.iter_camera_rows(camera_id)) <= 0:
                raise DatasetLayoutError(
                    f"no frame rows for {camera_id}: {self.camera_csv(camera_id)}"
                )
            if require_calibration:
                self.load_camera_sensor(camera_id)
        for imu_id in imu_ids:
            if self.take:
                _require_file(self.imu_csv(imu_id), f"{imu_id} CSV")
            if sum(1 for _ in self.iter_imu_rows(imu_id)) <= 0:
                raise DatasetLayoutError(
                    f"no IMU rows for {imu_id}: {self.imu_csv(imu_id)}"
                )
            if require_calibration:
                self.load_imu_sensor(imu_id)
        if require_calibration and self.take:
            _require_file(self.calibration_path(), "calibration_full.json")

    def iter_camera_rows(self, camera_id: str) -> Iterator[dict[str, str]]:
        path = self.camera_csv(camera_id)
        if self.take:
            _require_file(path, f"{camera_id} frame CSV")
        previous_frame_index: int | None = None
        previous_timestamp_ns: int | None = None

        def normalize(values: Mapping[str, Any]) -> dict[str, str]:
            nonlocal previous_frame_index, previous_timestamp_ns
            try:
                frame_index = int(str(values["frame_index"]).strip())
                timestamp_ns = int(str(values["timestamp_ns"]).strip())
                pts_ns = int(str(values["pts_ns"]).strip())
                v4l2_sequence = int(str(values["v4l2_sequence"]).strip())
                is_keyframe = int(str(values["is_keyframe"]).strip())
            except (KeyError, TypeError, ValueError) as exc:
                raise DatasetLayoutError(
                    f"invalid {camera_id} frame row in {path}: {values}"
                ) from exc
            if frame_index < 0 or pts_ns < 0 or v4l2_sequence < 0:
                raise DatasetLayoutError(
                    f"{camera_id} frame indices/PTS/sequences must be non-negative: {path}"
                )
            if timestamp_ns <= 0:
                raise DatasetLayoutError(
                    f"{camera_id} timestamp_ns must be positive: {path}"
                )
            if is_keyframe not in (0, 1):
                raise DatasetLayoutError(
                    f"{camera_id} is_keyframe must be 0 or 1: {path}"
                )
            if previous_frame_index is not None and frame_index <= previous_frame_index:
                raise DatasetLayoutError(
                    f"{camera_id} frame_index must be strictly increasing: {path}"
                )
            if previous_timestamp_ns is not None and timestamp_ns <= previous_timestamp_ns:
                raise DatasetLayoutError(
                    f"{camera_id} timestamp_ns must be strictly increasing: {path}"
                )
            previous_frame_index = frame_index
            previous_timestamp_ns = timestamp_ns
            return {
                "camera_id": camera_id,
                "frame_index": str(frame_index),
                "timestamp_ns": str(timestamp_ns),
                "pts_ns": str(pts_ns),
                "v4l2_sequence": str(v4l2_sequence),
                "is_keyframe": str(is_keyframe),
            }

        if not self.take:
            logical_id = self._head_camera_logical_id(camera_id)
            base = f"/sensor/camera/{logical_id}"
            values = {
                "frame_index": _h5_array(self.head_h5, f"{base}/frame_index", np.uint32),
                "timestamp_ns": _h5_array(self.head_h5, f"{base}/timestamp_ns", np.int64),
                "pts_ns": _h5_array(self.head_h5, f"{base}/pts_ns", np.int64),
                "v4l2_sequence": _h5_array(self.head_h5, f"{base}/v4l2_sequence", np.uint64),
                "is_keyframe": _h5_array(self.head_h5, f"{base}/is_keyframe", np.uint8),
            }
            lengths = {len(items) for items in values.values()}
            if len(lengths) != 1:
                raise DatasetLayoutError(f"head camera arrays have different lengths: {path}")
            for index in range(next(iter(lengths), 0)):
                yield normalize({key: int(items[index]) for key, items in values.items()})
            return

        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            _require_fields(reader.fieldnames, CAMERA_FIELDS, path)
            for row in reader:
                if row.get("camera_id", "").strip() != camera_id:
                    continue
                yield normalize(row)

    def iter_imu_rows(self, imu_id: str = "imu0") -> Iterator[dict[str, str]]:
        path = self.imu_csv(imu_id)
        if self.take:
            _require_file(path, f"{imu_id} CSV")
        previous_timestamp_ns: int | None = None

        def normalize(values: Mapping[str, Any]) -> dict[str, str]:
            nonlocal previous_timestamp_ns
            try:
                timestamp_ns = int(str(values["timestamp_ns"]).strip())
                normalized_values = [
                    _finite_number(values[field], f"{imu_id}.{field}")
                    for field in IMU_FIELDS[2:]
                ]
            except (KeyError, TypeError, ValueError) as exc:
                raise DatasetLayoutError(
                    f"invalid {imu_id} row in {path}: {values}"
                ) from exc
            if timestamp_ns <= 0:
                raise DatasetLayoutError(
                    f"{imu_id} timestamp_ns must be positive: {path}"
                )
            if previous_timestamp_ns is not None and timestamp_ns <= previous_timestamp_ns:
                raise DatasetLayoutError(
                    f"{imu_id} timestamp_ns must be strictly increasing: {path}"
                )
            previous_timestamp_ns = timestamp_ns
            return {
                "timestamp_ns": str(timestamp_ns),
                "imu_id": imu_id,
                **{
                    field: repr(value)
                    for field, value in zip(IMU_FIELDS[2:], normalized_values)
                },
            }

        if not self.take:
            base = f"/sensor/imu/{imu_id}"
            timestamps = _h5_array(self.head_h5, f"{base}/timestamp_ns", np.int64)
            gyroscope = _h5_array(self.head_h5, f"{base}/gyro_rad_s", np.float64)
            accelerometer = _h5_array(self.head_h5, f"{base}/accel_m_s2", np.float64)
            if len(gyroscope) != len(timestamps) * 3 or len(accelerometer) != len(timestamps) * 3:
                raise DatasetLayoutError(f"head IMU arrays have incompatible shapes: {path}")
            gyroscope = gyroscope.reshape(-1, 3)
            accelerometer = accelerometer.reshape(-1, 3)
            for index, timestamp_ns in enumerate(timestamps):
                yield normalize(
                    {
                        "timestamp_ns": int(timestamp_ns),
                        "gx_rad_s": float(gyroscope[index, 0]),
                        "gy_rad_s": float(gyroscope[index, 1]),
                        "gz_rad_s": float(gyroscope[index, 2]),
                        "ax_m_s2": float(accelerometer[index, 0]),
                        "ay_m_s2": float(accelerometer[index, 1]),
                        "az_m_s2": float(accelerometer[index, 2]),
                    }
                )
            return

        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            _require_fields(reader.fieldnames, IMU_FIELDS, path)
            for row in reader:
                if row.get("imu_id", "").strip() != imu_id:
                    continue
                yield normalize(row)

    def load_calibration(self) -> dict[str, Any]:
        path = self.calibration_path()
        if self.calibration_override is not None or self.take:
            path = _require_file(path, "calibration_full.json")
            calibration = _load_json_object(path, "calibration_full.json")
        else:
            calibration = _h5_json_object(
                self.head_h5,
                "/calib/calibration_full.json",
                "embedded calibration_full.json",
            )
        if calibration.get("schema") != CALIBRATION_SCHEMA:
            raise DatasetLayoutError(
                f"{path}: schema must be {CALIBRATION_SCHEMA!r}"
            )
        version = calibration.get("schema_version")
        if type(version) is not int or version != CALIBRATION_SCHEMA_VERSION:
            raise DatasetLayoutError(
                f"{path}: schema_version must be {CALIBRATION_SCHEMA_VERSION}"
            )
        convention = calibration.get("transform_convention")
        expected_convention = {
            "equation": "p_to = T_to_from * p_from",
            "matrix_layout": "row_major",
            "translation_unit": "m",
            "angle_unit": "rad",
            "vector_convention": "homogeneous_column",
        }
        if not isinstance(convention, Mapping):
            raise DatasetLayoutError(f"{path}: transform_convention is missing")
        for key, expected in expected_convention.items():
            if convention.get(key) != expected:
                raise DatasetLayoutError(
                    f"{path}: transform_convention.{key} must be {expected!r}"
                )
        return calibration

    def load_camera_sensor(self, camera_id: str) -> dict[str, Any]:
        calibration = self.load_calibration()
        bindings = calibration.get("sensor_bindings")
        cameras = calibration.get("cameras")
        canonical = calibration.get("canonical_camera_transforms")
        if not isinstance(bindings, Mapping) or not isinstance(cameras, Mapping):
            raise DatasetLayoutError(
                f"{self.calibration_path()}: missing sensor_bindings/cameras"
            )
        binding = bindings.get(camera_id)
        if not isinstance(binding, Mapping):
            raise DatasetLayoutError(
                f"{self.calibration_path()}: missing sensor_bindings.{camera_id}"
            )
        if binding.get("runtime_id") != camera_id:
            raise DatasetLayoutError(
                f"sensor_bindings.{camera_id}.runtime_id must be {camera_id!r}"
            )
        frame_id = binding.get("frame_id")
        if not isinstance(frame_id, str) or not frame_id:
            raise DatasetLayoutError(
                f"sensor_bindings.{camera_id}.frame_id must be non-empty"
            )
        legacy_key = binding.get("legacy_calibration_key")
        source = cameras.get(legacy_key) if isinstance(legacy_key, str) else None
        if not isinstance(source, Mapping):
            raise DatasetLayoutError(
                f"{self.calibration_path()}: invalid calibration binding for {camera_id}"
            )
        transforms = (
            canonical.get("transforms") if isinstance(canonical, Mapping) else None
        )
        transform = transforms.get(camera_id) if isinstance(transforms, Mapping) else None
        if not isinstance(transform, Mapping):
            raise DatasetLayoutError(
                f"{self.calibration_path()}: missing canonical transform for {camera_id}"
            )
        rig = calibration.get("rig")
        body_frame = rig.get("body_frame") if isinstance(rig, Mapping) else None
        if not isinstance(body_frame, str) or not body_frame:
            raise DatasetLayoutError(f"{self.calibration_path()}: rig.body_frame is missing")
        if transform.get("from_frame") != body_frame:
            raise DatasetLayoutError(
                f"canonical transform {camera_id}.from_frame must equal rig.body_frame"
            )
        if transform.get("to_frame") != frame_id:
            raise DatasetLayoutError(
                f"canonical transform {camera_id}.to_frame must equal its binding frame_id"
            )
        t_bs = _rigid_transform(
            transform.get("matrix"),
            f"canonical_camera_transforms.transforms.{camera_id}.matrix",
        )
        if frame_id == body_frame and not np.allclose(
            t_bs, np.eye(4), rtol=0.0, atol=1e-9
        ):
            raise DatasetLayoutError(
                f"canonical transform for body-frame camera {camera_id} must be identity"
            )

        sensor = copy.deepcopy(dict(source))
        sensor["T_BS"] = {
            "rows": 4,
            "cols": 4,
            "data": [float(value) for value in t_bs.reshape(-1)],
        }
        runtime_resolution = binding.get("runtime_resolution")
        if not isinstance(runtime_resolution, list) or len(runtime_resolution) != 2:
            raise DatasetLayoutError(
                f"sensor_bindings.{camera_id}.runtime_resolution must be a vec2"
            )
        width = _positive_dimension(
            runtime_resolution[0], f"sensor_bindings.{camera_id}.runtime_resolution[0]"
        )
        height = _positive_dimension(
            runtime_resolution[1], f"sensor_bindings.{camera_id}.runtime_resolution[1]"
        )
        source_resolution = sensor.get("resolution")
        intrinsics = sensor.get("intrinsics")
        normalized = sensor.get("intrinsics_normalized")
        if isinstance(normalized, list) and len(normalized) >= 4:
            sensor["intrinsics"] = [
                _positive_number(normalized[0], "normalized fx") * width,
                _positive_number(normalized[1], "normalized fy") * height,
                _finite_number(normalized[2], "normalized cx") * width,
                _finite_number(normalized[3], "normalized cy") * height,
            ]
        elif (
            isinstance(source_resolution, list)
            and len(source_resolution) == 2
            and isinstance(intrinsics, list)
            and len(intrinsics) >= 4
        ):
            source_width = _positive_number(source_resolution[0], "camera width")
            source_height = _positive_number(source_resolution[1], "camera height")
            sensor["intrinsics"] = [
                _positive_number(intrinsics[0], "camera fx") * width / source_width,
                _positive_number(intrinsics[1], "camera fy") * height / source_height,
                _finite_number(intrinsics[2], "camera cx") * width / source_width,
                _finite_number(intrinsics[3], "camera cy") * height / source_height,
            ]
        else:
            raise DatasetLayoutError(
                f"{self.calibration_path()}: cannot scale intrinsics for {camera_id}"
            )
        sensor["resolution"] = [width, height]
        sensor["comment"] = (
            f"{sensor.get('comment', camera_id)}; {self.format_name} runtime binding {camera_id}"
        )
        return sensor

    def load_imu_sensor(self, imu_id: str = "imu0") -> dict[str, Any]:
        calibration = self.load_calibration()
        imus = calibration.get("imus")
        imu = imus.get(imu_id) if isinstance(imus, Mapping) else None
        if not isinstance(imu, Mapping):
            raise DatasetLayoutError(
                f"{self.calibration_path()}: missing imus.{imu_id}"
            )
        if imu.get("runtime_id") != imu_id:
            raise DatasetLayoutError(f"imus.{imu_id}.runtime_id must be {imu_id!r}")
        extrinsic = imu.get("extrinsic")
        if not isinstance(extrinsic, Mapping):
            raise DatasetLayoutError(
                f"{self.calibration_path()}: missing imus.{imu_id}.extrinsic"
            )
        t_imu_from_source = _rigid_transform(
            extrinsic.get("matrix"), f"imus.{imu_id}.extrinsic.matrix"
        )
        rig = calibration.get("rig")
        body_frame = rig.get("body_frame") if isinstance(rig, Mapping) else None
        from_frame = extrinsic.get("from_frame")
        if not isinstance(body_frame, str) or not body_frame:
            raise DatasetLayoutError(f"{self.calibration_path()}: rig.body_frame is missing")
        legacy_profile_t_bs = extrinsic.get("legacy_field") == "T_BS"
        if legacy_profile_t_bs:
            # Bundled legacy IMU profiles keep the historical T_BS consumed by
            # the EuRoC pipeline: it already maps cam0 (the SLAM left camera)
            # into the IMU frame. Some migrated calibration_full files retain
            # a stale cam1_optical from_frame label; composing that label with
            # the canonical cam1 transform rotates/translates T_BS a second
            # time and makes inertial optimization diverge.
            t_imu_from_body = t_imu_from_source
        elif from_frame == body_frame:
            t_imu_from_body = t_imu_from_source
        else:
            bindings = calibration.get("sensor_bindings")
            source_camera_id = None
            if isinstance(bindings, Mapping):
                for candidate_id, binding in bindings.items():
                    if isinstance(binding, Mapping) and binding.get("frame_id") == from_frame:
                        source_camera_id = str(candidate_id)
                        break
            canonical = calibration.get("canonical_camera_transforms")
            transforms = (
                canonical.get("transforms") if isinstance(canonical, Mapping) else None
            )
            source_transform = (
                transforms.get(source_camera_id)
                if isinstance(transforms, Mapping) and source_camera_id
                else None
            )
            if not isinstance(source_transform, Mapping):
                raise DatasetLayoutError(
                    f"imus.{imu_id}.extrinsic.from_frame {from_frame!r} cannot be "
                    f"resolved from rig body frame {body_frame!r}"
                )
            if source_transform.get("from_frame") != body_frame or source_transform.get(
                "to_frame"
            ) != from_frame:
                raise DatasetLayoutError(
                    f"canonical transform for {source_camera_id} has inconsistent frames"
                )
            t_source_from_body = _rigid_transform(
                source_transform.get("matrix"),
                f"canonical_camera_transforms.transforms.{source_camera_id}.matrix",
            )
            t_imu_from_body = t_imu_from_source @ t_source_from_body

        measurement_frame = imu.get("measurement_frame_id")
        if not isinstance(measurement_frame, str) or not measurement_frame:
            raise DatasetLayoutError(
                f"imus.{imu_id}.measurement_frame_id must be non-empty"
            )
        if extrinsic.get("to_frame") != measurement_frame:
            raise DatasetLayoutError(
                f"imus.{imu_id}.extrinsic.to_frame must equal measurement_frame_id"
            )

        noise = imu.get("noise_model")
        if not isinstance(noise, Mapping):
            raise DatasetLayoutError(
                f"{self.calibration_path()}: missing imus.{imu_id}.noise_model"
            )
        sensor = {
            "sensor_type": "imu",
            "comment": f"{imu_id} from {self.format_name} calibration_full.json",
            "T_BS": {
                "rows": 4,
                "cols": 4,
                "data": [float(value) for value in t_imu_from_body.reshape(-1)],
            },
            "rate_hz": _positive_number(imu.get("rate_hz"), f"imus.{imu_id}.rate_hz"),
            "gyroscope_noise_density": _positive_number(
                noise.get("gyroscope_noise_density"),
                f"imus.{imu_id}.noise_model.gyroscope_noise_density",
            ),
            "gyroscope_random_walk": _positive_number(
                noise.get("gyroscope_random_walk"),
                f"imus.{imu_id}.noise_model.gyroscope_random_walk",
            ),
            "accelerometer_noise_density": _positive_number(
                noise.get("accelerometer_noise_density"),
                f"imus.{imu_id}.noise_model.accelerometer_noise_density",
            ),
            "accelerometer_random_walk": _positive_number(
                noise.get("accelerometer_random_walk"),
                f"imus.{imu_id}.noise_model.accelerometer_random_walk",
            ),
            "runtime_axis_mapping_applied": True,
            "runtime_gyro_bias_corrected": _runtime_gyro_bias_corrected(imu),
        }
        if not legacy_profile_t_bs:
            # Native v2 transforms describe the recorder's runtime output
            # frame directly, so the raw sensor axis mapping must not be
            # repeated. Legacy T_BS profiles are different: both the old and
            # compact recorders already applied the same low-level axis map,
            # while ORB-SLAM still needs the profile-specific headband/M900
            # input alignment. Leave those rotations unspecified so the
            # settings merger restores the established legacy calibration.
            sensor["input_gyro_rotation"] = np.eye(3).reshape(-1).tolist()
            sensor["input_accel_rotation"] = np.eye(3).reshape(-1).tolist()
            sensor["input_time_offset_s"] = 0.0
        temporal = calibration.get("temporal_calibration")
        if not isinstance(temporal, Mapping):
            raise DatasetLayoutError(
                f"{self.calibration_path()}: temporal_calibration is missing"
            )
        if temporal.get("timestamp_domain") != "CLOCK_MONOTONIC":
            raise DatasetLayoutError(
                "temporal_calibration.timestamp_domain must be CLOCK_MONOTONIC"
            )
        temporal_status = temporal.get("status")
        if temporal_status == "valid":
            offset_ns = temporal.get("camera_imu_offset_ns")
            if isinstance(offset_ns, bool):
                raise DatasetLayoutError(
                    "temporal_calibration.camera_imu_offset_ns must be numeric"
                )
            try:
                offset_s = float(offset_ns) * 1e-9
            except (TypeError, ValueError, OverflowError) as exc:
                raise DatasetLayoutError(
                    "temporal_calibration.camera_imu_offset_ns must be numeric"
                ) from exc
            if not math.isfinite(offset_s) or abs(offset_s) > 0.2:
                raise DatasetLayoutError(
                    "temporal calibration offset must be finite and within +/-0.2 seconds"
                )
            sensor["input_time_offset_s"] = offset_s
        elif temporal_status == "not_calibrated":
            if temporal.get("camera_imu_offset_ns") is not None:
                raise DatasetLayoutError(
                    "not_calibrated temporal calibration must have a null offset"
                )
        else:
            raise DatasetLayoutError(
                "temporal_calibration.status must be 'valid' or 'not_calibrated'"
            )
        return sensor


def _write_camera_csv(path: Path, rows: Iterable[Mapping[str, str]]) -> int:
    count = 0
    with path.open("w", encoding="utf-8", newline="") as stream:
        stream.write(
            "#frame_index,timestamp_ns,pts_ns,v4l2_sequence,is_keyframe\n"
        )
        writer = csv.writer(stream, lineterminator="\n")
        for row in rows:
            writer.writerow([row[field] for field in CAMERA_FIELDS[1:]])
            count += 1
    return count


def _write_imu_csv(path: Path, rows: Iterable[Mapping[str, str]]) -> int:
    count = 0
    with path.open("w", encoding="utf-8", newline="") as stream:
        stream.write(
            "#timestamp_ns,gx_rad_s,gy_rad_s,gz_rad_s,"
            "ax_m_s2,ay_m_s2,az_m_s2\n"
        )
        writer = csv.writer(stream, lineterminator="\n")
        for row in rows:
            writer.writerow(
                [row["timestamp_ns"]] + [row[field] for field in IMU_FIELDS[2:]]
            )
            count += 1
    return count


def _replace_symlink(path: Path, target: Path) -> None:
    if os.path.lexists(path):
        if path.is_dir() and not path.is_symlink():
            raise DatasetLayoutError(f"adapter path is unexpectedly a directory: {path}")
        path.unlink()
    if os.name == "nt":
        # Hard links need no developer mode/admin rights; across volumes copy.
        try:
            os.link(target.resolve(), path)
        except OSError:
            shutil.copy2(target.resolve(), path)
    else:
        path.symlink_to(target.resolve())


def _replace_adapter_directory(output: Path, marker_name: str) -> Path:
    if output.exists():
        marker = output / marker_name
        if not marker.is_file():
            raise DatasetLayoutError(
                f"refusing to replace non-adapter directory: {output}"
            )
        shutil.rmtree(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    return Path(tempfile.mkdtemp(prefix=f".{output.name}.tmp-", dir=str(output.parent)))


def prepare_euroc_view(
    dataset: str | os.PathLike[str],
    output_dir: str | os.PathLike[str],
    *,
    take: bool = False,
    calibration_file: str | os.PathLike[str] | None = None,
    cameras: Sequence[str] = ("cam0", "cam1"),
    imu_ids: Sequence[str] = ("imu0",),
    include_calibration: bool = True,
) -> Path:
    """Materialize a private EuRoC-shaped view for legacy internal tools."""
    layout = DatasetLayout.open(
        dataset,
        take=take,
        calibration_file=calibration_file,
    )
    export_calibration = bool(include_calibration or imu_ids)
    layout.validate(
        cameras=cameras,
        imu_ids=imu_ids,
        require_calibration=export_calibration,
    )
    output = Path(output_dir).expanduser().resolve()
    staging = _replace_adapter_directory(output, ADAPTER_MARKER)
    try:
        mav0 = staging / "mav0"
        camera_counts: dict[str, int] = {}
        for camera_id in cameras:
            camera_dir = mav0 / camera_id
            data_dir = camera_dir / "data"
            data_dir.mkdir(parents=True, exist_ok=True)
            _replace_symlink(
                data_dir / f"{camera_id}.mp4", layout.camera_video(camera_id)
            )
            camera_counts[camera_id] = _write_camera_csv(
                camera_dir / "data.csv", layout.iter_camera_rows(camera_id)
            )
            if camera_counts[camera_id] <= 0:
                raise DatasetLayoutError(f"no frame rows for {camera_id}")
            if export_calibration:
                (camera_dir / "sensor.yaml").write_text(
                    yaml.safe_dump(
                        layout.load_camera_sensor(camera_id),
                        sort_keys=False,
                        allow_unicode=True,
                    ),
                    encoding="utf-8",
                )

        imu_counts: dict[str, int] = {}
        for imu_id in imu_ids:
            imu_dir = mav0 / imu_id
            imu_dir.mkdir(parents=True, exist_ok=True)
            imu_counts[imu_id] = _write_imu_csv(
                imu_dir / "data.csv", layout.iter_imu_rows(imu_id)
            )
            if imu_counts[imu_id] <= 0:
                raise DatasetLayoutError(f"no IMU rows for {imu_id}")
            (imu_dir / "sensor.yaml").write_text(
                yaml.safe_dump(
                    layout.load_imu_sensor(imu_id),
                    sort_keys=False,
                    allow_unicode=True,
                ),
                encoding="utf-8",
            )
        marker = {
            "schema": "orbslam3.euroc_adapter",
            "schema_version": 1,
            "source_dataset": str(layout.root),
            "source_format": layout.format_name,
            "calibration_source": str(layout.calibration_path()),
            "calibration_exported": export_calibration,
            "cameras": camera_counts,
            "imus": imu_counts,
        }
        (staging / ADAPTER_MARKER).write_text(
            json.dumps(marker, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(staging, output)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return output / "mav0"


def prepare_take_view(
    dataset: str | os.PathLike[str],
    output_dir: str | os.PathLike[str],
    *,
    take: bool = False,
    calibration_file: str | os.PathLike[str] | None = None,
    cameras: Sequence[str] = ("cam0", "cam1", "cam2", "cam3"),
    imu_ids: Sequence[str] = ("imu0",),
) -> Path:
    """Return compact-v2 input, adapting a head dataset when necessary."""

    layout = DatasetLayout.open(
        dataset,
        take=take,
        calibration_file=calibration_file,
    )
    layout.validate(
        cameras=cameras,
        imu_ids=imu_ids,
        require_calibration=True,
    )
    if take:
        return layout.root

    output = Path(output_dir).expanduser().resolve()
    staging = _replace_adapter_directory(output, TAKE_ADAPTER_MARKER)
    try:
        video_dir = staging / "video"
        data_dir = staging / "data"
        video_dir.mkdir(parents=True, exist_ok=True)
        data_dir.mkdir(parents=True, exist_ok=True)

        videos: dict[str, str] = {}
        camera_counts: dict[str, int] = {}
        camera_csv = data_dir / "camera_frames.csv"
        with camera_csv.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CAMERA_FIELDS, lineterminator="\n")
            writer.writeheader()
            for camera_id in cameras:
                target = video_dir / f"{camera_id}.mp4"
                _replace_symlink(target, layout.camera_video(camera_id))
                videos[camera_id] = f"video/{camera_id}.mp4"
                count = 0
                for row in layout.iter_camera_rows(camera_id):
                    writer.writerow(row)
                    count += 1
                if count <= 0:
                    raise DatasetLayoutError(f"no frame rows for {camera_id}")
                camera_counts[camera_id] = count

        imu_counts: dict[str, int] = {}
        imu_csv = data_dir / "imu.csv"
        with imu_csv.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=IMU_FIELDS, lineterminator="\n")
            writer.writeheader()
            for imu_id in imu_ids:
                count = 0
                for row in layout.iter_imu_rows(imu_id):
                    writer.writerow(row)
                    count += 1
                if count <= 0:
                    raise DatasetLayoutError(f"no IMU rows for {imu_id}")
                imu_counts[imu_id] = count

        calibration = layout.load_calibration()
        (staging / "calibration_full.json").write_text(
            json.dumps(calibration, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        manifest = copy.deepcopy(layout.load_manifest())
        manifest["schema_version"] = COMPACT_SCHEMA_VERSION
        manifest["files"] = {
            "camera_frames": "data/camera_frames.csv",
            "imu": "data/imu.csv",
            "gripper": None,
            "videos": videos,
        }
        manifest["calibration"] = {
            **(
                dict(manifest.get("calibration"))
                if isinstance(manifest.get("calibration"), Mapping)
                else {}
            ),
            "status": "present",
            "embedded_file": "calibration_full.json",
        }
        (staging / "dataset.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        marker = {
            "schema": "orbslam3.take_adapter",
            "schema_version": 1,
            "source_dataset": str(layout.root),
            "source_format": layout.format_name,
            "source_hdf5": str(layout.head_h5),
            "calibration_source": str(layout.calibration_path()),
            "cameras": camera_counts,
            "imus": imu_counts,
        }
        (staging / TAKE_ADAPTER_MARKER).write_text(
            json.dumps(marker, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(staging, output)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return output


def _read_internal_csv(path: Path) -> Iterator[dict[str, str]]:
    """Read a generated mav0 CSV used only between pipeline stages."""

    _require_file(path, "generated compatibility CSV")
    header: str | None = None
    data_lines: list[str] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            if header is None:
                header = line[1:].strip()
            continue
        if header is None:
            header = line
            continue
        data_lines.append(line)
    if not header:
        raise DatasetLayoutError(f"CSV header not found: {path}")
    yield from csv.DictReader([header, *data_lines])


def _sensor_matrix(sensor: Mapping[str, Any], label: str) -> list[float]:
    t_bs = sensor.get("T_BS")
    values = t_bs.get("data") if isinstance(t_bs, Mapping) else None
    matrix = _rigid_transform(values, f"{label}.T_BS")
    return [float(value) for value in matrix.reshape(-1)]


def _calibration_from_internal_view(
    mav0: Path,
    cameras: Sequence[str],
    imu_ids: Sequence[str],
) -> dict[str, Any]:
    if not cameras:
        raise DatasetLayoutError("at least one camera is required")
    body_camera = cameras[0]
    body_frame = f"{body_camera}_optical"
    bindings: dict[str, Any] = {}
    transforms: dict[str, Any] = {}
    camera_calibrations: dict[str, Any] = {}
    for camera_id in cameras:
        sensor_path = mav0 / camera_id / "sensor.yaml"
        sensor = yaml.safe_load(_require_file(sensor_path, f"{camera_id} sensor.yaml").read_text(encoding="utf-8"))
        if not isinstance(sensor, Mapping):
            raise DatasetLayoutError(f"invalid camera sensor YAML: {sensor_path}")
        resolution = sensor.get("resolution")
        if not isinstance(resolution, list) or len(resolution) != 2:
            raise DatasetLayoutError(f"{sensor_path}: resolution must be a vec2")
        frame_id = f"{camera_id}_optical"
        camera_calibrations[camera_id] = copy.deepcopy(dict(sensor))
        bindings[camera_id] = {
            "runtime_id": camera_id,
            "frame_id": frame_id,
            "legacy_calibration_key": camera_id,
            "runtime_resolution": [int(resolution[0]), int(resolution[1])],
        }
        transforms[camera_id] = {
            "from_frame": body_frame,
            "to_frame": frame_id,
            "matrix": _sensor_matrix(sensor, camera_id),
        }

    imus: dict[str, Any] = {}
    temporal_offset_s: float | None = None
    for imu_id in imu_ids:
        sensor_path = mav0 / imu_id / "sensor.yaml"
        sensor = yaml.safe_load(_require_file(sensor_path, f"{imu_id} sensor.yaml").read_text(encoding="utf-8"))
        if not isinstance(sensor, Mapping):
            raise DatasetLayoutError(f"invalid IMU sensor YAML: {sensor_path}")
        measurement_frame = f"{imu_id}_output"
        imus[imu_id] = {
            "runtime_id": imu_id,
            "rate_hz": sensor.get("rate_hz"),
            "measurement_frame_id": measurement_frame,
            "extrinsic": {
                "from_frame": body_frame,
                "to_frame": measurement_frame,
                "matrix": _sensor_matrix(sensor, imu_id),
            },
            "noise_model": {
                "gyroscope_noise_density": sensor.get("gyroscope_noise_density"),
                "gyroscope_random_walk": sensor.get("gyroscope_random_walk"),
                "accelerometer_noise_density": sensor.get("accelerometer_noise_density"),
                "accelerometer_random_walk": sensor.get("accelerometer_random_walk"),
            },
            "six_face": {
                "status": "missing",
                "reason": "generated intermediate view",
                "result": None,
            },
        }
        value = sensor.get("input_time_offset_s")
        if value is not None:
            temporal_offset_s = _finite_number(value, f"{imu_id}.input_time_offset_s")

    temporal = {
        "status": "not_calibrated",
        "camera_imu_offset_ns": None,
        "timestamp_domain": "CLOCK_MONOTONIC",
    }
    if temporal_offset_s is not None:
        temporal = {
            "status": "valid",
            "camera_imu_offset_ns": int(round(temporal_offset_s * 1e9)),
            "timestamp_domain": "CLOCK_MONOTONIC",
        }
    return {
        "schema": CALIBRATION_SCHEMA,
        "schema_version": CALIBRATION_SCHEMA_VERSION,
        "transform_convention": {
            "equation": "p_to = T_to_from * p_from",
            "matrix_layout": "row_major",
            "translation_unit": "m",
            "angle_unit": "rad",
            "vector_convention": "homogeneous_column",
        },
        "rig": {"body_frame": body_frame},
        "sensor_bindings": bindings,
        "canonical_camera_transforms": {"transforms": transforms},
        "cameras": camera_calibrations,
        "imus": imus,
        "temporal_calibration": temporal,
    }


def prepare_take_from_internal_view(
    mav0_dir: str | os.PathLike[str],
    output_dir: str | os.PathLike[str],
    *,
    cameras: Sequence[str] = ("cam0", "cam1"),
    imu_ids: Sequence[str] = (),
) -> Path:
    """Convert a pipeline-generated mav0 view into a private compact take."""

    mav0 = Path(mav0_dir).expanduser().resolve()
    if mav0.name != "mav0" and (mav0 / "mav0").is_dir():
        mav0 = (mav0 / "mav0").resolve()
    if not mav0.is_dir():
        raise DatasetLayoutError(f"generated mav0 directory not found: {mav0}")
    output = Path(output_dir).expanduser().resolve()
    staging = _replace_adapter_directory(output, TAKE_ADAPTER_MARKER)
    try:
        video_dir = staging / "video"
        data_dir = staging / "data"
        video_dir.mkdir(parents=True, exist_ok=True)
        data_dir.mkdir(parents=True, exist_ok=True)

        videos: dict[str, str] = {}
        camera_counts: dict[str, int] = {}
        with (data_dir / "camera_frames.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=CAMERA_FIELDS, lineterminator="\n")
            writer.writeheader()
            for camera_id in cameras:
                _replace_symlink(
                    video_dir / f"{camera_id}.mp4",
                    _require_file(
                        mav0 / camera_id / "data" / f"{camera_id}.mp4",
                        f"{camera_id} generated MP4",
                    ),
                )
                videos[camera_id] = f"video/{camera_id}.mp4"
                count = 0
                for row in _read_internal_csv(mav0 / camera_id / "data.csv"):
                    frame_index = row.get("frame_index", row.get("video_frame_index"))
                    timestamp_ns = row.get("timestamp_ns", row.get("timestamp [ns]"))
                    if frame_index is None or timestamp_ns is None:
                        raise DatasetLayoutError(
                            f"generated camera CSV lacks frame/timestamp columns: {mav0 / camera_id / 'data.csv'}"
                        )
                    writer.writerow(
                        {
                            "camera_id": camera_id,
                            "frame_index": frame_index,
                            "timestamp_ns": timestamp_ns,
                            "pts_ns": row.get("pts_ns", "0"),
                            "v4l2_sequence": row.get("v4l2_sequence", frame_index),
                            "is_keyframe": row.get("is_keyframe", "0"),
                        }
                    )
                    count += 1
                if count <= 0:
                    raise DatasetLayoutError(f"no generated rows for {camera_id}")
                camera_counts[camera_id] = count

        imu_counts: dict[str, int] = {}
        imu_path: str | None = None
        if imu_ids:
            imu_path = "data/imu.csv"
            with (data_dir / "imu.csv").open(
                "w", encoding="utf-8", newline=""
            ) as stream:
                writer = csv.DictWriter(stream, fieldnames=IMU_FIELDS, lineterminator="\n")
                writer.writeheader()
                for imu_id in imu_ids:
                    count = 0
                    for row in _read_internal_csv(mav0 / imu_id / "data.csv"):
                        writer.writerow(
                            {
                                "timestamp_ns": row.get("timestamp_ns", row.get("timestamp [ns]")),
                                "imu_id": imu_id,
                                "gx_rad_s": row.get("gx_rad_s", row.get("gx")),
                                "gy_rad_s": row.get("gy_rad_s", row.get("gy")),
                                "gz_rad_s": row.get("gz_rad_s", row.get("gz")),
                                "ax_m_s2": row.get("ax_m_s2", row.get("ax")),
                                "ay_m_s2": row.get("ay_m_s2", row.get("ay")),
                                "az_m_s2": row.get("az_m_s2", row.get("az")),
                            }
                        )
                        count += 1
                    if count <= 0:
                        raise DatasetLayoutError(f"no generated rows for {imu_id}")
                    imu_counts[imu_id] = count

        calibration = _calibration_from_internal_view(mav0, cameras, imu_ids)
        (staging / "calibration_full.json").write_text(
            json.dumps(calibration, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        manifest = {
            "schema": COMPACT_SCHEMA,
            "schema_version": COMPACT_SCHEMA_VERSION,
            "dataset_id": output.name,
            "complete": True,
            "stop_reason": "generated_intermediate",
            "clock": {"domain": "CLOCK_MONOTONIC"},
            "files": {
                "camera_frames": "data/camera_frames.csv",
                "imu": imu_path,
                "gripper": None,
                "videos": videos,
            },
            "calibration": {
                "status": "present",
                "embedded_file": "calibration_full.json",
            },
        }
        (staging / "dataset.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        marker = {
            "schema": "orbslam3.derived_take_adapter",
            "schema_version": 1,
            "source_mav0": str(mav0),
            "cameras": camera_counts,
            "imus": imu_counts,
        }
        (staging / TAKE_ADAPTER_MARKER).write_text(
            json.dumps(marker, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(staging, output)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return output


def _add_layout_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument(
        "--take",
        action="store_true",
        help="Read an optional compact-v2 take instead of the default head dataset",
    )
    parser.add_argument(
        "--calibration-file",
        type=Path,
        help="Override embedded calibration_full.json without modifying the dataset",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate = subparsers.add_parser("validate", help="Validate selected inputs")
    _add_layout_arguments(validate)
    validate.add_argument("--camera", action="append", default=[])
    validate.add_argument("--imu", action="append", default=[])
    validate.add_argument("--require-calibration", action="store_true")

    prepare = subparsers.add_parser(
        "prepare-euroc", help="Materialize an internal EuRoC compatibility view"
    )
    _add_layout_arguments(prepare)
    prepare.add_argument("--output", required=True, type=Path)
    prepare.add_argument("--camera", action="append", default=[])
    prepare.add_argument("--imu", action="append", default=[])
    prepare.add_argument(
        "--without-calibration",
        action="store_true",
        help="Create a frame-only view without requiring camera calibration",
    )

    prepare_take = subparsers.add_parser(
        "prepare-take", help="Materialize a compact-v2 compatibility view for head input"
    )
    _add_layout_arguments(prepare_take)
    prepare_take.add_argument("--output", required=True, type=Path)
    prepare_take.add_argument("--camera", action="append", default=[])
    prepare_take.add_argument("--imu", action="append", default=[])
    prepare_take.add_argument("--without-imu", action="store_true")

    derived_take = subparsers.add_parser(
        "prepare-derived-take",
        help="Convert a pipeline-generated compatibility view to compact-v2",
    )
    derived_take.add_argument("--mav0", required=True, type=Path)
    derived_take.add_argument("--output", required=True, type=Path)
    derived_take.add_argument("--camera", action="append", default=[])
    derived_take.add_argument("--imu", action="append", default=[])

    count = subparsers.add_parser("count-camera", help="Count one camera's rows")
    _add_layout_arguments(count)
    count.add_argument("--camera", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.command == "validate":
            layout = DatasetLayout.open(
                args.dataset,
                take=args.take,
                calibration_file=args.calibration_file,
            )
            layout.validate(
                cameras=args.camera,
                imu_ids=args.imu,
                require_calibration=args.require_calibration,
            )
            print(f"{layout.format_name}: {layout.root}")
            return 0
        if args.command == "prepare-euroc":
            cameras = tuple(args.camera or ("cam0", "cam1"))
            imu_ids = tuple(args.imu)
            if args.without_calibration and imu_ids:
                raise DatasetLayoutError(
                    "--without-calibration cannot be combined with --imu"
                )
            print(
                prepare_euroc_view(
                    args.dataset,
                    args.output,
                    take=args.take,
                    calibration_file=args.calibration_file,
                    cameras=cameras,
                    imu_ids=imu_ids,
                    include_calibration=not args.without_calibration,
                )
            )
            return 0
        if args.command == "prepare-take":
            if args.without_imu and args.imu:
                raise DatasetLayoutError("--without-imu cannot be combined with --imu")
            print(
                prepare_take_view(
                    args.dataset,
                    args.output,
                    take=args.take,
                    calibration_file=args.calibration_file,
                    cameras=tuple(args.camera or ("cam0", "cam1", "cam2", "cam3")),
                    imu_ids=(
                        ()
                        if args.without_imu
                        else tuple(args.imu or ("imu0",))
                    ),
                )
            )
            return 0
        if args.command == "prepare-derived-take":
            print(
                prepare_take_from_internal_view(
                    args.mav0,
                    args.output,
                    cameras=tuple(args.camera or ("cam0", "cam1")),
                    imu_ids=tuple(args.imu),
                )
            )
            return 0
        if args.command == "count-camera":
            layout = DatasetLayout.open(
                args.dataset,
                take=args.take,
                calibration_file=args.calibration_file,
            )
            print(sum(1 for _ in layout.iter_camera_rows(args.camera)))
            return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
