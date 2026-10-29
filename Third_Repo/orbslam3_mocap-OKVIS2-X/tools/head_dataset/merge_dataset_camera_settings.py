#!/usr/bin/env python3
"""
Merge dataset camera calibration into an ORB-SLAM3 settings file.

Head-v3 HDF5 is the default input contract. Camera/IMU metadata is resolved
from its embedded calibration; ``--take`` selects compact-v2 input.
"""

import argparse
import os
import re
import sys
from pathlib import Path

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from dataset_layout import DatasetLayout


MANAGED_KEYS = {
    "Camera.type",
    "Camera.width",
    "Camera.height",
    "Camera.newWidth",
    "Camera.newHeight",
    "Camera1.fx",
    "Camera1.fy",
    "Camera1.cx",
    "Camera1.cy",
    "Camera1.k1",
    "Camera1.k2",
    "Camera1.k3",
    "Camera1.k4",
    "Camera1.p1",
    "Camera1.p2",
    "Camera1.overlappingBegin",
    "Camera1.overlappingEnd",
    "Camera2.fx",
    "Camera2.fy",
    "Camera2.cx",
    "Camera2.cy",
    "Camera2.k1",
    "Camera2.k2",
    "Camera2.k3",
    "Camera2.k4",
    "Camera2.p1",
    "Camera2.p2",
    "Camera2.overlappingBegin",
    "Camera2.overlappingEnd",
    "Stereo.T_c1_c2",
    "IMU.staticInit",
    "IMU.NoiseGyro",
    "IMU.NoiseAcc",
    "IMU.GyroWalk",
    "IMU.AccWalk",
    "IMU.Frequency",
}
IMU_EXTRINSIC_KEY = "IMU.T_b_c1"
IMU_INIT_GYRO_BIAS_KEY = "IMU.InitGyroBias"
IMU_INPUT_CALIBRATION_KEYS = {
    "IMU.InputGyroRotation",
    "IMU.InputAccelRotation",
    "IMU.InputTimeOffset",
}

# The second-generation/headband recorder's CSV gyro and accelerometer axes
# differ from the calibrated IMU body axes described by T_BS. Older datasets do
# not carry that extra metadata, so retain the validated profile as a
# compatibility match on their distinctive T_BS rotation.
HEADBAND_T_BS_ROTATION = np.array(
    [
        [0.0580, -0.4330, -0.8995],
        [0.2500, -0.8660, 0.4330],
        [-0.9665, -0.2500, 0.0580],
    ],
    dtype=float,
)
HEADBAND_45_T_BS_ROTATION = np.array(
    [
        [0.066987298, -0.353553391, -0.933012702],
        [0.353553391, -0.866025404, 0.353553391],
        [-0.933012702, -0.353553391, 0.066987298],
    ],
    dtype=float,
)
HEADBAND_45_OFFLINE_NOISE_ACC = 0.05
HEADBAND_INPUT_GYRO_ROTATION = np.array(
    [
        [-0.805601141323, 0.357353409283, -0.472551946322],
        [0.438369796872, -0.177015903015, -0.881190837033],
        [-0.398545759342, -0.917040844758, -0.014048727937],
    ],
    dtype=float,
)
HEADBAND_INPUT_ACCEL_ROTATION = np.array(
    [
        [-0.799688274883, -0.367177014565, 0.475057578604],
        [0.442955881283, 0.173340642379, 0.879626687258],
        [-0.405325486801, 0.913856696465, 0.024024738803],
    ],
    dtype=float,
)
HEADBAND_INPUT_TIME_OFFSET_S = -0.024

# The M900 profile stores its calibrated camera-to-raw-IMU rotation directly
# in T_BS, so no additional input-axis rotation is required. Keep the measured
# camera/IMU timestamp correction here to preserve the standard EuRoC sensor
# YAML schema while still generating the runtime setting automatically.
M900_T_BS_ROTATION = np.array(
    [
        [0.0, -0.707106781, 0.707106781],
        [-1.0, 0.0, 0.0],
        [0.0, -0.707106781, -0.707106781],
    ],
    dtype=float,
)
M900_INPUT_TIME_OFFSET_S = -0.0222

KEY_RE = re.compile(r"^([A-Za-z0-9_.]+):")
INT_SETTING_RE = re.compile(r"^\s*{key}\s*:\s*([0-9]+)\s*(?:#.*)?$")


def read_int_setting(path, key, default=None):
    pattern = re.compile(INT_SETTING_RE.pattern.format(key=re.escape(key)))
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            match = pattern.match(line)
            if match:
                return int(match.group(1))
    return default


def choose_processing_size(width, height, base_settings):
    target_width = read_int_setting(base_settings, "Camera.newWidth", 640)
    target_height = read_int_setting(base_settings, "Camera.newHeight", None)
    if not target_width or target_width <= 0:
        target_width = 640

    if width <= target_width:
        return width, height

    scale = target_width / float(width)
    proc_width = target_width
    proc_height = max(1, int(round(height * scale)))

    # Preserve the existing 16:9 behavior exactly for the old 1920x1080 camera.
    if target_height and width == 1920 and height == 1080:
        proc_height = target_height

    return proc_width, proc_height


def load_yaml(path):
    if isinstance(path, dict):
        return path
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def require_sequence(sensor, key, length, path):
    value = sensor.get(key)
    if not isinstance(value, list) or len(value) < length:
        raise ValueError(f"{path}: expected '{key}' with at least {length} values")
    return value


def require_t_bs(sensor, path):
    t_bs = sensor.get("T_BS")
    if not isinstance(t_bs, dict) or "data" not in t_bs:
        raise ValueError(f"{path}: missing T_BS.data")
    rows = int(t_bs.get("rows", 4))
    cols = int(t_bs.get("cols", 4))
    data = t_bs["data"]
    if rows != 4 or cols != 4 or not isinstance(data, list) or len(data) != 16:
        raise ValueError(f"{path}: expected T_BS to be a 4x4 matrix")
    return np.array(data, dtype=float).reshape(4, 4)


def require_imu_t_bs(sensor, path):
    """Validate a dataset IMU transform before exposing it to ORB-SLAM3."""
    transform = require_t_bs(sensor, path)
    if not np.isfinite(transform).all():
        raise ValueError(f"{path}: T_BS contains non-finite values")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], rtol=0.0, atol=1e-9):
        raise ValueError(f"{path}: T_BS has an invalid homogeneous final row")

    rotation = transform[:3, :3]
    determinant = float(np.linalg.det(rotation))
    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    if abs(determinant - 1.0) > 1e-3 or orthogonality_error > 1e-3:
        raise ValueError(f"{path}: T_BS rotation is not a proper rotation matrix")
    return transform


def require_rotation(sensor, key, path):
    value = sensor.get(key)
    if isinstance(value, dict):
        if value.get("rows", 3) != 3 or value.get("cols", 3) != 3:
            raise ValueError(f"{path}: {key} must be a 3x3 matrix")
        value = value.get("data")
    try:
        rotation = np.asarray(value, dtype=float).reshape(3, 3)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{path}: {key} must contain 9 numeric values") from exc
    determinant = float(np.linalg.det(rotation))
    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    if (
        not np.isfinite(rotation).all()
        or abs(determinant - 1.0) > 1e-3
        or orthogonality_error > 1e-3
    ):
        raise ValueError(f"{path}: {key} is not a proper rotation matrix")
    return rotation


def input_time_offset(sensor, default, path):
    try:
        time_offset = float(sensor.get("input_time_offset_s", default))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            f"{path}: input_time_offset_s must be finite and within +/-0.2 seconds"
        ) from exc
    if not np.isfinite(time_offset) or abs(time_offset) > 0.2:
        raise ValueError(
            f"{path}: input_time_offset_s must be finite and within +/-0.2 seconds"
        )
    return time_offset


def imu_input_calibration(sensor, t_bs, path):
    gyro_key = "input_gyro_rotation"
    accel_key = "input_accel_rotation"
    has_gyro = gyro_key in sensor
    has_accel = accel_key in sensor
    if has_gyro != has_accel:
        raise ValueError(
            f"{path}: {gyro_key} and {accel_key} must be provided together"
        )
    if has_gyro:
        return {
            "source": "dataset metadata",
            "gyro_rotation": require_rotation(sensor, gyro_key, path),
            "accel_rotation": require_rotation(sensor, accel_key, path),
            "time_offset_s": input_time_offset(sensor, 0.0, path),
            "gyro_bias": (
                np.zeros((3, 1), dtype=float)
                if sensor.get("runtime_gyro_bias_corrected") is True
                else None
            ),
        }

    headband_source = None
    if np.allclose(
        t_bs[:3, :3], HEADBAND_T_BS_ROTATION, rtol=0.0, atol=1e-4
    ):
        headband_source = "legacy headband T_BS profile"
    elif np.allclose(
        t_bs[:3, :3], HEADBAND_45_T_BS_ROTATION, rtol=0.0, atol=1e-4
    ):
        headband_source = "headband 45-degree T_BS profile"
    if headband_source is not None:
        return {
            "source": headband_source,
            "gyro_rotation": HEADBAND_INPUT_GYRO_ROTATION,
            "accel_rotation": HEADBAND_INPUT_ACCEL_ROTATION,
            "time_offset_s": input_time_offset(
                sensor, HEADBAND_INPUT_TIME_OFFSET_S, path
            ),
            "gyro_bias": (
                np.zeros((3, 1), dtype=float)
                if sensor.get("runtime_gyro_bias_corrected") is True
                else None
            ),
        }
    if np.allclose(
        t_bs[:3, :3], M900_T_BS_ROTATION, rtol=0.0, atol=1e-4
    ):
        return {
            "source": "M900 calibrated T_BS profile",
            "gyro_rotation": np.eye(3),
            "accel_rotation": np.eye(3),
            "time_offset_s": input_time_offset(
                sensor, M900_INPUT_TIME_OFFSET_S, path
            ),
            "gyro_bias": (
                np.zeros((3, 1), dtype=float)
                if sensor.get("runtime_gyro_bias_corrected") is True
                else None
            ),
        }
    return None


def invert_se3(transform):
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def mat_to_opencv_yaml(name, mat):
    mat = np.asarray(mat, dtype=float)
    rows, cols = mat.shape
    data_rows = []
    for row in mat:
        data_rows.append(", ".join(f"{value:.16g}" for value in row))
    data = (",\n          ").join(data_rows)
    return "\n".join(
        [
            f"{name}: !!opencv-matrix",
            f"   rows: {rows}",
            f"   cols: {cols}",
            "   dt: f",
            f"   data: [{data}]",
        ]
    )


def is_fisheye(sensor):
    camera_model = str(sensor.get("camera_model", "")).lower()
    distortion_model = str(sensor.get("distortion_model", "")).lower()
    distortion_model_full = str(sensor.get("distortion_model_full", "")).lower()
    return (
        camera_model in {"fisheye", "kannalabrandt8", "kannala_brandt8"}
        or distortion_model in {"equidistant", "opencv-fisheye", "opencv_fisheye"}
        or distortion_model_full in {"equidistant", "opencv-fisheye", "opencv_fisheye"}
    )


def normalize_camera_intrinsics_swap_mode(value):
    if value is True:
        return "all"
    if value in (False, None, "", "0", "none"):
        return "none"
    mode = str(value).lower()
    if mode == "1":
        return "all"
    if mode not in {"all", "focal", "principal", "distortion"}:
        raise ValueError(
            "camera intrinsics swap mode must be one of: "
            "none, all, focal, principal, distortion"
        )
    return mode


def build_camera_block(
    base_settings,
    left_path,
    right_path,
    swap_camera_intrinsics=False,
):
    swap_mode = normalize_camera_intrinsics_swap_mode(swap_camera_intrinsics)
    left = load_yaml(left_path)
    right = load_yaml(right_path)

    left_intr = list(require_sequence(left, "intrinsics", 4, left_path))
    right_intr = list(require_sequence(right, "intrinsics", 4, right_path))
    left_dist = list(require_sequence(left, "distortion_coefficients", 4, left_path))
    right_dist = list(require_sequence(right, "distortion_coefficients", 4, right_path))
    left_resolution = require_sequence(left, "resolution", 2, left_path)
    right_resolution = require_sequence(right, "resolution", 2, right_path)

    if left_resolution != right_resolution:
        print(
            f"[WARN] Camera resolutions differ; using left resolution: "
            f"left={left_resolution}, right={right_resolution}",
            file=sys.stderr,
        )

    left_is_fisheye = is_fisheye(left)
    right_is_fisheye = is_fisheye(right)
    if left_is_fisheye != right_is_fisheye:
        raise ValueError(
            f"camera model mismatch: {left_path} fisheye={left_is_fisheye}, "
            f"{right_path} fisheye={right_is_fisheye}"
        )
    if swap_mode != "none":
        if left_resolution != right_resolution:
            raise ValueError(
                "cannot swap camera intrinsics between different resolutions: "
                f"left={left_resolution}, right={right_resolution}"
            )
    if swap_mode in {"all", "focal"}:
        for index in (0, 1):
            left_intr[index], right_intr[index] = right_intr[index], left_intr[index]
    if swap_mode in {"all", "principal"}:
        for index in (2, 3):
            left_intr[index], right_intr[index] = right_intr[index], left_intr[index]
    if swap_mode in {"all", "distortion"}:
        left_dist, right_dist = right_dist, left_dist

    width = int(left_resolution[0])
    height = int(left_resolution[1])
    proc_width, proc_height = choose_processing_size(width, height, base_settings)
    camera_type = "KannalaBrandt8" if left_is_fisheye else "PinHole"

    t_bs_left = require_t_bs(left, left_path)
    t_bs_right = require_t_bs(right, right_path)
    t_c1_c2 = t_bs_left @ invert_se3(t_bs_right)
    baseline = float(np.linalg.norm(t_c1_c2[:3, 3]))

    lines = [
        "",
        "#--------------------------------------------------------------------------------------------",
        "# Dataset camera calibration override",
        f"# Base settings: {base_settings}",
        f"# Left sensor:   {left_path}",
        f"# Right sensor:  {right_path}",
        f"# Baseline:      {baseline:.6f} m",
        (
            f"# Intrinsics:    EXPERIMENTAL cam0/cam1 swap mode: {swap_mode}"
            if swap_mode != "none"
            else "# Intrinsics:    dataset camera assignment"
        ),
        "#--------------------------------------------------------------------------------------------",
        f'Camera.type: "{camera_type}"',
        "",
        (
            f"# Left camera experimental swap mode: {swap_mode}"
            if swap_mode != "none"
            else "# Left camera intrinsics and distortion from cam0/sensor.yaml"
        ),
        f"Camera1.fx: {left_intr[0]}",
        f"Camera1.fy: {left_intr[1]}",
        f"Camera1.cx: {left_intr[2]}",
        f"Camera1.cy: {left_intr[3]}",
        "",
        (
            f"# Right camera experimental swap mode: {swap_mode}"
            if swap_mode != "none"
            else "# Right camera intrinsics and distortion from cam1/sensor.yaml"
        ),
        f"Camera2.fx: {right_intr[0]}",
        f"Camera2.fy: {right_intr[1]}",
        f"Camera2.cx: {right_intr[2]}",
        f"Camera2.cy: {right_intr[3]}",
        "",
    ]

    if camera_type == "KannalaBrandt8":
        lines.extend(
            [
                "# Kannala-Brandt / OpenCV fisheye distortion parameters",
                f"Camera1.k1: {left_dist[0]}",
                f"Camera1.k2: {left_dist[1]}",
                f"Camera1.k3: {left_dist[2]}",
                f"Camera1.k4: {left_dist[3]}",
                "",
                f"Camera2.k1: {right_dist[0]}",
                f"Camera2.k2: {right_dist[1]}",
                f"Camera2.k3: {right_dist[2]}",
                f"Camera2.k4: {right_dist[3]}",
                "",
                "Camera1.overlappingBegin: 0",
                f"Camera1.overlappingEnd: {width - 1}",
                "Camera2.overlappingBegin: 0",
                f"Camera2.overlappingEnd: {width - 1}",
                "",
            ]
        )
    else:
        lines.extend(
            [
                "# Pinhole/radtan distortion parameters",
                f"Camera1.k1: {left_dist[0]}",
                f"Camera1.k2: {left_dist[1]}",
                f"Camera1.p1: {left_dist[2]}",
                f"Camera1.p2: {left_dist[3]}",
                f"Camera2.k1: {right_dist[0]}",
                f"Camera2.k2: {right_dist[1]}",
                f"Camera2.p1: {right_dist[2]}",
                f"Camera2.p2: {right_dist[3]}",
                "",
            ]
        )
        if len(left_dist) >= 5 and len(right_dist) >= 5:
            lines.extend(
                [
                    f"Camera1.k3: {left_dist[4]}",
                    f"Camera2.k3: {right_dist[4]}",
                    "",
                ]
            )

    lines.extend(
        [
            f"Camera.width: {width}",
            f"Camera.height: {height}",
            f"Camera.newWidth: {proc_width}",
            f"Camera.newHeight: {proc_height}",
            "",
            "# Stereo extrinsic: transform from right camera to left camera",
            "# Computed as T_c1_c2 = T_BS_left * inverse(T_BS_right)",
            mat_to_opencv_yaml("Stereo.T_c1_c2", t_c1_c2),
            "",
        ]
    )

    return "\n".join(lines), baseline


def resolve_mav0(dataset):
    mav0 = os.path.join(os.path.abspath(dataset), "mav0")
    if os.path.isdir(mav0):
        return mav0
    return os.path.abspath(dataset)


def load_imu_samples(csv_path, max_window_s=1.5):
    samples = []
    if not os.path.isfile(csv_path):
        return samples

    first_ts = None
    with open(csv_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 7:
                continue
            try:
                ts = float(parts[0]) * 1e-9
                gyro = np.array([float(parts[1]), float(parts[2]), float(parts[3])], dtype=float)
                acc = np.array([float(parts[4]), float(parts[5]), float(parts[6])], dtype=float)
            except ValueError:
                continue
            if first_ts is None:
                first_ts = ts
            if ts - first_ts > max_window_s:
                break
            samples.append((ts, gyro, acc))
    return samples


def opening_is_stationary(
    samples,
    gyro_median_max=0.08,
    gyro_p95_max=0.10,
    acc_median_gravity_tol=0.25,
    acc_norm_p95_max=0.30,
    acc_norm_p99_max=0.40,
    acc_vector_p95_max=0.40,
    acc_direction_p95_max_deg=2.0,
    min_samples=20,
    min_duration_s=1.25,
    max_gap_s=0.10,
):
    empty_stats = {
        "sample_count": len(samples),
        "duration_s": None,
        "max_gap_s": None,
        "gyro_median": None,
        "gyro_p95": None,
        "acc_norm_median": None,
        "acc_norm_p95_error": None,
        "acc_norm_p95_deviation": None,
        "acc_norm_p99_deviation": None,
        "acc_vector_p95_deviation": None,
        "acc_direction_p95_deg": None,
    }
    if len(samples) < min_samples:
        return False, empty_stats

    timestamps = np.array([timestamp for timestamp, _, _ in samples], dtype=float)
    time_deltas = np.diff(timestamps)
    duration_s = float(timestamps[-1] - timestamps[0])
    observed_max_gap_s = float(np.max(time_deltas)) if time_deltas.size else float("inf")
    gyro_norms = np.array([np.linalg.norm(gyro) for _, gyro, _ in samples], dtype=float)
    acc_vectors = np.array([acc for _, _, acc in samples], dtype=float)
    acc_norms = np.linalg.norm(acc_vectors, axis=1)
    acc_errors = np.abs(acc_norms - 9.81)
    gyro_median = float(np.median(gyro_norms))
    gyro_p95 = float(np.percentile(gyro_norms, 95))
    acc_norm_median = float(np.median(acc_norms))
    acc_norm_p95_error = float(np.percentile(acc_errors, 95))
    acc_norm_deviations = np.abs(acc_norms - acc_norm_median)
    acc_norm_p95_deviation = float(np.percentile(acc_norm_deviations, 95))
    acc_norm_p99_deviation = float(np.percentile(acc_norm_deviations, 99))
    acc_vector_median = np.median(acc_vectors, axis=0)
    acc_vector_p95_deviation = float(
        np.percentile(np.linalg.norm(acc_vectors - acc_vector_median, axis=1), 95)
    )
    acc_vector_median_norm = float(np.linalg.norm(acc_vector_median))
    if acc_vector_median_norm > 1e-9 and np.all(acc_norms > 1e-9):
        direction_cosines = (
            acc_vectors @ acc_vector_median
        ) / (acc_norms * acc_vector_median_norm)
        direction_angles = np.degrees(
            np.arccos(np.clip(direction_cosines, -1.0, 1.0))
        )
        acc_direction_p95_deg = float(np.percentile(direction_angles, 95))
    else:
        acc_direction_p95_deg = float("inf")

    timestamps_are_valid = (
        duration_s >= min_duration_s
        and np.all(time_deltas > 0.0)
        and observed_max_gap_s <= max_gap_s
    )
    stationary = (
        timestamps_are_valid
        and gyro_median <= gyro_median_max
        and gyro_p95 <= gyro_p95_max
        and abs(acc_norm_median - 9.81) <= acc_median_gravity_tol
        and acc_norm_p95_deviation <= acc_norm_p95_max
        and acc_norm_p99_deviation <= acc_norm_p99_max
        and acc_vector_p95_deviation <= acc_vector_p95_max
        and acc_direction_p95_deg <= acc_direction_p95_max_deg
    )
    return stationary, {
        "sample_count": len(samples),
        "duration_s": duration_s,
        "max_gap_s": observed_max_gap_s,
        "gyro_median": gyro_median,
        "gyro_p95": gyro_p95,
        "acc_norm_median": acc_norm_median,
        "acc_norm_p95_error": acc_norm_p95_error,
        "acc_norm_p95_deviation": acc_norm_p95_deviation,
        "acc_norm_p99_deviation": acc_norm_p99_deviation,
        "acc_vector_p95_deviation": acc_vector_p95_deviation,
        "acc_direction_p95_deg": acc_direction_p95_deg,
    }


def build_imu_block(
    dataset,
    static_init_override=None,
    noise_acc_override=None,
    extrinsic_source="base",
    imu_sensor=None,
    imu_data_path=None,
    imu_source_label=None,
    imu_samples=None,
):
    if extrinsic_source not in {"base", "dataset"}:
        raise ValueError("IMU extrinsic source must be one of: base, dataset")

    mav0 = resolve_mav0(dataset)
    imu_sensor_path = imu_source_label or os.path.join(mav0, "imu0", "sensor.yaml")
    if imu_data_path is None:
        imu_data_path = os.path.join(mav0, "imu0", "data.csv")
    if imu_sensor is None and not os.path.isfile(imu_sensor_path):
        if extrinsic_source == "dataset":
            raise ValueError(
                f"dataset IMU extrinsic requested but sensor metadata is missing: {imu_sensor_path}"
            )
        return "", {"available": False, "reason": f"missing {imu_sensor_path}"}

    imu = imu_sensor if imu_sensor is not None else load_yaml(imu_sensor_path)
    imu_t_bs = None
    if isinstance(imu.get("T_BS"), dict):
        imu_t_bs = require_imu_t_bs(imu, imu_sensor_path)
    elif extrinsic_source == "dataset":
        raise ValueError(
            f"dataset IMU extrinsic requested but T_BS is missing: {imu_sensor_path}"
        )
    imu_extrinsic = None
    if extrinsic_source == "dataset":
        imu_extrinsic = imu_t_bs
    input_calibration = (
        imu_input_calibration(imu, imu_t_bs, imu_sensor_path)
        if imu_t_bs is not None
        else None
    )
    required = {
        "rate_hz": "IMU.Frequency",
        "gyroscope_noise_density": "IMU.NoiseGyro",
        "accelerometer_noise_density": "IMU.NoiseAcc",
        "gyroscope_random_walk": "IMU.GyroWalk",
        "accelerometer_random_walk": "IMU.AccWalk",
    }
    missing = [key for key in required if key not in imu]
    if missing:
        raise ValueError(f"{imu_sensor_path}: missing IMU calibration keys: {', '.join(missing)}")

    samples = imu_samples if imu_samples is not None else load_imu_samples(imu_data_path)
    stationary, stats = opening_is_stationary(samples)
    if static_init_override is None:
        static_init = bool(stationary)
        static_init_source = "auto"
    else:
        static_init = bool(static_init_override)
        static_init_source = "forced"
    is_headband_45 = imu_t_bs is not None and np.allclose(
        imu_t_bs[:3, :3], HEADBAND_45_T_BS_ROTATION, rtol=0.0, atol=1e-4
    )
    if noise_acc_override is None:
        if is_headband_45:
            # The profile keeps the measured sensor noise density.  This
            # offline backend needs a more conservative acceleration weight:
            # otherwise rotation segments pull the metric stereo translation
            # away from the visual loop by roughly one metre on these takes.
            noise_acc = HEADBAND_45_OFFLINE_NOISE_ACC
            noise_acc_source = "headband45 offline default"
        else:
            noise_acc = float(imu["accelerometer_noise_density"])
            noise_acc_source = "dataset"
    else:
        noise_acc = float(noise_acc_override)
        if not np.isfinite(noise_acc) or noise_acc <= 0.0:
            raise ValueError("IMU accelerometer noise override must be finite and positive")
        noise_acc_source = "override"

    decision = {
        "available": True,
        "static_init": static_init,
        "static_init_source": static_init_source,
        "noise_acc": noise_acc,
        "noise_acc_source": noise_acc_source,
        "headband_45": bool(is_headband_45),
        "extrinsic_source": extrinsic_source,
        "extrinsic_path": imu_sensor_path if imu_extrinsic is not None else None,
        "input_calibration_source": (
            input_calibration["source"] if input_calibration else None
        ),
        "input_gyro_bias_source": (
            "runtime-corrected compact-v2 measurements"
            if input_calibration is not None
            and input_calibration.get("gyro_bias") is not None
            else None
        ),
        **stats,
    }

    lines = [
        "",
        "#--------------------------------------------------------------------------------------------",
        "# Dataset IMU override",
        f"# IMU sensor: {imu_sensor_path}",
        f"# Opening window samples: {decision['sample_count']}",
    ]
    if decision["gyro_median"] is not None:
        lines.extend(
            [
                f"# Opening duration/max gap: {decision['duration_s']:.6g} / {decision['max_gap_s']:.6g} s",
                f"# Opening gyro norm median/p95: {decision['gyro_median']:.6g} / {decision['gyro_p95']:.6g} rad/s",
                f"# Opening acc norm median/centered p95/p99: {decision['acc_norm_median']:.6g} / {decision['acc_norm_p95_deviation']:.6g} / {decision['acc_norm_p99_deviation']:.6g} m/s^2",
                f"# Opening acc vector p95/direction p95: {decision['acc_vector_p95_deviation']:.6g} m/s^2 / {decision['acc_direction_p95_deg']:.6g} deg",
            ]
        )
    lines.extend(
        [
            "# Static init is enabled only when the opening window is actually stationary.",
            "# Non-stationary recordings use motion-based IMU initialization instead.",
            f"# Static init decision: {static_init_source}",
            f"# Accelerometer noise source: {noise_acc_source}",
            "#--------------------------------------------------------------------------------------------",
            f"IMU.staticInit: {1 if static_init else 0}",
            f"IMU.NoiseGyro: {float(imu['gyroscope_noise_density']):.12g}",
            f"IMU.NoiseAcc: {noise_acc:.12g}",
            f"IMU.GyroWalk: {float(imu['gyroscope_random_walk']):.12g}",
            f"IMU.AccWalk: {float(imu['accelerometer_random_walk']):.12g}",
            f"IMU.Frequency: {float(imu['rate_hz']):.1f}",
            "",
        ]
    )
    if imu_extrinsic is not None:
        lines.extend(
            [
                "# IMU extrinsic from dataset imu0/sensor.yaml:T_BS",
                mat_to_opencv_yaml(IMU_EXTRINSIC_KEY, imu_extrinsic),
                "",
            ]
        )
    if input_calibration is not None:
        lines.extend(
            [
                f"# Raw IMU input calibration: {input_calibration['source']}",
                mat_to_opencv_yaml(
                    "IMU.InputGyroRotation", input_calibration["gyro_rotation"]
                ),
                mat_to_opencv_yaml(
                    "IMU.InputAccelRotation", input_calibration["accel_rotation"]
                ),
                f"IMU.InputTimeOffset: {input_calibration['time_offset_s']:.12g}",
                "",
            ]
        )
        if input_calibration.get("gyro_bias") is not None:
            lines.extend(
                [
                    "# compact-v2 gyro bias was already removed by the recorder",
                    mat_to_opencv_yaml(
                        IMU_INIT_GYRO_BIAS_KEY,
                        input_calibration["gyro_bias"],
                    ),
                    "",
                ]
            )
    return "\n".join(lines), decision


def merge_settings(
    base_path,
    camera_block,
    imu_block="",
    replace_imu_extrinsic=False,
    replace_imu_input_calibration=False,
    replace_imu_init_gyro_bias=False,
):
    with open(base_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    managed_keys = set(MANAGED_KEYS)
    if replace_imu_extrinsic:
        managed_keys.add(IMU_EXTRINSIC_KEY)
    if replace_imu_input_calibration:
        managed_keys.update(IMU_INPUT_CALIBRATION_KEYS)
    if replace_imu_init_gyro_bias:
        managed_keys.add(IMU_INIT_GYRO_BIAS_KEY)

    output = []
    inserted = False
    index = 0
    while index < len(lines):
        line = lines[index]
        match = KEY_RE.match(line)
        key = match.group(1) if match else None
        if key in managed_keys:
            if not inserted:
                output.append(camera_block)
                if imu_block:
                    output.append(imu_block)
                inserted = True
            index += 1
            while index < len(lines) and lines[index].startswith((" ", "\t")):
                index += 1
            continue
        output.append(line.rstrip("\n"))
        index += 1

    if not inserted:
        insertion_index = 0
        for i, line in enumerate(output):
            if line.startswith("System.mode:"):
                insertion_index = i + 1
                break
        blocks = [camera_block]
        if imu_block:
            blocks.append(imu_block)
        output[insertion_index:insertion_index] = blocks

    return "\n".join(output).rstrip() + "\n"


def override_scalar_setting(settings, key, value):
    pattern = re.compile(rf"^\s*{re.escape(key)}\s*:.*$", re.MULTILINE)
    replacement = f"{key}: {value}"
    if pattern.search(settings):
        return pattern.sub(replacement, settings, count=1)
    return settings.rstrip() + f"\n{replacement}\n"


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Merge dataset calibration_full.json into ORB-SLAM3 settings; "
            "head is the default and --take selects compact-v2"
        )
    )
    parser.add_argument("--base-settings", required=True, help="Base ORB-SLAM3 settings YAML")
    parser.add_argument("--dataset", required=True, help="Dataset root")
    parser.add_argument(
        "--calibration-file",
        help="Override embedded calibration_full.json without modifying the dataset",
    )
    parser.add_argument(
        "--imu-profile",
        help=(
            "Override IMU extrinsic/noise metadata with a sensor.yaml profile; "
            "IMU samples are still read from the dataset"
        ),
    )
    parser.add_argument(
        "--take",
        action="store_true",
        help="Read an optional compact-v2 take instead of the default head dataset",
    )
    parser.add_argument("--output", required=True, help="Output merged settings YAML")
    parser.add_argument("--left-cam", default="cam0", help="Left camera directory name")
    parser.add_argument("--right-cam", default="cam1", help="Right camera directory name")
    parser.add_argument(
        "--swap-camera-intrinsics",
        nargs="?",
        const="all",
        choices=("all", "focal", "principal", "distortion"),
        default="none",
        help=(
            "Experiment only: swap all camera intrinsics, only fx/fy (focal), "
            "only cx/cy (principal), or only distortion, while preserving "
            "stereo extrinsics; a bare flag selects all"
        ),
    )
    parser.add_argument("--pose-port", type=int, help="Override ZMQ.pose_port for this run")
    parser.add_argument(
        "--imu-static-init",
        choices=("auto", "0", "1"),
        default="auto",
        help="Select IMU initialization: auto-detect, force motion-based (0), or force guarded static init (1)",
    )
    parser.add_argument(
        "--imu-scale-only",
        choices=("0", "1"),
        help=(
            "Override IMU.scaleOnly: 0 keeps the full inertial backend; "
            "1 keeps visual-stereo poses and uses IMU only as a guarded scale backend"
        ),
    )
    parser.add_argument(
        "--loop-closing",
        choices=("0", "1"),
        help=(
            "Override loopClosing for this run. The base setting is preserved "
            "when this option is omitted."
        ),
    )
    parser.add_argument(
        "--imu-noise-acc",
        type=float,
        help="Override IMU accelerometer noise density for this run",
    )
    parser.add_argument(
        "--imu-input-time-offset",
        type=float,
        help="Add this many seconds to every loaded IMU timestamp",
    )
    parser.add_argument(
        "--imu-extrinsic-source",
        choices=("base", "dataset"),
        default="base",
        help=(
            "Select IMU.T_b_c1 from the base settings (default), or replace it "
            "with the selected dataset calibration"
        ),
    )
    args = parser.parse_args()

    if args.imu_profile and not os.path.isfile(args.imu_profile):
        parser.error(f"--imu-profile not found: {args.imu_profile}")

    if args.pose_port is not None and not 1024 <= args.pose_port <= 65535:
        parser.error("--pose-port must be between 1024 and 65535")
    if args.imu_input_time_offset is not None and (
        not np.isfinite(args.imu_input_time_offset)
        or abs(args.imu_input_time_offset) > 0.2
    ):
        parser.error("--imu-input-time-offset must be finite and within +/-0.2 seconds")

    dataset = os.path.abspath(args.dataset)
    try:
        layout = DatasetLayout.open(
            dataset,
            take=args.take,
            calibration_file=args.calibration_file,
        )
        layout.validate(
            cameras=(args.left_cam, args.right_cam),
            imu_ids=("imu0",),
            require_calibration=True,
        )
        left_sensor = layout.load_camera_sensor(args.left_cam)
        right_sensor = layout.load_camera_sensor(args.right_cam)
        compact_imu_sensor = layout.load_imu_sensor("imu0")
        compact_imu_samples = [
            (
                float(row["timestamp_ns"]) * 1e-9,
                np.array([float(row["gx_rad_s"]), float(row["gy_rad_s"]), float(row["gz_rad_s"])]),
                np.array([float(row["ax_m_s2"]), float(row["ay_m_s2"]), float(row["az_m_s2"])]),
            )
            for row in layout.iter_imu_rows("imu0")
        ]
        imu_source_label = f"{layout.calibration_path()}:imus.imu0"
        processing_dataset = dataset
    except Exception as exc:
        print(f"ERROR: failed to resolve {('take' if args.take else 'head')} dataset: {exc}", file=sys.stderr)
        return 1

    if args.imu_profile:
        try:
            compact_imu_sensor = load_yaml(os.path.abspath(args.imu_profile))
            if not isinstance(compact_imu_sensor, dict):
                raise ValueError("profile root must be a mapping")
            imu_source_label = os.path.abspath(args.imu_profile)
        except Exception as exc:
            print(f"ERROR: failed to load IMU profile: {exc}", file=sys.stderr)
            return 1

    try:
        camera_block, baseline = build_camera_block(
            os.path.abspath(args.base_settings),
            left_sensor,
            right_sensor,
            swap_camera_intrinsics=args.swap_camera_intrinsics,
        )
        static_init_override = None
        if args.imu_static_init != "auto":
            static_init_override = int(args.imu_static_init)
        imu_block, imu_decision = build_imu_block(
            processing_dataset,
            static_init_override,
            args.imu_noise_acc,
            args.imu_extrinsic_source,
            imu_sensor=compact_imu_sensor,
            imu_source_label=imu_source_label,
            imu_samples=compact_imu_samples,
        )
        merged = merge_settings(
            args.base_settings,
            camera_block,
            imu_block,
            replace_imu_extrinsic=args.imu_extrinsic_source == "dataset",
            replace_imu_input_calibration=bool(
                imu_decision.get("input_calibration_source")
            ),
            replace_imu_init_gyro_bias=bool(
                imu_decision.get("input_gyro_bias_source")
            ),
        )
        if args.pose_port is not None:
            merged = override_scalar_setting(merged, "ZMQ.pose_port", args.pose_port)
        if args.imu_input_time_offset is not None:
            merged = override_scalar_setting(
                merged,
                "IMU.InputTimeOffset",
                f"{args.imu_input_time_offset:.12g}",
            )
        if args.imu_scale_only is not None:
            merged = override_scalar_setting(
                merged,
                "IMU.scaleOnly",
                args.imu_scale_only,
            )
        if args.loop_closing is not None:
            merged = override_scalar_setting(
                merged,
                "loopClosing",
                args.loop_closing,
            )
    except Exception as exc:
        print(f"ERROR: failed to merge dataset camera calibration: {exc}", file=sys.stderr)
        return 1

    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(merged)

    print(f"Merged settings: {args.output}")
    print(f"Dataset format:  {layout.format_name}")
    if args.calibration_file:
        print(f"Calibration:     {os.path.abspath(args.calibration_file)} (override)")
    if args.imu_profile:
        print(f"IMU profile:     {os.path.abspath(args.imu_profile)} (override)")
    print(f"Camera sensors:  {left_sensor} + {right_sensor}")
    print(
        "Camera intrinsics: "
        + (
            f"EXPERIMENTAL cam0/cam1 swap mode={args.swap_camera_intrinsics}"
            if args.swap_camera_intrinsics != "none"
            else "dataset assignment"
        )
    )
    print(f"Stereo baseline: {baseline:.6f} m")
    if imu_decision.get("available"):
        mode = "static" if imu_decision["static_init"] else "motion-based"
        source = imu_decision.get("static_init_source", "auto")
        metadata_source = imu_source_label or (
            "calibration_full.json:imus.imu0"
        )
        print(f"IMU noise/timing: {metadata_source}, init={mode} ({source})")
    else:
        print(
            "IMU noise/timing: base settings "
            f"({imu_decision.get('reason', 'no dataset IMU')})"
        )
    if args.imu_extrinsic_source == "dataset":
        print(f"IMU extrinsic:   {imu_decision['extrinsic_path']}:T_BS")
    else:
        print(f"IMU extrinsic:   base settings ({os.path.abspath(args.base_settings)})")
    if imu_decision.get("input_calibration_source"):
        print(
            "IMU input axes:  "
            f"{imu_decision['input_calibration_source']}"
        )
    if imu_decision.get("input_gyro_bias_source"):
        print(
            "IMU gyro bias:   "
            f"zeroed ({imu_decision['input_gyro_bias_source']})"
        )
    if args.pose_port is not None:
        print(f"ZMQ pose port:   {args.pose_port}")
    if args.imu_input_time_offset is not None:
        print(f"IMU time offset: {args.imu_input_time_offset:.9g} s")
    if args.imu_scale_only is not None:
        print(
            "IMU backend mode: "
            + (
                "visual-stereo poses with guarded IMU scale observation"
                if args.imu_scale_only == "1"
                else "full stereo-inertial"
            )
        )
    if args.loop_closing is not None:
        print(
            "Loop closing:   "
            + (
                "enabled (explicit override)"
                if args.loop_closing == "1"
                else "disabled"
            )
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
