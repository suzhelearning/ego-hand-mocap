"""WiLoR pose on calibrated virtual views using measured native detections."""

import time

import cv2
import numpy as np

from ego_hand_benchmark.core.skeleton import BONES, HANDS
from ego_hand_benchmark.evaluation.joint_mapping import wilor_camera_joints
from ego_hand_benchmark.geometry.fisheye import project
from ego_hand_benchmark.geometry.virtual_camera import (
    ViewUnavailable,
    calibrated_translation,
    make_view,
)


def infer_calibrated(
    image, detection, model, model_config, device, config, source_camera, save_preview=False
):
    import torch
    from wilor.datasets.vitdet_dataset import ViTDetDataset
    from wilor.utils import recursive_to

    boxes, scores, classes = (detection[k] for k in ("boxes_xyxy", "scores", "classes"))
    chosen = detection["selected_detection_indices"].astype(int)
    samples, geometry, ids, views = [], [], [], []
    reasons = np.array(["no_detection", "no_detection"], dtype="<U64")
    raw = {
        k: detection[k] for k in ("boxes_xyxy", "scores", "classes", "selected_detection_indices")
    }
    geometry_seconds = 0.0
    for person, selected in enumerate(chosen):
        side = int(classes[selected])
        tick = time.perf_counter()
        try:
            view, box, meta = make_view(
                image,
                boxes[selected],
                source_camera,
                size=config.get("virtual_size", 512),
                rescale_factor=config.get("rescale_factor", 2.0),
                max_fov=config.get("max_virtual_fov_deg", 160.0),
            )
        except ViewUnavailable as error:
            reasons[side] = str(error)
            geometry_seconds += time.perf_counter() - tick
            continue
        geometry_seconds += time.perf_counter() - tick
        dataset = ViTDetDataset(
            model_config,
            view,
            box[None],
            np.array([side]),
            rescale_factor=config.get("rescale_factor", 2.0),
            fp16=False,
        )
        sample = dataset[0]
        sample["personid"] = person
        samples.append(sample)
        geometry.append(meta)
        ids.append(selected)
        views.append(view)
    predictions, previews = [], []
    records = []
    if samples:
        batch = recursive_to(torch.utils.data.default_collate(samples), device)
        with torch.inference_mode():
            native = model(batch)
        native_numpy = {
            key: value.detach().cpu().numpy()
            for key, value in native.items()
            if torch.is_tensor(value)
        }
        native_numpy.update(
            {
                f"mano_{key}": value.detach().cpu().numpy()
                for key, value in native["pred_mano_params"].items()
            }
        )
        for i, selected in enumerate(ids):
            side = int(classes[selected])
            meta = geometry[i]
            center = batch["box_center"][i].detach().cpu().numpy()
            size = float(batch["box_size"][i].item())
            try:
                translation = calibrated_translation(
                    native_numpy["pred_cam"][i], center, size, meta["K_virtual"], side
                )
            except ViewUnavailable as error:
                reasons[side] = str(error)
                continue
            virtual_joints = wilor_camera_joints(
                native_numpy["pred_keypoints_3d"][i], translation, HANDS[side]
            )
            rotation = meta["R_virtual_from_source"]
            source_joints = virtual_joints @ rotation
            predictions.append((side, source_joints, float(scores[selected])))
            reasons[side] = ""
            record = {key: value[i] for key, value in native_numpy.items()}
            vertices = native_numpy["pred_vertices"][i].copy()
            vertices[:, 0] *= 2 * side - 1
            record.update(meta)
            record.update(
                cam_t_virtual=translation,
                cam_t_source=translation @ rotation,
                joints_camera_virtual=virtual_joints,
                joints_camera_source=source_joints,
                vertices_camera_source=(vertices + translation) @ rotation,
                crop_center_virtual=center,
                crop_size_virtual=np.array(size),
                is_right=np.array(side),
                predicted_detection_index=np.array(selected),
            )
            records.append(record)
            if save_preview:
                camera = {
                    "model": "PINHOLE",
                    "image_width": views[i].shape[1],
                    "image_height": views[i].shape[0],
                    "camera_matrix": {"matrix_3x3": meta["K_virtual"].tolist()},
                    "distortion_coefficients": [],
                }
                uv = project(virtual_joints, camera)
                preview = views[i].copy()
                for a, b in BONES:
                    if np.isfinite(uv[[a, b]]).all() and np.abs(uv[[a, b]]).max() < 1e5:
                        cv2.line(
                            preview,
                            tuple(uv[a].astype(int)),
                            tuple(uv[b].astype(int)),
                            (0, 255, 0),
                            1,
                            cv2.LINE_AA,
                        )
                previews.append((HANDS[side], preview))
    if records:
        for key in records[0]:
            raw[key] = np.stack([record[key] for record in records])
    raw["failure_reason_per_side"] = reasons
    raw["view_generation_seconds"] = np.array(geometry_seconds)
    raw["detector_seconds"] = np.asarray(detection["detector_seconds"])
    return predictions, raw, previews
