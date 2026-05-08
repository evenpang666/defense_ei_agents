"""RGB-D scene-state helpers for the real UR7e workflow."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence


@dataclass(frozen=True)
class RGBDFrame:
    color: Any
    depth_m: Any
    intrinsics: dict[str, float]
    depth_scale: float
    serial: str = ""


COLOR_THRESHOLDS: dict[str, tuple[int, int, int, int, int, int]] = {
    "red": (120, 255, 0, 110, 0, 110),
    "green": (0, 140, 80, 255, 0, 140),
    "blue": (0, 120, 0, 150, 100, 255),
    "yellow": (120, 255, 100, 255, 0, 130),
    "orange": (140, 255, 45, 180, 0, 90),
    "purple": (80, 190, 0, 100, 80, 220),
    "black": (0, 65, 0, 65, 0, 65),
    "white": (170, 255, 170, 255, 170, 255),
}

_YOLO_MODEL_CACHE: dict[str, Any] = {}


def load_transform(path_text: str | Path | None) -> list[list[float]] | None:
    """Load a 4x4 homogeneous transform from JSON, or return None."""
    if not path_text:
        return None
    path = Path(str(path_text)).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"transform file not found: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    raw = payload.get("matrix", payload.get("transform", payload))
    if not isinstance(raw, list) or len(raw) != 4:
        raise ValueError(f"expected a 4x4 transform matrix in {path}")
    matrix: list[list[float]] = []
    for row in raw:
        if not isinstance(row, list) or len(row) != 4:
            raise ValueError(f"expected a 4x4 transform matrix in {path}")
        matrix.append([float(v) for v in row])
    return matrix


def gripper_to_wrist_camera_transform(offset_y_m: float = -0.02) -> list[list[float]]:
    """Approximate gripper->wrist-camera transform.

    The default follows the lab note that the wrist camera is mounted slightly
    along gripper -Y. Rotation is identity until a proper hand-eye calibration
    is supplied.
    """
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, float(offset_y_m)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def tcp_pose_to_transform(pose_vec: Sequence[float]) -> list[list[float]]:
    if len(pose_vec) != 6:
        raise ValueError(f"expected [x,y,z,rx,ry,rz], got {len(pose_vec)} values")
    rot = _rotvec_to_matrix(pose_vec[3:6])
    return [
        [rot[0][0], rot[0][1], rot[0][2], float(pose_vec[0])],
        [rot[1][0], rot[1][1], rot[1][2], float(pose_vec[1])],
        [rot[2][0], rot[2][1], rot[2][2], float(pose_vec[2])],
        [0.0, 0.0, 0.0, 1.0],
    ]


def build_scene_state(
    *,
    task_text: str,
    global_frame: RGBDFrame | None,
    wrist_frame: RGBDFrame | None,
    tcp_pose_base: Sequence[float] | None,
    t_base_global_camera: Sequence[Sequence[float]] | None,
    t_gripper_wrist_camera: Sequence[Sequence[float]] | None,
    perception_backend: str = "color",
    yolo_model_path: str | Path | None = None,
    yolo_confidence: float = 0.35,
    yolo_iou: float = 0.5,
    yolo_target_labels: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Build a structured 3D scene-state summary for coder and judger."""
    backend = str(perception_backend or "color").strip().lower()
    labels = _labels_from_text(task_text, extra_labels=yolo_target_labels)
    tcp_transform = tcp_pose_to_transform(tcp_pose_base) if tcp_pose_base else None
    t_base_gripper = tcp_transform
    t_gripper_base = _invert_transform(t_base_gripper) if t_base_gripper else None
    t_base_wrist = (
        _matmul4(t_base_gripper, t_gripper_wrist_camera)
        if t_base_gripper is not None and t_gripper_wrist_camera is not None
        else None
    )
    t_wrist_base = _invert_transform(t_base_wrist) if t_base_wrist is not None else None

    objects: list[dict[str, Any]] = []
    perception_error: str | None = None
    if global_frame is not None:
        try:
            if backend == "yolo":
                estimates = _estimate_yolo_objects(
                    frame=global_frame,
                    model_path=yolo_model_path,
                    confidence=float(yolo_confidence),
                    iou=float(yolo_iou),
                    target_labels=yolo_target_labels or [],
                )
            elif backend == "none":
                estimates = []
            else:
                estimates = [
                    estimate
                    for label in labels
                    for estimate in [_estimate_color_object(label, global_frame)]
                    if estimate is not None
                ]
        except Exception as exc:
            estimates = []
            perception_error = str(exc)

        seen_requested = set()
        for estimate in estimates:
            label = str(estimate["label"])
            seen_requested.add(label.lower())
            objects.append(
                _object_record_from_estimate(
                    estimate=estimate,
                    t_base_global_camera=t_base_global_camera,
                    t_gripper_base=t_gripper_base,
                    t_wrist_base=t_wrist_base,
                    backend=backend,
                )
            )

        missing_labels = list(yolo_target_labels or ([] if backend == "yolo" else labels))
        for label in missing_labels:
            if label.lower() not in seen_requested and backend != "none":
                objects.append(
                    {
                        "label": label,
                        "source": f"global_rgbd_{backend}",
                        "status": "not_detected",
                        "confidence": 0.0,
                    }
                )

    return {
        "schema": "defense_ei_scene_state.v1",
        "status": "ok" if global_frame is not None else "unavailable",
        "coordinate_frames": {
            "base": "UR robot base frame.",
            "global_camera": "Fixed external RealSense camera frame.",
            "gripper": "Current UR TCP/gripper frame from RTDE.",
            "wrist_camera": "Camera mounted on gripper; default transform is approximate.",
        },
        "calibration": {
            "has_t_base_global_camera": t_base_global_camera is not None,
            "has_t_gripper_wrist_camera": t_gripper_wrist_camera is not None,
            "global_to_base_required_for_base_and_gripper_coordinates": True,
        },
        "perception": {
            "backend": backend,
            "yolo_model_path": str(yolo_model_path) if yolo_model_path else None,
            "yolo_confidence": float(yolo_confidence),
            "yolo_iou": float(yolo_iou),
            "error": perception_error,
        },
        "tcp_pose_base": _round_list(tcp_pose_base, 6),
        "t_base_gripper": _round_matrix(t_base_gripper, 6),
        "t_base_global_camera": _round_matrix(t_base_global_camera, 6),
        "t_gripper_wrist_camera": _round_matrix(t_gripper_wrist_camera, 6),
        "objects": objects,
        "object_labels_requested": labels,
        "wrist_rgbd": {
            "available": wrist_frame is not None,
            "role": "near-field alignment and verification; global RGB-D remains the full-scene source.",
        },
    }


def scene_state_brief(scene_state: dict[str, Any], *, max_objects: int = 8) -> dict[str, Any]:
    """Return a compact prompt-friendly scene-state subset."""
    objects = []
    for obj in scene_state.get("objects", [])[:max_objects]:
        objects.append(
            {
                "label": obj.get("label"),
                "status": obj.get("status"),
                "confidence": obj.get("confidence"),
                "center_base_m": obj.get("center_base_m"),
                "center_gripper_mm": obj.get("center_gripper_mm"),
                "grasp_region_center_gripper_mm": obj.get("grasp_region_center_gripper_mm"),
            }
        )
    return {
        "schema": scene_state.get("schema"),
        "status": scene_state.get("status"),
        "calibration": scene_state.get("calibration"),
        "perception": scene_state.get("perception"),
        "objects": objects,
    }


def _object_record_from_estimate(
    *,
    estimate: dict[str, Any],
    t_base_global_camera: Sequence[Sequence[float]] | None,
    t_gripper_base: Sequence[Sequence[float]] | None,
    t_wrist_base: Sequence[Sequence[float]] | None,
    backend: str,
) -> dict[str, Any]:
    center_global = estimate["center_camera_m"]
    center_base = (
        _transform_point(t_base_global_camera, center_global)
        if t_base_global_camera is not None
        else None
    )
    center_gripper = (
        _transform_point(t_gripper_base, center_base)
        if t_gripper_base is not None and center_base is not None
        else None
    )
    center_wrist = (
        _transform_point(t_wrist_base, center_base)
        if t_wrist_base is not None and center_base is not None
        else None
    )
    return {
        "label": estimate["label"],
        "source": f"global_rgbd_{backend}",
        "status": "estimated",
        "confidence": estimate["confidence"],
        "pixel_count": estimate["pixel_count"],
        "bbox_px": estimate["bbox_px"],
        "class_id": estimate.get("class_id"),
        "center_global_camera_m": _round_list(center_global, 5),
        "center_base_m": _round_list(center_base, 5),
        "center_gripper_mm": _round_list(_meters_to_mm(center_gripper), 1),
        "center_wrist_camera_mm": _round_list(_meters_to_mm(center_wrist), 1),
        "grasp_region_center_base_m": _round_list(center_base, 5),
        "grasp_region_center_gripper_mm": _round_list(_meters_to_mm(center_gripper), 1),
        "notes": estimate.get(
            "notes",
            "Grasp region equals robust segmentation-mask depth center.",
        ),
    }


def _labels_from_text(text: str, extra_labels: Sequence[str] | None = None) -> list[str]:
    lowered = str(text or "").lower()
    labels: list[str] = []
    for color in COLOR_THRESHOLDS:
        if color in lowered:
            labels.append(color)
    for label in extra_labels or []:
        clean = str(label).strip().lower()
        if clean and clean not in labels:
            labels.append(clean)
    if not labels:
        labels.append("foreground_object")
    return labels


def _estimate_yolo_objects(
    *,
    frame: RGBDFrame,
    model_path: str | Path | None,
    confidence: float,
    iou: float,
    target_labels: Sequence[str],
) -> list[dict[str, Any]]:
    import numpy as np

    if not model_path:
        raise RuntimeError("YOLO perception backend requires --yolo-seg-model.")
    model = _load_yolo_model(model_path)
    results = model.predict(
        source=frame.color,
        conf=float(confidence),
        iou=float(iou),
        verbose=False,
    )
    if not results:
        return []

    result = results[0]
    masks_obj = getattr(result, "masks", None)
    boxes_obj = getattr(result, "boxes", None)
    if masks_obj is None or getattr(masks_obj, "data", None) is None:
        raise RuntimeError("YOLO model returned no segmentation masks; use a segmentation model such as YOLOv8-seg.")

    names = getattr(result, "names", None) or getattr(model, "names", {}) or {}
    mask_data = masks_obj.data
    if hasattr(mask_data, "detach"):
        mask_np = mask_data.detach().cpu().numpy()
    else:
        mask_np = np.asarray(mask_data)

    confs: list[float] = []
    class_ids: list[int] = []
    if boxes_obj is not None:
        conf_raw = getattr(boxes_obj, "conf", None)
        cls_raw = getattr(boxes_obj, "cls", None)
        if conf_raw is not None:
            confs = [float(v) for v in conf_raw.detach().cpu().numpy().tolist()]
        if cls_raw is not None:
            class_ids = [int(v) for v in cls_raw.detach().cpu().numpy().tolist()]

    estimates: list[dict[str, Any]] = []
    target_set = {str(label).strip().lower() for label in target_labels if str(label).strip()}
    include_all = not target_set or target_set == {"foreground_object"}
    for idx, raw_mask in enumerate(mask_np):
        class_id = class_ids[idx] if idx < len(class_ids) else -1
        label = str(names.get(class_id, class_id))
        label_l = label.lower()
        if not include_all and not _label_matches(label_l, target_set):
            continue
        conf = confs[idx] if idx < len(confs) else float(confidence)
        mask = _resize_mask_nearest(np.asarray(raw_mask) > 0.5, frame.depth_m.shape)
        estimate = _estimate_mask_object(
            label=label,
            frame=frame,
            mask=mask,
            confidence=conf,
            class_id=class_id,
            source_note="YOLO segmentation mask with robust depth median.",
        )
        if estimate is not None:
            estimates.append(estimate)
    return estimates


def _load_yolo_model(model_path: str | Path) -> Any:
    path_text = str(model_path)
    if path_text not in _YOLO_MODEL_CACHE:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError("Missing dependency 'ultralytics'. Install with: pip install ultralytics") from exc
        _YOLO_MODEL_CACHE[path_text] = YOLO(path_text)
    return _YOLO_MODEL_CACHE[path_text]


def _label_matches(label: str, targets: set[str]) -> bool:
    label_tokens = set(label.replace("_", " ").replace("-", " ").split())
    for target in targets:
        target_tokens = set(target.replace("_", " ").replace("-", " ").split())
        if label == target or label in target or target in label:
            return True
        if label_tokens & target_tokens:
            return True
    return False


def _resize_mask_nearest(mask: Any, shape: Sequence[int]) -> Any:
    import numpy as np

    target_h, target_w = int(shape[0]), int(shape[1])
    if mask.shape[:2] == (target_h, target_w):
        return mask.astype(bool)
    src_h, src_w = mask.shape[:2]
    y_idx = np.clip((np.arange(target_h) * src_h / target_h).astype(np.int64), 0, src_h - 1)
    x_idx = np.clip((np.arange(target_w) * src_w / target_w).astype(np.int64), 0, src_w - 1)
    return mask[y_idx[:, None], x_idx[None, :]].astype(bool)


def _estimate_mask_object(
    *,
    label: str,
    frame: RGBDFrame,
    mask: Any,
    confidence: float,
    class_id: int | None,
    source_note: str,
) -> dict[str, Any] | None:
    import numpy as np

    valid_mask = (
        mask
        & np.isfinite(frame.depth_m)
        & (frame.depth_m > 0.05)
        & (frame.depth_m < 3.0)
    )
    ys, xs = np.where(valid_mask)
    if xs.size < 30:
        return None
    if xs.size > 12000:
        step = max(1, xs.size // 12000)
        xs = xs[::step]
        ys = ys[::step]
    zs = frame.depth_m[ys, xs]
    points = _deproject_pixels(xs, ys, zs, frame.intrinsics)
    center = np.nanmedian(points, axis=0)
    bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    return {
        "label": str(label),
        "class_id": int(class_id) if class_id is not None else None,
        "center_camera_m": [float(center[0]), float(center[1]), float(center[2])],
        "pixel_count": int(xs.size),
        "bbox_px": bbox,
        "confidence": round(float(confidence), 3),
        "notes": source_note,
    }


def _estimate_color_object(label: str, frame: RGBDFrame) -> dict[str, Any] | None:
    import numpy as np

    color_key = next((name for name in COLOR_THRESHOLDS if name in label), "")
    if not color_key:
        return None
    r0, r1, g0, g1, b0, b1 = COLOR_THRESHOLDS[color_key]
    rgb = frame.color
    depth = frame.depth_m
    mask = (
        (rgb[:, :, 0] >= r0)
        & (rgb[:, :, 0] <= r1)
        & (rgb[:, :, 1] >= g0)
        & (rgb[:, :, 1] <= g1)
        & (rgb[:, :, 2] >= b0)
        & (rgb[:, :, 2] <= b1)
        & np.isfinite(depth)
        & (depth > 0.05)
        & (depth < 2.5)
    )
    ys, xs = np.where(mask)
    if xs.size < 30:
        return None
    if xs.size > 6000:
        step = max(1, xs.size // 6000)
        xs = xs[::step]
        ys = ys[::step]
    zs = depth[ys, xs]
    valid = np.isfinite(zs) & (zs > 0.05)
    xs = xs[valid]
    ys = ys[valid]
    zs = zs[valid]
    if xs.size < 30:
        return None

    points = _deproject_pixels(xs, ys, zs, frame.intrinsics)
    center = np.nanmedian(points, axis=0)
    bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    confidence = min(0.95, 0.35 + math.log10(float(xs.size)) / 4.0)
    return {
        "label": label,
        "center_camera_m": [float(center[0]), float(center[1]), float(center[2])],
        "pixel_count": int(xs.size),
        "bbox_px": bbox,
        "confidence": round(float(confidence), 3),
    }


def _deproject_pixels(xs: Any, ys: Any, zs: Any, intrinsics: dict[str, float]) -> Any:
    import numpy as np

    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    ppx = float(intrinsics["ppx"])
    ppy = float(intrinsics["ppy"])
    x = (xs.astype(np.float64) - ppx) / fx * zs
    y = (ys.astype(np.float64) - ppy) / fy * zs
    return np.stack([x, y, zs.astype(np.float64)], axis=1)


def _rotvec_to_matrix(rotvec: Sequence[float]) -> list[list[float]]:
    rx, ry, rz = [float(v) for v in rotvec]
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z = rx / theta, ry / theta, rz / theta
    c = math.cos(theta)
    s = math.sin(theta)
    one_c = 1.0 - c
    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ]


def _matmul4(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    return [
        [sum(float(a[i][k]) * float(b[k][j]) for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def _invert_transform(matrix: Sequence[Sequence[float]] | None) -> list[list[float]] | None:
    if matrix is None:
        return None
    rot = [[float(matrix[i][j]) for j in range(3)] for i in range(3)]
    trans = [float(matrix[i][3]) for i in range(3)]
    rot_t = [[rot[j][i] for j in range(3)] for i in range(3)]
    inv_t = [-sum(rot_t[i][j] * trans[j] for j in range(3)) for i in range(3)]
    return [
        [rot_t[0][0], rot_t[0][1], rot_t[0][2], inv_t[0]],
        [rot_t[1][0], rot_t[1][1], rot_t[1][2], inv_t[1]],
        [rot_t[2][0], rot_t[2][1], rot_t[2][2], inv_t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _transform_point(
    matrix: Sequence[Sequence[float]] | None,
    point: Sequence[float] | None,
) -> list[float] | None:
    if matrix is None or point is None:
        return None
    x, y, z = [float(v) for v in point[:3]]
    return [
        float(matrix[0][0]) * x + float(matrix[0][1]) * y + float(matrix[0][2]) * z + float(matrix[0][3]),
        float(matrix[1][0]) * x + float(matrix[1][1]) * y + float(matrix[1][2]) * z + float(matrix[1][3]),
        float(matrix[2][0]) * x + float(matrix[2][1]) * y + float(matrix[2][2]) * z + float(matrix[2][3]),
    ]


def _meters_to_mm(values: Sequence[float] | None) -> list[float] | None:
    if values is None:
        return None
    return [float(v) * 1000.0 for v in values]


def _round_list(values: Iterable[float] | None, ndigits: int) -> list[float] | None:
    if values is None:
        return None
    return [round(float(v), ndigits) for v in values]


def _round_matrix(matrix: Sequence[Sequence[float]] | None, ndigits: int) -> list[list[float]] | None:
    if matrix is None:
        return None
    return [[round(float(v), ndigits) for v in row] for row in matrix]
