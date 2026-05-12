"""RGB-D scene-state helpers for the real UR7e workflow."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence


RGBD_MIN_DEPTH_M = 0.02
RGBD_MAX_DEPTH_M = 6.0


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


def gripper_to_wrist_camera_transform(
    offset_y_m: float = -0.04,
    offset_z_m: float = -0.09,
) -> list[list[float]]:
    """Approximate gripper->wrist-camera transform.

    The default follows the lab note that the wrist camera is mounted 4 cm
    along gripper -Y and 9 cm along gripper -Z. Rotation is identity until a
    proper hand-eye calibration is supplied.
    """
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, float(offset_y_m)],
        [0.0, 0.0, 1.0, float(offset_z_m)],
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
    yolo_device: str | None = "cpu",
) -> dict[str, Any]:
    """Build a structured 3D scene-state summary for coder and judger."""
    backend = str(perception_backend or "color").strip().lower()
    labels = _labels_from_text(
        task_text,
        extra_labels=list(yolo_target_labels or []),
    )
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
    wrist_objects: list[dict[str, Any]] = []
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
                    device=yolo_device,
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
            if backend == "yolo":
                raise RuntimeError(f"{backend.upper()} perception failed: {exc}") from exc
            estimates = []
            perception_error = str(exc)
        if backend == "yolo" and not estimates:
            target_text = ", ".join(yolo_target_labels or labels) or "any class"
            perception_error = (
                f"{backend.upper()} global perception produced zero detections. "
                f"Targets: {target_text}."
            )

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

        if backend == "yolo":
            missing_labels = list(yolo_target_labels or [])
        else:
            missing_labels = list(labels)
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

    if wrist_frame is not None and backend == "yolo":
        try:
            wrist_estimates = _estimate_yolo_objects(
                frame=wrist_frame,
                model_path=yolo_model_path,
                confidence=float(yolo_confidence),
                iou=float(yolo_iou),
                target_labels=yolo_target_labels or [],
                device=yolo_device,
            )
            for estimate in wrist_estimates:
                wrist_objects.append(
                    _wrist_object_record_from_estimate(
                        estimate=estimate,
                        t_gripper_wrist_camera=t_gripper_wrist_camera,
                        t_base_wrist_camera=t_base_wrist,
                        backend=backend,
                    )
                )
        except Exception as exc:
            perception_error = perception_error or f"wrist_yolo_failed: {exc}"

    return {
        "schema": "defense_ei_scene_state.v2",
        "status": "ok" if global_frame is not None else "unavailable",
        "coordinate_frames": {
            "base": "UR robot base frame.",
            "global_camera": "Fixed external RealSense camera frame; primary object coordinate frame.",
            "gripper": "Current UR TCP/gripper frame from RTDE.",
            "wrist_camera": "Camera mounted on gripper; default transform is approximate.",
        },
        "calibration": {
            "has_t_base_global_camera": t_base_global_camera is not None,
            "has_t_gripper_wrist_camera": t_gripper_wrist_camera is not None,
            "global_camera_is_primary_object_frame": True,
            "global_to_base_required_for_object_coordinates": False,
            "base_and_gripper_object_coordinates_are_optional_derived_fields": True,
        },
        "perception": {
            "backend": backend,
            "yolo_model_path": str(yolo_model_path) if yolo_model_path else None,
            "yolo_confidence": float(yolo_confidence),
            "yolo_iou": float(yolo_iou),
            "yolo_device": str(yolo_device) if yolo_device else None,
            "error": perception_error,
        },
        "tcp_pose_base": _round_list(tcp_pose_base, 6),
        "tcp": {
            "pose_base": _round_list(tcp_pose_base, 6),
            "position_base_m": _round_list(tcp_pose_base[:3], 6) if tcp_pose_base else None,
            "rotation_vector_base_rad": _round_list(tcp_pose_base[3:6], 6) if tcp_pose_base else None,
        },
        "t_base_gripper": _round_matrix(t_base_gripper, 6),
        "t_base_global_camera": _round_matrix(t_base_global_camera, 6),
        "t_gripper_wrist_camera": _round_matrix(t_gripper_wrist_camera, 6),
        "objects": objects,
        "wrist_objects": wrist_objects,
        "object_labels_requested": labels,
        "wrist_rgbd": {
            "available": wrist_frame is not None,
            "role": "near-field closed-loop alignment and verification.",
            "detected_object_count": len(wrist_objects),
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
                "bbox_px": obj.get("bbox_px"),
                "center_global_camera_m": obj.get("center_global_camera_m"),
                "grasp_region_center_global_camera_m": obj.get("grasp_region_center_global_camera_m"),
                "center_base_m": obj.get("center_base_m"),
                "center_gripper_mm": obj.get("center_gripper_mm"),
                "grasp_region_center_gripper_mm": obj.get("grasp_region_center_gripper_mm"),
                "point_cloud": obj.get("point_cloud"),
            }
        )
    wrist_objects = []
    for obj in scene_state.get("wrist_objects", [])[:max_objects]:
        wrist_objects.append(
            {
                "label": obj.get("label"),
                "status": obj.get("status"),
                "confidence": obj.get("confidence"),
                "bbox_px": obj.get("bbox_px"),
                "center_gripper_mm": obj.get("center_gripper_mm"),
                "grasp_region_center_gripper_mm": obj.get("grasp_region_center_gripper_mm"),
            }
        )
    return {
        "schema": scene_state.get("schema"),
        "status": scene_state.get("status"),
        "calibration": scene_state.get("calibration"),
        "perception": scene_state.get("perception"),
        "tcp": scene_state.get("tcp"),
        "objects": objects,
        "wrist_objects": wrist_objects,
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
    grasp_global, grasp_base = _estimate_grasp_point_from_point_cloud(
        estimate=estimate,
        t_base_global_camera=t_base_global_camera,
        center_global=center_global,
        center_base=center_base,
    )
    center_gripper = (
        _transform_point(t_gripper_base, center_base)
        if t_gripper_base is not None and center_base is not None
        else None
    )
    grasp_gripper = (
        _transform_point(t_gripper_base, grasp_base)
        if t_gripper_base is not None and grasp_base is not None
        else None
    )
    center_wrist = (
        _transform_point(t_wrist_base, center_base)
        if t_wrist_base is not None and center_base is not None
        else None
    )
    grasp_wrist = (
        _transform_point(t_wrist_base, grasp_base)
        if t_wrist_base is not None and grasp_base is not None
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
        "grasp_region_center_global_camera_m": _round_list(grasp_global, 5),
        "grasp_region_center_base_m": _round_list(grasp_base, 5),
        "grasp_region_center_gripper_mm": _round_list(_meters_to_mm(grasp_gripper), 1),
        "grasp_region_center_wrist_camera_mm": _round_list(_meters_to_mm(grasp_wrist), 1),
        "point_cloud": estimate.get("point_cloud_summary"),
        "notes": estimate.get(
            "notes",
            "Grasp region is estimated from the detected object's local RGB-D point cloud.",
        ),
    }


def _wrist_object_record_from_estimate(
    *,
    estimate: dict[str, Any],
    t_gripper_wrist_camera: Sequence[Sequence[float]] | None,
    t_base_wrist_camera: Sequence[Sequence[float]] | None,
    backend: str,
) -> dict[str, Any]:
    center_wrist = estimate["center_camera_m"]
    grasp_wrist, _ = _estimate_grasp_point_from_point_cloud(
        estimate=estimate,
        t_base_global_camera=None,
        center_global=center_wrist,
        center_base=None,
    )
    center_gripper = (
        _transform_point(t_gripper_wrist_camera, center_wrist)
        if t_gripper_wrist_camera is not None
        else None
    )
    grasp_gripper = (
        _transform_point(t_gripper_wrist_camera, grasp_wrist)
        if t_gripper_wrist_camera is not None and grasp_wrist is not None
        else center_gripper
    )
    center_base = (
        _transform_point(t_base_wrist_camera, center_wrist)
        if t_base_wrist_camera is not None
        else None
    )
    grasp_base = (
        _transform_point(t_base_wrist_camera, grasp_wrist)
        if t_base_wrist_camera is not None and grasp_wrist is not None
        else None
    )
    return {
        "label": estimate["label"],
        "source": f"wrist_rgbd_{backend}",
        "status": "detected",
        "confidence": estimate["confidence"],
        "pixel_count": estimate["pixel_count"],
        "bbox_px": estimate["bbox_px"],
        "class_id": estimate.get("class_id"),
        "center_wrist_camera_m": _round_list(center_wrist, 5),
        "center_wrist_camera_mm": _round_list(_meters_to_mm(center_wrist), 1),
        "center_gripper_mm": _round_list(_meters_to_mm(center_gripper), 1),
        "center_base_m": _round_list(center_base, 5),
        "grasp_region_center_wrist_camera_m": _round_list(grasp_wrist, 5),
        "grasp_region_center_wrist_camera_mm": _round_list(_meters_to_mm(grasp_wrist), 1),
        "grasp_region_center_gripper_mm": _round_list(_meters_to_mm(grasp_gripper), 1),
        "grasp_region_center_base_m": _round_list(grasp_base, 5),
        "point_cloud": estimate.get("point_cloud_summary"),
        "notes": estimate.get(
            "notes",
            "Wrist-camera grasp region estimated from local RGB-D point cloud.",
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
    device: str | None,
) -> list[dict[str, Any]]:
    import numpy as np

    if not model_path:
        raise RuntimeError("YOLO perception backend requires a model path or model name.")
    model = _load_yolo_model(model_path)
    results = model.predict(
        source=frame.color,
        conf=float(confidence),
        iou=float(iou),
        device=str(device) if device else None,
        verbose=False,
    )
    if not results:
        return []

    result = results[0]
    masks_obj = getattr(result, "masks", None)
    boxes_obj = getattr(result, "boxes", None)
    if masks_obj is None or getattr(masks_obj, "data", None) is None:
        return _estimate_yolo_box_objects(
            frame=frame,
            model=model,
            result=result,
            boxes_obj=boxes_obj,
            confidence=float(confidence),
            target_labels=target_labels,
        )

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


def _estimate_yolo_box_objects(
    *,
    frame: RGBDFrame,
    model: Any,
    result: Any,
    boxes_obj: Any,
    confidence: float,
    target_labels: Sequence[str],
) -> list[dict[str, Any]]:
    import numpy as np

    if boxes_obj is None:
        return []

    xyxy_raw = getattr(boxes_obj, "xyxy", None)
    if xyxy_raw is None:
        return []
    if hasattr(xyxy_raw, "detach"):
        boxes_np = xyxy_raw.detach().cpu().numpy()
    else:
        boxes_np = np.asarray(xyxy_raw)

    confs: list[float] = []
    class_ids: list[int] = []
    conf_raw = getattr(boxes_obj, "conf", None)
    cls_raw = getattr(boxes_obj, "cls", None)
    if conf_raw is not None:
        confs = [float(v) for v in conf_raw.detach().cpu().numpy().tolist()]
    if cls_raw is not None:
        class_ids = [int(v) for v in cls_raw.detach().cpu().numpy().tolist()]

    names = getattr(result, "names", None) or getattr(model, "names", {}) or {}
    target_set = {str(label).strip().lower() for label in target_labels if str(label).strip()}
    include_all = not target_set or target_set == {"foreground_object"}

    estimates: list[dict[str, Any]] = []
    for idx, raw_box in enumerate(boxes_np):
        if len(raw_box) < 4:
            continue
        class_id = class_ids[idx] if idx < len(class_ids) else -1
        label = str(names.get(class_id, class_id))
        label_l = label.lower()
        if not include_all and not _label_matches(label_l, target_set):
            continue
        conf = confs[idx] if idx < len(confs) else float(confidence)
        estimate = _estimate_box_object(
            label=label,
            frame=frame,
            box_xyxy=raw_box[:4],
            confidence=conf,
            class_id=class_id,
            source_note="YOLO detection box with robust RGB-D depth median.",
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


def ensure_yolo_model_available(model_path: str | Path | None) -> None:
    """Fail fast if the requested YOLO model cannot be loaded."""
    if not model_path:
        raise RuntimeError("YOLO perception backend requires --yolo-seg-model or a default YOLO model name.")
    _load_yolo_model(model_path)


def annotate_yolo_image(
    *,
    frame: RGBDFrame,
    model_path: str | Path | None,
    confidence: float,
    iou: float,
    target_labels: Sequence[str] | None = None,
    device: str | None = "cpu",
) -> Any:
    """Return an RGB image with YOLO masks, boxes, and class labels drawn on top."""
    import numpy as np

    if not model_path:
        raise RuntimeError("YOLO annotation requires a model path or model name.")
    model = _load_yolo_model(model_path)
    results = model.predict(
        source=frame.color,
        conf=float(confidence),
        iou=float(iou),
        device=str(device) if device else None,
        verbose=False,
    )
    annotated = np.asarray(frame.color).copy()
    if not results:
        _draw_image_banner(annotated, "NO YOLO DETECTIONS")
        return annotated

    result = results[0]
    boxes_obj = getattr(result, "boxes", None)
    if boxes_obj is None:
        _draw_image_banner(annotated, "NO YOLO DETECTIONS")
        return annotated
    xyxy_raw = getattr(boxes_obj, "xyxy", None)
    if xyxy_raw is None:
        _draw_image_banner(annotated, "NO YOLO DETECTIONS")
        return annotated
    boxes_np = xyxy_raw.detach().cpu().numpy() if hasattr(xyxy_raw, "detach") else np.asarray(xyxy_raw)

    confs: list[float] = []
    class_ids: list[int] = []
    conf_raw = getattr(boxes_obj, "conf", None)
    cls_raw = getattr(boxes_obj, "cls", None)
    if conf_raw is not None:
        confs = [float(v) for v in conf_raw.detach().cpu().numpy().tolist()]
    if cls_raw is not None:
        class_ids = [int(v) for v in cls_raw.detach().cpu().numpy().tolist()]

    names = getattr(result, "names", None) or getattr(model, "names", {}) or {}
    target_set = {str(label).strip().lower() for label in target_labels or [] if str(label).strip()}
    include_all = not target_set or target_set == {"foreground_object"}
    masks_np = None
    masks_obj = getattr(result, "masks", None)
    mask_data = getattr(masks_obj, "data", None) if masks_obj is not None else None
    if mask_data is not None:
        masks_np = mask_data.detach().cpu().numpy() if hasattr(mask_data, "detach") else np.asarray(mask_data)
    drawn = 0
    for idx, raw_box in enumerate(boxes_np):
        if len(raw_box) < 4:
            continue
        class_id = class_ids[idx] if idx < len(class_ids) else -1
        label = str(names.get(class_id, class_id))
        if not include_all and not _label_matches(label.lower(), target_set):
            continue
        conf = confs[idx] if idx < len(confs) else float(confidence)
        text = f"{label} {conf:.2f}"
        if masks_np is not None and idx < len(masks_np):
            mask = _resize_mask_nearest(np.asarray(masks_np[idx]) > 0.5, annotated.shape[:2])
            _overlay_mask(annotated, mask, _mask_color(idx))
        _draw_box_label(annotated, raw_box[:4], text)
        drawn += 1
    if drawn == 0:
        _draw_image_banner(annotated, "NO YOLO DETECTIONS")
    return annotated


def _mask_color(index: int) -> tuple[int, int, int]:
    palette = (
        (255, 64, 64),
        (64, 180, 255),
        (80, 220, 120),
        (255, 190, 64),
        (190, 96, 255),
        (255, 96, 190),
        (80, 220, 220),
    )
    return palette[int(index) % len(palette)]


def _overlay_mask(image: Any, mask: Any, color: Sequence[int], *, alpha: float = 0.38) -> None:
    import numpy as np

    mask_np = np.asarray(mask, dtype=bool)
    if mask_np.shape[:2] != image.shape[:2] or not mask_np.any():
        return
    color_np = np.asarray(color, dtype=np.float32)
    base = image[mask_np].astype(np.float32)
    blended = base * (1.0 - float(alpha)) + color_np * float(alpha)
    image[mask_np] = np.clip(blended, 0, 255).astype(image.dtype)


def _draw_image_banner(image: Any, text: str) -> None:
    import numpy as np

    try:
        from PIL import Image, ImageDraw, ImageFont

        h, w = image.shape[:2]
        pil_image = Image.fromarray(image)
        draw = ImageDraw.Draw(pil_image)
        font = ImageFont.load_default()
        text_bbox = draw.textbbox((0, 0), text, font=font)
        text_w = text_bbox[2] - text_bbox[0]
        text_h = text_bbox[3] - text_bbox[1]
        draw.rectangle((0, 0, min(w - 1, text_w + 12), text_h + 10), fill=(255, 40, 40))
        draw.text((6, 5), text, fill=(255, 255, 255), font=font)
        image[:, :, :] = np.asarray(pil_image)
    except Exception:
        image[: min(18, image.shape[0]), : min(180, image.shape[1])] = np.array([255, 40, 40], dtype=image.dtype)


def _draw_box_label(image: Any, box_xyxy: Sequence[float], text: str) -> None:
    import numpy as np

    h, w = image.shape[:2]
    x0, y0, x1, y1 = [int(round(float(v))) for v in box_xyxy]
    left = max(0, min(w - 1, min(x0, x1)))
    right = max(0, min(w - 1, max(x0, x1)))
    top = max(0, min(h - 1, min(y0, y1)))
    bottom = max(0, min(h - 1, max(y0, y1)))
    if right <= left or bottom <= top:
        return

    try:
        from PIL import Image, ImageDraw, ImageFont

        pil_image = Image.fromarray(image)
        draw = ImageDraw.Draw(pil_image)
        font = ImageFont.load_default()
        draw.rectangle((left, top, right, bottom), outline=(255, 40, 40), width=3)
        text_bbox = draw.textbbox((0, 0), text, font=font)
        text_w = text_bbox[2] - text_bbox[0]
        text_h = text_bbox[3] - text_bbox[1]
        label_top = max(0, top - text_h - 6)
        draw.rectangle((left, label_top, min(w - 1, left + text_w + 8), label_top + text_h + 6), fill=(255, 40, 40))
        draw.text((left + 4, label_top + 3), text, fill=(255, 255, 255), font=font)
        image[:, :, :] = np.asarray(pil_image)
        return
    except Exception:
        pass

    color = np.array([255, 40, 40], dtype=image.dtype)
    fill = np.array([255, 255, 255], dtype=image.dtype)
    thickness = 3
    image[top : min(h, top + thickness), left : right + 1] = color
    image[max(0, bottom - thickness + 1) : bottom + 1, left : right + 1] = color
    image[top : bottom + 1, left : min(w, left + thickness)] = color
    image[top : bottom + 1, max(0, right - thickness + 1) : right + 1] = color

    char_w = 6
    char_h = 9
    pad = 3
    label_w = min(w - left, max(1, len(text) * char_w + pad * 2))
    label_h = min(h - top, char_h + pad * 2)
    label_top = max(0, top - label_h) if top >= label_h else top
    image[label_top : label_top + label_h, left : left + label_w] = color
    _draw_ascii_text(image, left + pad, label_top + pad, text, fill)


def _draw_ascii_text(image: Any, x: int, y: int, text: str, color: Any) -> None:
    font = {
        "0": ("111", "101", "101", "101", "111"),
        "1": ("010", "110", "010", "010", "111"),
        "2": ("111", "001", "111", "100", "111"),
        "3": ("111", "001", "111", "001", "111"),
        "4": ("101", "101", "111", "001", "001"),
        "5": ("111", "100", "111", "001", "111"),
        "6": ("111", "100", "111", "101", "111"),
        "7": ("111", "001", "010", "010", "010"),
        "8": ("111", "101", "111", "101", "111"),
        "9": ("111", "101", "111", "001", "111"),
        ".": ("0", "0", "0", "0", "1"),
    }
    h, w = image.shape[:2]
    cursor = int(x)
    for ch in str(text)[:32].upper():
        glyph = font.get(ch, ("111", "100", "110", "100", "111") if ch != " " else ("0", "0", "0", "0", "0"))
        for gy, row in enumerate(glyph):
            for gx, bit in enumerate(row):
                if bit == "1":
                    yy = y + gy
                    xx = cursor + gx
                    if 0 <= yy < h and 0 <= xx < w:
                        image[yy, xx] = color
        cursor += len(glyph[0]) + 1
        if cursor >= w:
            break


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
        & (frame.depth_m >= RGBD_MIN_DEPTH_M)
        & (frame.depth_m <= RGBD_MAX_DEPTH_M)
    )
    ys, xs = np.where(valid_mask)
    if xs.size < 30:
        return None
    if xs.size > 12000:
        step = max(1, xs.size // 12000)
        xs = xs[::step]
        ys = ys[::step]
    zs = frame.depth_m[ys, xs]
    points = _filter_object_points(_deproject_pixels(xs, ys, zs, frame.intrinsics))
    if points.shape[0] < 30:
        return None
    center = np.nanmedian(points, axis=0)
    bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    return {
        "label": str(label),
        "class_id": int(class_id) if class_id is not None else None,
        "center_camera_m": [float(center[0]), float(center[1]), float(center[2])],
        "point_cloud_camera_m": points.tolist(),
        "point_cloud_summary": _point_cloud_summary(points, method="yolo_mask_local_point_cloud"),
        "pixel_count": int(points.shape[0]),
        "bbox_px": bbox,
        "confidence": round(float(confidence), 3),
        "notes": source_note,
    }


def _estimate_box_object(
    *,
    label: str,
    frame: RGBDFrame,
    box_xyxy: Sequence[float],
    confidence: float,
    class_id: int | None,
    source_note: str,
) -> dict[str, Any] | None:
    import numpy as np

    h, w = frame.depth_m.shape[:2]
    x0, y0, x1, y1 = [float(v) for v in box_xyxy]
    left = max(0, min(w - 1, int(math.floor(min(x0, x1)))))
    right = max(0, min(w, int(math.ceil(max(x0, x1)))))
    top = max(0, min(h - 1, int(math.floor(min(y0, y1)))))
    bottom = max(0, min(h, int(math.ceil(max(y0, y1)))))
    if right <= left or bottom <= top:
        return None

    box_w = right - left
    box_h = bottom - top
    # Use the center of the box to reduce background depth when boxes are loose.
    inset_x = int(round(box_w * 0.2))
    inset_y = int(round(box_h * 0.2))
    inner_left = min(right - 1, left + inset_x)
    inner_right = max(inner_left + 1, right - inset_x)
    inner_top = min(bottom - 1, top + inset_y)
    inner_bottom = max(inner_top + 1, bottom - inset_y)

    depth_patch = frame.depth_m[inner_top:inner_bottom, inner_left:inner_right]
    valid_mask = np.isfinite(depth_patch) & (depth_patch >= RGBD_MIN_DEPTH_M) & (depth_patch <= RGBD_MAX_DEPTH_M)
    ys_rel, xs_rel = np.where(valid_mask)
    if xs_rel.size < 30:
        depth_patch = frame.depth_m[top:bottom, left:right]
        valid_mask = np.isfinite(depth_patch) & (depth_patch >= RGBD_MIN_DEPTH_M) & (depth_patch <= RGBD_MAX_DEPTH_M)
        ys_rel, xs_rel = np.where(valid_mask)
        if xs_rel.size < 30:
            return None
        xs = xs_rel + left
        ys = ys_rel + top
    else:
        xs = xs_rel + inner_left
        ys = ys_rel + inner_top

    if xs.size > 12000:
        step = max(1, xs.size // 12000)
        xs = xs[::step]
        ys = ys[::step]
    zs = frame.depth_m[ys, xs]
    points = _filter_object_points(_deproject_pixels(xs, ys, zs, frame.intrinsics))
    if points.shape[0] < 30:
        return None
    center = np.nanmedian(points, axis=0)
    return {
        "label": str(label),
        "class_id": int(class_id) if class_id is not None else None,
        "center_camera_m": [float(center[0]), float(center[1]), float(center[2])],
        "point_cloud_camera_m": points.tolist(),
        "point_cloud_summary": _point_cloud_summary(points, method="yolo_box_local_point_cloud"),
        "pixel_count": int(points.shape[0]),
        "bbox_px": [int(left), int(top), int(right), int(bottom)],
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
        & (depth >= RGBD_MIN_DEPTH_M)
        & (depth <= RGBD_MAX_DEPTH_M)
    )
    ys, xs = np.where(mask)
    if xs.size < 30:
        return None
    if xs.size > 6000:
        step = max(1, xs.size // 6000)
        xs = xs[::step]
        ys = ys[::step]
    zs = depth[ys, xs]
    valid = np.isfinite(zs) & (zs >= RGBD_MIN_DEPTH_M) & (zs <= RGBD_MAX_DEPTH_M)
    xs = xs[valid]
    ys = ys[valid]
    zs = zs[valid]
    if xs.size < 30:
        return None

    points = _filter_object_points(_deproject_pixels(xs, ys, zs, frame.intrinsics))
    if points.shape[0] < 30:
        return None
    center = np.nanmedian(points, axis=0)
    bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    confidence = min(0.95, 0.35 + math.log10(float(xs.size)) / 4.0)
    return {
        "label": label,
        "center_camera_m": [float(center[0]), float(center[1]), float(center[2])],
        "point_cloud_camera_m": points.tolist(),
        "point_cloud_summary": _point_cloud_summary(points, method="color_mask_local_point_cloud"),
        "pixel_count": int(points.shape[0]),
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


def _filter_object_points(points: Any) -> Any:
    import numpy as np

    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        return np.zeros((0, 3), dtype=np.float64)
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if pts.shape[0] < 30:
        return pts

    median = np.median(pts, axis=0)
    mad = np.median(np.abs(pts - median), axis=0)
    scale = np.maximum(1.4826 * mad, np.array([0.015, 0.015, 0.02], dtype=np.float64))
    keep = np.all(np.abs(pts - median) <= 4.5 * scale, axis=1)
    filtered = pts[keep]
    return filtered if filtered.shape[0] >= 30 else pts


def _point_cloud_summary(points: Any, *, method: str) -> dict[str, Any]:
    import numpy as np

    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return {"method": method, "point_count": 0}
    mins = np.min(pts, axis=0)
    maxs = np.max(pts, axis=0)
    extent = maxs - mins
    return {
        "method": method,
        "point_count": int(pts.shape[0]),
        "bounds_camera_m": {
            "min": _round_list(mins.tolist(), 5),
            "max": _round_list(maxs.tolist(), 5),
            "extent": _round_list(extent.tolist(), 5),
        },
    }


def _estimate_grasp_point_from_point_cloud(
    *,
    estimate: dict[str, Any],
    t_base_global_camera: Sequence[Sequence[float]] | None,
    center_global: Sequence[float],
    center_base: Sequence[float] | None,
) -> tuple[list[float] | None, list[float] | None]:
    import numpy as np

    raw_points = estimate.get("point_cloud_camera_m")
    if not raw_points:
        return list(center_global), list(center_base) if center_base is not None else None
    points_global = np.asarray(raw_points, dtype=np.float64)
    if points_global.ndim != 2 or points_global.shape[1] != 3 or points_global.shape[0] < 30:
        return list(center_global), list(center_base) if center_base is not None else None

    if t_base_global_camera is not None:
        points_base = _transform_points(t_base_global_camera, points_global)
        z_cut = np.percentile(points_base[:, 2], 80.0)
        top_points = points_base[points_base[:, 2] >= z_cut]
        if top_points.shape[0] < 10:
            top_points = points_base
        grasp_base_np = np.median(top_points, axis=0)
        grasp_global_np = _transform_points(_invert_transform(t_base_global_camera), grasp_base_np[None, :])[0]
        estimate["point_cloud_summary"] = {
            **dict(estimate.get("point_cloud_summary") or {}),
            "grasp_point_method": "top_surface_percentile_in_base_frame",
        }
        return grasp_global_np.tolist(), grasp_base_np.tolist()

    z_cut = np.percentile(points_global[:, 2], 20.0)
    visible_points = points_global[points_global[:, 2] <= z_cut]
    if visible_points.shape[0] < 10:
        visible_points = points_global
    grasp_global_np = np.median(visible_points, axis=0)
    estimate["point_cloud_summary"] = {
        **dict(estimate.get("point_cloud_summary") or {}),
        "grasp_point_method": "visible_surface_percentile_in_camera_frame",
    }
    return grasp_global_np.tolist(), None


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


def _transform_points(matrix: Sequence[Sequence[float]] | None, points: Any) -> Any:
    import numpy as np

    if matrix is None:
        return None
    pts = np.asarray(points, dtype=np.float64)
    rot = np.asarray([[float(matrix[i][j]) for j in range(3)] for i in range(3)], dtype=np.float64)
    trans = np.asarray([float(matrix[i][3]) for i in range(3)], dtype=np.float64)
    return pts @ rot.T + trans


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
