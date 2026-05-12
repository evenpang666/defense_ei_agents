"""Generate four visual test outputs from the global RGB-D camera.

Outputs:
  1. global_depth_vis.png
  2. global_point_cloud_front.png
  3. global_rgb.png
  4. global_yolo.png

The script can capture a live Intel RealSense frame, or regenerate the outputs
from an existing log directory containing current_global_rgb.png,
current_global_rgb_depth_m.npy, and current_global_rgb_intrinsics.json.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Sequence

import imageio.v2 as imageio
import numpy as np

from scene_perception import RGBDFrame, annotate_yolo_image


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_YOLO_MODEL = PROJECT_ROOT / "checkpoints" / "yolo26n-seg.pt"
DEFAULT_LOG_ROOT = PROJECT_ROOT / "logs" / "global_camera_visual_test"
DEFAULT_GLOBAL_CAMERA_SERIAL = "405622072940"
POINT_CLOUD_MIN_DEPTH_M = 0.02
POINT_CLOUD_MAX_DEPTH_M = 6.0


def _log(message: str) -> None:
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{stamp}] [global_camera_visual_test] {message}", flush=True)


def _resolve_path(path_text: str | Path) -> Path:
    path = Path(str(path_text)).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def _save_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _parse_labels(text: str) -> list[str]:
    return [part.strip() for part in str(text or "").split(",") if part.strip()]


def _load_frame_from_log(input_dir: Path) -> RGBDFrame:
    rgb_path = input_dir / "current_global_rgb.png"
    depth_path = input_dir / "current_global_rgb_depth_m.npy"
    intrinsics_path = input_dir / "current_global_rgb_intrinsics.json"
    missing = [path for path in (rgb_path, depth_path, intrinsics_path) if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing required global RGB-D files: " + ", ".join(str(p) for p in missing))

    intr_payload = json.loads(intrinsics_path.read_text(encoding="utf-8"))
    intrinsics = intr_payload.get("intrinsics", intr_payload)
    return RGBDFrame(
        color=np.asarray(imageio.imread(rgb_path), dtype=np.uint8)[:, :, :3].copy(),
        depth_m=np.asarray(np.load(depth_path), dtype=np.float32),
        intrinsics={key: float(value) for key, value in intrinsics.items()},
        depth_scale=float(intr_payload.get("depth_scale", 0.001)),
        serial=str(intr_payload.get("serial", "")),
    )


class RealSenseGlobalCamera:
    PLACEHOLDER_SERIALS = {"GLOBAL_SERIAL", "FRONT_SERIAL", "SERIAL", "CAMERA_SERIAL"}

    def __init__(
        self,
        *,
        serial: str,
        width: int,
        height: int,
        fps: int,
        warmup_frames: int,
        frame_timeout_ms: int,
        capture_retries: int,
    ) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise ImportError("Missing dependency 'pyrealsense2'. Install Intel RealSense SDK bindings first.") from exc
        self._rs = rs
        self._serial = str(serial or "").strip()
        self._width = int(width)
        self._height = int(height)
        self._fps = int(fps)
        self._warmup_frames = max(0, int(warmup_frames))
        self._frame_timeout_ms = max(1000, int(frame_timeout_ms))
        self._capture_retries = max(1, int(capture_retries))
        self._pipeline: Any | None = None
        self._profile: Any | None = None
        self._align_to_color = rs.align(rs.stream.color)
        self._depth_filters: list[Any] = []
        self._active_serial = ""

    def __enter__(self) -> "RealSenseGlobalCamera":
        if self._serial.upper() in self.PLACEHOLDER_SERIALS or self._serial.upper().endswith("_SERIAL"):
            raise RuntimeError(f"Replace placeholder serial '{self._serial}' with a real RealSense serial.")
        serial = self._serial or self._discover_first_serial()
        if not serial:
            raise RuntimeError("No Intel RealSense camera was detected.")

        cfg = self._rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(self._rs.stream.color, self._width, self._height, self._rs.format.rgb8, self._fps)
        cfg.enable_stream(self._rs.stream.depth, self._width, self._height, self._rs.format.z16, self._fps)
        self._pipeline = self._rs.pipeline()
        self._profile = self._pipeline.start(cfg)
        self._active_serial = serial
        self._configure_depth_sensor(self._profile.get_device().first_depth_sensor())
        self._depth_filters = self._make_depth_filters()
        for _ in range(self._warmup_frames):
            self.capture()
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._pipeline = None
        self._profile = None
        self._depth_filters = []

    def _discover_first_serial(self) -> str:
        devices = list(self._rs.context().query_devices())
        if not devices:
            return ""
        return devices[0].get_info(self._rs.camera_info.serial_number)

    def _set_sensor_option(self, sensor: Any, option: Any, value: float, label: str) -> None:
        try:
            if hasattr(sensor, "supports") and not sensor.supports(option):
                return
            sensor.set_option(option, float(value))
        except Exception as exc:
            _log(f"RealSense depth option '{label}' was not applied: {exc}")

    def _set_filter_option(self, filter_obj: Any, option: Any, value: float) -> None:
        try:
            if hasattr(filter_obj, "supports") and not filter_obj.supports(option):
                return
            filter_obj.set_option(option, float(value))
        except Exception:
            pass

    def _configure_depth_sensor(self, depth_sensor: Any) -> None:
        option = self._rs.option
        self._set_sensor_option(depth_sensor, option.visual_preset, 4, "visual_preset=high_density")
        self._set_sensor_option(depth_sensor, option.emitter_enabled, 1, "emitter_enabled")
        self._set_sensor_option(depth_sensor, option.enable_auto_exposure, 1, "enable_auto_exposure")
        try:
            laser_range = depth_sensor.get_option_range(option.laser_power)
            self._set_sensor_option(depth_sensor, option.laser_power, laser_range.max, "laser_power=max")
        except Exception as exc:
            _log(f"RealSense laser power was not adjusted: {exc}")

    def _make_depth_filters(self) -> list[Any]:
        filters: list[Any] = []
        try:
            depth_to_disparity = self._rs.disparity_transform(True)
            spatial = self._rs.spatial_filter()
            temporal = self._rs.temporal_filter()
            disparity_to_depth = self._rs.disparity_transform(False)
            hole_filling = self._rs.hole_filling_filter()
            self._set_filter_option(spatial, self._rs.option.filter_magnitude, 2)
            self._set_filter_option(spatial, self._rs.option.filter_smooth_alpha, 0.5)
            self._set_filter_option(spatial, self._rs.option.filter_smooth_delta, 20)
            self._set_filter_option(spatial, self._rs.option.holes_fill, 3)
            self._set_filter_option(temporal, self._rs.option.filter_smooth_alpha, 0.4)
            self._set_filter_option(temporal, self._rs.option.filter_smooth_delta, 20)
            persistence_control = getattr(self._rs.option, "persistence_control", None)
            if persistence_control is not None:
                self._set_filter_option(temporal, persistence_control, 3)
            self._set_filter_option(hole_filling, self._rs.option.holes_fill, 1)
            filters.extend([depth_to_disparity, spatial, temporal, disparity_to_depth, hole_filling])
        except Exception as exc:
            _log(f"RealSense depth post-processing filters were not created: {exc}")
        return filters

    def _process_depth_frame(self, depth: Any) -> Any:
        processed = depth
        for depth_filter in self._depth_filters:
            try:
                processed = depth_filter.process(processed)
            except Exception as exc:
                _log(f"RealSense depth filter failed: {exc}")
                return depth
        return processed

    def _wait_for_frames_with_retry(self) -> Any:
        if self._pipeline is None:
            raise RuntimeError("RealSense pipeline is not started.")
        last_error: Exception | None = None
        for attempt in range(1, self._capture_retries + 1):
            try:
                return self._pipeline.wait_for_frames(self._frame_timeout_ms)
            except RuntimeError as exc:
                last_error = exc
                _log(
                    f"RealSense frame timeout for serial '{self._active_serial}' "
                    f"(attempt {attempt}/{self._capture_retries}, timeout {self._frame_timeout_ms} ms): {exc}"
                )
                time.sleep(min(0.5 * attempt, 2.0))
        raise RuntimeError(f"RealSense frame did not arrive for serial '{self._active_serial}'.") from last_error

    def capture(self) -> RGBDFrame:
        if self._profile is None:
            raise RuntimeError("RealSense camera is not started.")
        frames = self._align_to_color.process(self._wait_for_frames_with_retry())
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("Failed to capture aligned RGB-D frames.")
        depth = self._process_depth_frame(depth)

        intr = color.profile.as_video_stream_profile().intrinsics
        depth_sensor = self._profile.get_device().first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        depth_raw = np.asarray(depth.get_data(), dtype=np.uint16).copy()
        return RGBDFrame(
            color=np.asarray(color.get_data(), dtype=np.uint8).copy(),
            depth_m=depth_raw.astype(np.float32) * depth_scale,
            intrinsics={
                "width": float(intr.width),
                "height": float(intr.height),
                "fx": float(intr.fx),
                "fy": float(intr.fy),
                "ppx": float(intr.ppx),
                "ppy": float(intr.ppy),
            },
            depth_scale=depth_scale,
            serial=self._active_serial,
        )


def _colorize_depth(depth_m: np.ndarray) -> np.ndarray:
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    depth_u8 = np.zeros(depth_m.shape[:2], dtype=np.uint8)
    if valid.any():
        lo, hi = np.percentile(depth_m[valid], [2.0, 98.0])
        if hi <= lo:
            hi = lo + 1e-6
        scaled = (np.clip(depth_m, lo, hi) - lo) / (hi - lo)
        depth_u8 = np.nan_to_num(scaled * 255.0, nan=0.0, posinf=255.0, neginf=0.0).astype(np.uint8)
        depth_u8[~valid] = 0
    x = depth_u8.astype(np.float32) / 255.0
    r = np.clip(1.5 - np.abs(4.0 * x - 3.0), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4.0 * x - 2.0), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4.0 * x - 1.0), 0.0, 1.0)
    rgb = (np.stack([r, g, b], axis=2) * 255.0).astype(np.uint8)
    rgb[~valid] = np.array([0, 0, 0], dtype=np.uint8)
    return rgb


def _rgbd_to_point_cloud(frame: RGBDFrame, *, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    depth = np.asarray(frame.depth_m, dtype=np.float64)
    valid = np.isfinite(depth) & (depth >= POINT_CLOUD_MIN_DEPTH_M) & (depth <= POINT_CLOUD_MAX_DEPTH_M)
    ys, xs = np.where(valid)
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 3), dtype=np.uint8)
    if xs.size > int(max_points):
        rng = np.random.default_rng(0)
        keep = np.sort(rng.choice(xs.size, size=int(max_points), replace=False))
        xs = xs[keep]
        ys = ys[keep]
    zs = depth[ys, xs]
    intr = frame.intrinsics
    x = (xs.astype(np.float64) - float(intr["ppx"])) / float(intr["fx"]) * zs
    y = (ys.astype(np.float64) - float(intr["ppy"])) / float(intr["fy"]) * zs
    points = np.stack([x, y, zs], axis=1)
    colors = np.asarray(frame.color, dtype=np.uint8)[ys, xs, :3]
    return points, colors


def _save_point_cloud_view(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    canvas_size = 900
    canvas = np.full((canvas_size, canvas_size, 3), 18, dtype=np.uint8)
    if points.shape[0] > 0:
        a = points[:, 0]
        b = points[:, 1]
        finite = np.isfinite(a) & np.isfinite(b)
        a = a[finite]
        b = b[finite]
        rgb = colors[finite]
        if a.size > 0:
            a_lo, a_hi = np.percentile(a, [0.2, 99.8])
            b_lo, b_hi = np.percentile(b, [0.2, 99.8])
            if a_hi <= a_lo:
                a_hi = a_lo + 1e-6
            if b_hi <= b_lo:
                b_hi = b_lo + 1e-6
            u = np.clip(((a - a_lo) / (a_hi - a_lo) * (canvas_size - 1)).astype(np.int32), 0, canvas_size - 1)
            v = np.clip(((1.0 - (b - b_lo) / (b_hi - b_lo)) * (canvas_size - 1)).astype(np.int32), 0, canvas_size - 1)
            canvas[v, u] = rgb
    path.parent.mkdir(parents=True, exist_ok=True)
    imageio.imwrite(path, canvas)


def _depth_stats(depth_m: np.ndarray) -> dict[str, Any]:
    valid = np.isfinite(depth_m) & (depth_m >= POINT_CLOUD_MIN_DEPTH_M) & (depth_m <= POINT_CLOUD_MAX_DEPTH_M)
    values = depth_m[valid]
    out: dict[str, Any] = {
        "total_pixels": int(depth_m.size),
        "valid_pixels": int(valid.sum()),
        "valid_ratio": round(float(valid.sum()) / float(depth_m.size), 4) if depth_m.size else 0.0,
        "min_depth_m": POINT_CLOUD_MIN_DEPTH_M,
        "max_depth_m": POINT_CLOUD_MAX_DEPTH_M,
    }
    if values.size:
        out.update(
            {
                "depth_p01_m": round(float(np.percentile(values, 1.0)), 4),
                "depth_p50_m": round(float(np.percentile(values, 50.0)), 4),
                "depth_p99_m": round(float(np.percentile(values, 99.0)), 4),
            }
        )
    return out


def generate_outputs(
    *,
    frame: RGBDFrame,
    out_dir: Path,
    yolo_model: Path,
    yolo_conf: float,
    yolo_iou: float,
    yolo_target_labels: Sequence[str],
    yolo_device: str | None,
    max_points: int,
    source: str,
) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = out_dir / "global_rgb.png"
    depth_path = out_dir / "global_depth_vis.png"
    point_cloud_path = out_dir / "global_point_cloud_front.png"
    yolo_path = out_dir / "global_yolo.png"

    imageio.imwrite(rgb_path, frame.color)
    imageio.imwrite(depth_path, _colorize_depth(np.asarray(frame.depth_m, dtype=np.float32)))

    points, colors = _rgbd_to_point_cloud(frame, max_points=int(max_points))
    _save_point_cloud_view(point_cloud_path, points, colors)

    imageio.imwrite(
        yolo_path,
        annotate_yolo_image(
            frame=frame,
            model_path=str(yolo_model),
            confidence=float(yolo_conf),
            iou=float(yolo_iou),
            target_labels=yolo_target_labels,
            device=yolo_device,
        ),
    )

    summary = {
        "source": source,
        "serial": frame.serial,
        "intrinsics": frame.intrinsics,
        "depth": _depth_stats(np.asarray(frame.depth_m, dtype=np.float32)),
        "point_cloud": {
            "point_count": int(points.shape[0]),
            "frame": "global camera",
            "units": "meters",
            "view": "front projection, camera X/Y axes",
        },
        "yolo": {
            "model": str(yolo_model),
            "confidence": float(yolo_conf),
            "iou": float(yolo_iou),
            "target_labels": list(yolo_target_labels),
            "device": yolo_device,
        },
        "images": {
            "depth_vis": str(depth_path),
            "point_cloud_front": str(point_cloud_path),
            "rgb": str(rgb_path),
            "yolo": str(yolo_path),
        },
    }
    _save_json(out_dir / "summary.json", summary)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test global RGB-D depth/point-cloud and RGB/YOLO visualization outputs.")
    parser.add_argument("--input-dir", default="", help="Existing log directory with current_global_rgb* files. If omitted, captures live.")
    parser.add_argument("--out-dir", default="", help="Output directory. Default: logs/global_camera_visual_test/<timestamp>.")
    parser.add_argument(
        "--camera-serial",
        default=DEFAULT_GLOBAL_CAMERA_SERIAL,
        help=f"Global RealSense serial for live capture. Default: {DEFAULT_GLOBAL_CAMERA_SERIAL}.",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=15)
    parser.add_argument("--camera-warmup-frames", type=int, default=15)
    parser.add_argument("--camera-frame-timeout-ms", type=int, default=20000)
    parser.add_argument("--camera-capture-retries", type=int, default=5)
    parser.add_argument("--yolo-model", default=str(DEFAULT_YOLO_MODEL), help=f"YOLO model path. Default: {DEFAULT_YOLO_MODEL}.")
    parser.add_argument("--yolo-conf", type=float, default=0.35)
    parser.add_argument("--yolo-iou", type=float, default=0.5)
    parser.add_argument("--yolo-target-labels", default="", help="Comma-separated class labels. Empty means all YOLO detections.")
    parser.add_argument("--yolo-device", default="cpu", help="cpu, 0, cuda:0, or empty string for Ultralytics default.")
    parser.add_argument("--max-points", type=int, default=200000)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = _resolve_path(args.out_dir) if args.out_dir else DEFAULT_LOG_ROOT / timestamp
    yolo_model = _resolve_path(args.yolo_model)
    if not yolo_model.exists():
        raise FileNotFoundError(f"YOLO model not found: {yolo_model}")

    if args.input_dir:
        input_dir = _resolve_path(args.input_dir)
        _log(f"Loading global RGB-D frame from {input_dir}")
        frame = _load_frame_from_log(input_dir)
        source = str(input_dir)
    else:
        _log("Capturing one live global RGB-D frame")
        with RealSenseGlobalCamera(
            serial=str(args.camera_serial or ""),
            width=int(args.camera_width),
            height=int(args.camera_height),
            fps=int(args.camera_fps),
            warmup_frames=int(args.camera_warmup_frames),
            frame_timeout_ms=int(args.camera_frame_timeout_ms),
            capture_retries=int(args.camera_capture_retries),
        ) as camera:
            frame = camera.capture()
        source = f"live_realsense:{frame.serial or 'auto'}"

    summary = generate_outputs(
        frame=frame,
        out_dir=out_dir,
        yolo_model=yolo_model,
        yolo_conf=float(args.yolo_conf),
        yolo_iou=float(args.yolo_iou),
        yolo_target_labels=_parse_labels(args.yolo_target_labels),
        yolo_device=str(args.yolo_device or "").strip() or None,
        max_points=int(args.max_points),
        source=source,
    )
    _log(f"Wrote outputs to {out_dir}")
    print(json.dumps(summary["images"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ImportError, RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
