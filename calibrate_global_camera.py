"""Calibrate the fixed global RealSense camera to the UR base frame.

The script collects 3D-3D correspondences:
    camera point from aligned RealSense RGB-D
    base point from UR RTDE current TCP position

It then solves the rigid transform T_base_global_camera and writes the JSON
file consumed by evaluate_defense_agent_real.py.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Sequence

import imageio.v2 as imageio
import numpy as np

from ur7e_controller import ROBOT_IP, UR7eVectorController


PLACEHOLDER_SERIALS = {
    "GLOBAL_SERIAL",
    "FRONT_SERIAL",
    "SERIAL",
    "CAMERA_SERIAL",
}


def _log(message: str) -> None:
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{stamp}] [calibrate_global_camera] {message}", flush=True)


def _resolve_path(path_text: str | Path) -> Path:
    path = Path(str(path_text)).expanduser()
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    return path


def _save_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _list_realsense_cameras() -> list[dict[str, str]]:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise ImportError("Missing dependency 'pyrealsense2'.") from exc

    cameras: list[dict[str, str]] = []
    for dev in list(rs.context().query_devices()):
        entry: dict[str, str] = {}
        for key, label in (
            (rs.camera_info.name, "name"),
            (rs.camera_info.serial_number, "serial"),
            (rs.camera_info.firmware_version, "firmware"),
            (rs.camera_info.usb_type_descriptor, "usb"),
        ):
            try:
                entry[label] = dev.get_info(key)
            except Exception:
                pass
        cameras.append(entry)
    return cameras


class RealSenseDepthCamera:
    def __init__(self, serial: str, width: int, height: int, fps: int, warmup_frames: int) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise ImportError("Missing dependency 'pyrealsense2'.") from exc

        self._rs = rs
        self._serial = str(serial or "").strip()
        self._width = int(width)
        self._height = int(height)
        self._fps = int(fps)
        self._warmup_frames = max(0, int(warmup_frames))
        self._pipeline: Any | None = None
        self._profile: Any | None = None
        self._align = rs.align(rs.stream.color)

    def __enter__(self) -> "RealSenseDepthCamera":
        if self._serial.upper() in PLACEHOLDER_SERIALS or self._serial.upper().endswith("_SERIAL"):
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

    def _discover_first_serial(self) -> str:
        devices = list(self._rs.context().query_devices())
        if not devices:
            return ""
        return devices[0].get_info(self._rs.camera_info.serial_number)

    def capture(self) -> tuple[np.ndarray, np.ndarray, dict[str, float], float]:
        if self._pipeline is None or self._profile is None:
            raise RuntimeError("RealSense camera is not started.")
        frames = self._align.process(self._pipeline.wait_for_frames())
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("Failed to capture aligned RGB-D frames.")

        intr = color.profile.as_video_stream_profile().intrinsics
        depth_sensor = self._profile.get_device().first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        color_np = np.asarray(color.get_data(), dtype=np.uint8).copy()
        depth_np = np.asarray(depth.get_data(), dtype=np.uint16).astype(np.float32) * depth_scale
        intrinsics = {
            "width": float(intr.width),
            "height": float(intr.height),
            "fx": float(intr.fx),
            "fy": float(intr.fy),
            "ppx": float(intr.ppx),
            "ppy": float(intr.ppy),
        }
        return color_np, depth_np, intrinsics, depth_scale


def _deproject_pixel(u: float, v: float, depth_m: float, intrinsics: dict[str, float]) -> list[float]:
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    ppx = float(intrinsics["ppx"])
    ppy = float(intrinsics["ppy"])
    z = float(depth_m)
    x = (float(u) - ppx) / fx * z
    y = (float(v) - ppy) / fy * z
    return [x, y, z]


def _robust_depth_at(depth_m: np.ndarray, u: int, v: int, radius: int) -> float:
    h, w = depth_m.shape[:2]
    x0 = max(0, int(u) - int(radius))
    x1 = min(w, int(u) + int(radius) + 1)
    y0 = max(0, int(v) - int(radius))
    y1 = min(h, int(v) + int(radius) + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 3.0)]
    if valid.size == 0:
        raise RuntimeError(f"No valid depth around pixel ({u}, {v}) with radius {radius}.")
    return float(np.median(valid))


def _prompt_pixel(image_path: Path) -> tuple[int, int]:
    print(f"Open this image and choose the calibration point pixel: {image_path}")
    while True:
        raw = input("Enter pixel as 'u v' or 'u,v': ").strip()
        parts = raw.replace(",", " ").split()
        if len(parts) != 2:
            print("Expected two numbers, for example: 321 244")
            continue
        try:
            return int(round(float(parts[0]))), int(round(float(parts[1])))
        except ValueError:
            print("Could not parse pixel coordinates.")


def _maybe_click_pixel(image_path: Path) -> tuple[int, int] | None:
    try:
        import cv2
    except Exception:
        return None

    image = cv2.imread(str(image_path))
    if image is None:
        return None
    selected: list[tuple[int, int]] = []

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            selected.clear()
            selected.append((int(x), int(y)))

    window = "click calibration point; press Enter when selected, Esc to type manually"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    while True:
        shown = image.copy()
        if selected:
            cv2.drawMarker(shown, selected[0], (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
        cv2.imshow(window, shown)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 10) and selected:
            cv2.destroyWindow(window)
            return selected[0]
        if key == 27:
            cv2.destroyWindow(window)
            return None


def _solve_rigid_transform(
    camera_points: np.ndarray,
    base_points: np.ndarray,
) -> tuple[np.ndarray, float, list[float]]:
    if camera_points.shape != base_points.shape or camera_points.shape[1] != 3:
        raise ValueError("camera_points and base_points must both be Nx3 arrays")
    if camera_points.shape[0] < 3:
        raise ValueError("At least 3 point correspondences are required.")

    camera_centroid = camera_points.mean(axis=0)
    base_centroid = base_points.mean(axis=0)
    camera_centered = camera_points - camera_centroid
    base_centered = base_points - base_centroid
    h = camera_centered.T @ base_centered
    u, _s, vt = np.linalg.svd(h)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T
    trans = base_centroid - rot @ camera_centroid

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rot
    transform[:3, 3] = trans

    predicted = (rot @ camera_points.T).T + trans
    residuals = np.linalg.norm(predicted - base_points, axis=1)
    return transform, float(np.sqrt(np.mean(residuals * residuals))), [float(v) for v in residuals]


def _load_correspondences(path: Path) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    points = payload.get("points", payload.get("correspondences", []))
    if not isinstance(points, list):
        raise ValueError("correspondence file must contain a points list")
    camera_points: list[list[float]] = []
    base_points: list[list[float]] = []
    clean_points: list[dict[str, Any]] = []
    for item in points:
        if not isinstance(item, dict):
            continue
        camera = item.get("camera_point_m", item.get("global_camera_m"))
        base = item.get("base_point_m", item.get("base_m"))
        if not isinstance(camera, list) or not isinstance(base, list) or len(camera) != 3 or len(base) != 3:
            continue
        camera_vec = [float(v) for v in camera]
        base_vec = [float(v) for v in base]
        camera_points.append(camera_vec)
        base_points.append(base_vec)
        clean_points.append({**item, "camera_point_m": camera_vec, "base_point_m": base_vec})
    return np.asarray(camera_points, dtype=np.float64), np.asarray(base_points, dtype=np.float64), clean_points


def _write_calibration(
    *,
    output_path: Path,
    transform: np.ndarray,
    rmse_m: float,
    residuals_m: Sequence[float],
    points: list[dict[str, Any]],
    metadata: dict[str, Any],
) -> None:
    payload = {
        "schema": "defense_ei_agents.T_base_global_camera.v1",
        "description": "Homogeneous transform from fixed global RealSense camera frame to UR base frame.",
        "usage": "P_base = T_base_global_camera * P_global_camera",
        "matrix": [[round(float(v), 9) for v in row] for row in transform.tolist()],
        "rmse_m": round(float(rmse_m), 6),
        "residuals_m": [round(float(v), 6) for v in residuals_m],
        "point_count": len(points),
        "points": points,
        "metadata": metadata,
    }
    _save_json(output_path, payload)


def collect_interactive(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]], dict[str, Any]]:
    sample_dir = _resolve_path(args.sample_dir)
    sample_dir.mkdir(parents=True, exist_ok=True)
    points: list[dict[str, Any]] = []
    camera_points: list[list[float]] = []
    base_points: list[list[float]] = []

    with RealSenseDepthCamera(
        serial=str(args.camera_serial or ""),
        width=int(args.camera_width),
        height=int(args.camera_height),
        fps=int(args.camera_fps),
        warmup_frames=int(args.camera_warmup_frames),
    ) as camera, UR7eVectorController(
        robot_ip=str(args.robot_ip),
        strict_gripper_connection=False,
    ) as controller:
        _log("Connected camera and robot. Use Freedrive/teach pendant to place TCP on each calibration point.")
        _log("For each point, the script captures RGB-D, asks for the pixel, and reads current TCP xyz as base point.")
        for idx in range(1, int(args.num_points) + 1):
            input(f"\nMove TCP to calibration point #{idx}, then press Enter to capture.")
            color, depth_m, intrinsics, depth_scale = camera.capture()
            image_path = sample_dir / f"calib_point_{idx:02d}_rgb.png"
            depth_path = sample_dir / f"calib_point_{idx:02d}_depth_m.npy"
            imageio.imwrite(image_path, color)
            np.save(depth_path, depth_m)

            pixel = None if args.no_click else _maybe_click_pixel(image_path)
            if pixel is None:
                pixel = _prompt_pixel(image_path)
            u_px, v_px = pixel
            depth_value_m = _robust_depth_at(depth_m, u_px, v_px, int(args.depth_radius))
            camera_point_m = _deproject_pixel(u_px, v_px, depth_value_m, intrinsics)
            tcp_pose = controller.get_current_tcp_pose()
            base_point_m = [float(v) for v in tcp_pose[:3]]

            point_record = {
                "index": idx,
                "pixel_uv": [int(u_px), int(v_px)],
                "depth_m": round(depth_value_m, 6),
                "camera_point_m": [round(float(v), 6) for v in camera_point_m],
                "base_point_m": [round(float(v), 6) for v in base_point_m],
                "tcp_pose_base": [round(float(v), 6) for v in tcp_pose],
                "rgb_image": str(image_path),
                "depth_m_npy": str(depth_path),
            }
            points.append(point_record)
            camera_points.append(camera_point_m)
            base_points.append(base_point_m)
            _log(
                f"Point #{idx}: pixel={point_record['pixel_uv']} "
                f"camera={point_record['camera_point_m']} base={point_record['base_point_m']}"
            )

    metadata = {
        "camera_serial": str(args.camera_serial or "auto"),
        "camera_width": int(args.camera_width),
        "camera_height": int(args.camera_height),
        "camera_fps": int(args.camera_fps),
        "depth_radius_px": int(args.depth_radius),
        "sample_dir": str(sample_dir),
    }
    return (
        np.asarray(camera_points, dtype=np.float64),
        np.asarray(base_points, dtype=np.float64),
        points,
        metadata,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate T_base_global_camera for defense_ei_agents.")
    parser.add_argument("--list-cameras", action="store_true", help="List detected RealSense cameras and exit.")
    parser.add_argument("--from-correspondences", type=Path, default=None, help="Fit from an existing correspondence JSON file.")
    parser.add_argument("--output", type=Path, default=Path("calibration/t_base_global_camera.json"))
    parser.add_argument("--sample-dir", type=Path, default=Path("calibration/samples"))
    parser.add_argument("--num-points", type=int, default=8)
    parser.add_argument("--robot-ip", default=ROBOT_IP)
    parser.add_argument("--camera-serial", default="", help="Global RealSense serial. If omitted, uses the first detected camera.")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-warmup-frames", type=int, default=15)
    parser.add_argument("--depth-radius", type=int, default=3, help="Median depth radius around selected pixel.")
    parser.add_argument("--no-click", action="store_true", help="Disable OpenCV click UI and type pixels manually.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list_cameras:
        print(json.dumps({"realsense_cameras": _list_realsense_cameras()}, ensure_ascii=False, indent=2))
        return 0

    output_path = _resolve_path(args.output)
    if args.from_correspondences:
        corr_path = _resolve_path(args.from_correspondences)
        camera_points, base_points, points = _load_correspondences(corr_path)
        metadata = {"source_correspondences": str(corr_path)}
    else:
        camera_points, base_points, points, metadata = collect_interactive(args)

    transform, rmse_m, residuals_m = _solve_rigid_transform(camera_points, base_points)
    _write_calibration(
        output_path=output_path,
        transform=transform,
        rmse_m=rmse_m,
        residuals_m=residuals_m,
        points=points,
        metadata=metadata,
    )

    _log(f"Wrote {output_path}")
    _log(f"Calibration RMSE: {rmse_m * 1000.0:.1f} mm")
    if rmse_m > 0.02:
        _log("Warning: RMSE is above 20 mm. Recheck clicked pixels, TCP contact points, and depth quality.")
    print(json.dumps({"output": str(output_path), "rmse_m": rmse_m, "point_count": len(points)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ImportError, RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
