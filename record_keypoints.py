"""Record named TCP poses with wrist RGB snapshots.

This utility is for building a small keypoint database such as:

{
  "schema": "defense_ei_keypoint_database.v1",
  "records": [
    {
      "name": "tube_1_grasp_point",
      "tcp_pose": [x, y, z, rx, ry, rz],
      "wrist_img_id": "20260511_143012_123456.png"
    }
  ]
}
"""

from __future__ import annotations

import argparse
import json
import time
import uuid
from pathlib import Path
from typing import Any

import imageio.v2 as imageio

from evaluate_defense_agent_real import RealSenseRGBPair, _list_realsense_cameras
from ur7e_controller import ROBOT_IP, UR7eVectorController


PROJECT_ROOT = Path(__file__).resolve().parent


def _resolve_path(path_text: str | Path) -> Path:
    path = Path(str(path_text)).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def _load_database(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {
            "schema": "defense_ei_keypoint_database.v1",
            "records": [],
        }
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, list):
        return {
            "schema": "defense_ei_keypoint_database.v1",
            "records": payload,
        }
    if not isinstance(payload, dict):
        raise ValueError(f"Unsupported keypoint database format: {path}")
    records = payload.setdefault("records", [])
    if not isinstance(records, list):
        raise ValueError(f"Keypoint database 'records' must be a list: {path}")
    payload.setdefault("schema", "defense_ei_keypoint_database.v1")
    return payload


def _save_database(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp_path.replace(path)


def _new_image_id(*, random_name: bool) -> str:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    if random_name:
        return f"{stamp}_{uuid.uuid4().hex[:8]}.png"
    return f"{stamp}_{int((time.time() % 1) * 1000000):06d}.png"


def _append_record(
    *,
    database_path: Path,
    image_dir: Path,
    name: str,
    tcp_pose: list[float],
    wrist_rgb: Any,
    wrist_serial: str,
    random_image_name: bool,
) -> dict[str, Any]:
    image_dir.mkdir(parents=True, exist_ok=True)
    image_id = _new_image_id(random_name=random_image_name)
    image_path = image_dir / image_id
    imageio.imwrite(image_path, wrist_rgb)

    record = {
        "name": str(name).strip(),
        "tcp_pose": [round(float(v), 6) for v in tcp_pose],
        "wrist_img_id": image_id,
        "wrist_img_path": str(image_path),
        "wrist_camera_serial": wrist_serial,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    database = _load_database(database_path)
    database["records"].append(record)
    _save_database(database_path, database)
    return record


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record named TCP keypoints with wrist RGB images.")
    parser.add_argument("--list-cameras", action="store_true", help="List detected RealSense cameras and exit.")
    parser.add_argument("--robot-ip", default=ROBOT_IP)
    parser.add_argument("--robotiq-urscript-defs-path", default="")
    parser.add_argument("--strict-gripper-connection", action="store_true")
    parser.add_argument(
        "--camera-serials",
        default="405622072940,420222071502",
        help="Comma-separated RealSense serials: global,wrist. The second camera is saved as wrist_image.",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-warmup-frames", type=int, default=15)
    parser.add_argument("--camera-frame-timeout-ms", type=int, default=15000)
    parser.add_argument("--camera-capture-retries", type=int, default=3)
    parser.add_argument("--database", type=Path, default=Path("keypoint_database.json"))
    parser.add_argument("--image-dir", type=Path, default=Path("record_image"))
    parser.add_argument("--name", default="", help="Record one keypoint with this name, then exit.")
    parser.add_argument("--random-image-name", action="store_true", help="Add a random suffix to image filenames.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list_cameras:
        print(json.dumps({"realsense_cameras": _list_realsense_cameras()}, ensure_ascii=False, indent=2))
        return 0

    database_path = _resolve_path(args.database)
    image_dir = _resolve_path(args.image_dir)
    serials = [serial.strip() for serial in str(args.camera_serials).split(",") if serial.strip()]

    print(f"Keypoint database: {database_path}")
    print(f"Wrist images: {image_dir}")
    print("Input a keypoint name and press Enter to record. Empty input or 'q' exits.")

    with UR7eVectorController(
        robot_ip=str(args.robot_ip),
        robotiq_urscript_defs_path=str(args.robotiq_urscript_defs_path).strip() or None,
        strict_gripper_connection=bool(args.strict_gripper_connection),
    ) as controller, RealSenseRGBPair(
        serials=serials,
        width=int(args.camera_width),
        height=int(args.camera_height),
        fps=int(args.camera_fps),
        warmup_frames=int(args.camera_warmup_frames),
        frame_timeout_ms=int(args.camera_frame_timeout_ms),
        capture_retries=int(args.camera_capture_retries),
    ) as cameras:
        while True:
            if str(args.name).strip():
                keypoint_name = str(args.name).strip()
            else:
                keypoint_name = input("keypoint name> ").strip()
            if not keypoint_name or keypoint_name.lower() in {"q", "quit", "exit"}:
                break

            tcp_pose = controller.get_current_tcp_pose()
            _, wrist_frame = cameras.capture_rgbd()
            record = _append_record(
                database_path=database_path,
                image_dir=image_dir,
                name=keypoint_name,
                tcp_pose=tcp_pose,
                wrist_rgb=wrist_frame.color,
                wrist_serial=wrist_frame.serial,
                random_image_name=bool(args.random_image_name),
            )
            print(json.dumps(record, ensure_ascii=False, indent=2))

            if str(args.name).strip():
                break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
