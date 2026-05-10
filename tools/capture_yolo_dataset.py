from __future__ import annotations

import argparse
import time
from pathlib import Path

import imageio.v3 as iio

from evaluate_defense_agent_real import RealSenseRGBPair


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="datasets/raw_yolo_capture")
    parser.add_argument("--camera-serials", default="")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--interval-sec", type=float, default=1.0)
    parser.add_argument("--count", type=int, default=300)
    parser.add_argument("--timeout-ms", type=int, default=20000)
    parser.add_argument("--retries", type=int, default=5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out)
    global_dir = out_dir / "global"
    wrist_dir = out_dir / "wrist"
    global_dir.mkdir(parents=True, exist_ok=True)
    wrist_dir.mkdir(parents=True, exist_ok=True)

    serials = [s.strip() for s in args.camera_serials.split(",") if s.strip()]
    serial_pair = tuple(serials[:2])

    with RealSenseRGBPair(
        serials=serial_pair,
        width=args.width,
        height=args.height,
        fps=args.fps,
        warmup_frames=args.warmup_frames,
        frame_timeout_ms=args.timeout_ms,
        capture_retries=args.retries,
    ) as cameras:
        for idx in range(args.count):
            global_frame, wrist_frame = cameras.capture_rgbd()
            stem = f"{idx:06d}"
            iio.imwrite(global_dir / f"{stem}_global.png", global_frame.color)
            iio.imwrite(wrist_dir / f"{stem}_wrist.png", wrist_frame.color)
            print(f"saved {stem}")
            time.sleep(args.interval_sec)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
