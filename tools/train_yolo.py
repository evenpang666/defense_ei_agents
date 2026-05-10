from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path

from ultralytics import YOLO


DEFAULT_CHECKPOINT_DIR = Path("checkpoints")


def resolve_model_path(model_name_or_path: str) -> str:
    model_text = str(model_name_or_path).strip()
    model_path = Path(model_text).expanduser()
    if model_path.is_absolute() or model_path.parent != Path("."):
        return str(model_path)

    DEFAULT_CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)
    target = DEFAULT_CHECKPOINT_DIR / model_path.name
    if target.exists():
        return str(target)

    old_cwd = Path.cwd()
    try:
        from ultralytics.utils.downloads import attempt_download_asset

        os.chdir(DEFAULT_CHECKPOINT_DIR)
        downloaded = Path(str(attempt_download_asset(model_path.name))).expanduser()
    finally:
        os.chdir(old_cwd)
    if not downloaded.is_absolute():
        downloaded = DEFAULT_CHECKPOINT_DIR / downloaded
    if downloaded.exists() and downloaded.resolve() != target.resolve():
        shutil.copy2(downloaded, target)
    if not target.exists():
        raise FileNotFoundError(f"YOLO base model was not found or downloaded: {target}")
    return str(target)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", required=True, help="Path to data.yaml")
    parser.add_argument("--model", default="yolo26n-seg.pt", help="Base segmentation model, e.g. yolo26n-seg.pt")
    parser.add_argument("--epochs", type=int, default=120)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="cpu", help="cpu, 0, or cuda:0")
    parser.add_argument("--project", default="runs/robot_yolo")
    parser.add_argument("--name", default="robot_yolo26n_seg")
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument(
        "--export-checkpoint",
        default="checkpoints/robot_yolo26n_seg.pt",
        help="Copy the trained best.pt here after training. Use empty string to disable.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data_path = Path(args.data)
    if not data_path.exists():
        raise FileNotFoundError(f"data.yaml not found: {data_path}")

    model = YOLO(resolve_model_path(args.model))
    results = model.train(
        data=str(data_path),
        task="segment",
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        patience=args.patience,
        workers=4,
        cache=False,
        plots=True,
    )
    export_checkpoint = str(args.export_checkpoint or "").strip()
    if export_checkpoint:
        save_dir = Path(getattr(results, "save_dir", Path(args.project) / args.name))
        best_path = save_dir / "weights" / "best.pt"
        if not best_path.exists():
            raise FileNotFoundError(f"training finished, but best.pt was not found: {best_path}")
        target = Path(export_checkpoint)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(best_path, target)
        print(f"exported best checkpoint to {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
