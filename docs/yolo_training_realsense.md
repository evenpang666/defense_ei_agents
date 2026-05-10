# RealSense 场景 YOLO 模型训练与数据采集

本文档用于训练适配当前 UR7e + 双 RealSense D435i 工作流的自定义 YOLO 模型。目标是让 `evaluate_defense_agent_real.py` 中的 `--perception-backend yolo` 能稳定识别可操作物体，并通过 YOLO mask/box + 深度图重建局部点云，得到更可靠的抓取位置点。

## 推荐结论

优先训练 **YOLO26 segmentation 模型**，例如从 `yolo26n-seg.pt` 微调。原因是当前工作流会用检测结果筛选深度像素并生成物体局部点云：

- 有 segmentation mask 时：只取物体轮廓内的深度点，抓取点更干净。
- 只有 detection box 时：会取整个框内的深度点，容易混入桌面、夹爪、邻近物体。

如果你的任务只是粗定位，检测模型也能用；如果要做稳定抓取，建议标注实例分割 mask。

## 数据目录

建议在仓库外或仓库内 `datasets/` 下建立如下结构：

```text
datasets/robot_objects_seg/
  images/
    train/
    val/
    test/
  labels/
    train/
    val/
    test/
  data.yaml
```

`data.yaml` 示例：

```yaml
path: C:/Users/15261/Documents/projects/defense_ei_agents/datasets/robot_objects_seg
train: images/train
val: images/val
test: images/test

names:
  0: cube
  1: bowl
  2: beaker
  3: cup
  4: sponge
```

类别名要和运行时 `--yolo-target-labels`、任务描述中的物体名尽量一致。例如训练类别叫 `cube`，运行时也使用：

```bash
--yolo-target-labels cube,bowl
```

## 数据采集方法

### 1. 采集原则

每个类别至少先采集 200 到 500 张有效图像；如果物体外观变化大，建议 1000 张以上。数据要覆盖：

- global 相机视角：完整桌面、不同物体位置、不同遮挡关系。
- wrist 相机视角：夹爪靠近前、靠近中、抓取前的近距离视角。
- 光照变化：正常光、偏暗、反光、阴影。
- 背景变化：桌面不同区域、容器内外、靠近机器人本体。
- 操作状态：单物体、多物体相邻、被夹爪部分遮挡、物体贴近容器边缘。

不要只拍“干净居中”的物体。模型真正上线时最容易失败的是边缘、遮挡、反光、远近尺度变化。

### 2. 用现有工作流快速采集

当前脚本已经支持只采集图像，不连接机器人执行任务：

```bash
python evaluate_defense_agent_real.py ^
  --task "capture yolo dataset" ^
  --capture-only ^
  --camera-serials GLOBAL_SERIAL,WRIST_SERIAL ^
  --camera-width 640 ^
  --camera-height 480 ^
  --camera-fps 15 ^
  --camera-frame-timeout-ms 20000 ^
  --camera-capture-retries 5
```

每次运行会在 `logs/defense_agent_real/<timestamp>/` 下保存：

```text
current_global_rgb.png
current_wrist_rgb.png
current_global_depth.npy
current_wrist_depth.npy
current_global_rgb_intrinsics.json
current_wrist_rgb_intrinsics.json
summary.json
```

训练 YOLO 只需要 RGB 图像；深度图不参与 YOLO 训练，但后续运行时会用于点云和抓取点估计。

采集后，将 RGB 图像复制到数据集目录，例如：

```text
datasets/robot_objects_seg/images/train/
  000001_global.png
  000001_wrist.png
  000002_global.png
  000002_wrist.png
```

建议命名时保留视角信息：`global`、`wrist`。两个视角可以放在同一个训练集里，这样一个模型能同时适应远景和腕部近景。

### 3. 连续采集脚本

如果需要一次性采集很多张，可以新建 `tools/capture_yolo_dataset.py`，内容如下：

```python
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
    serial_pair = tuple(serials[:2]) if serials else None

    with RealSenseRGBPair(
        serials=serial_pair,
        width=args.width,
        height=args.height,
        fps=args.fps,
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
```

运行：

```bash
python tools/capture_yolo_dataset.py ^
  --out datasets/raw_yolo_capture ^
  --camera-serials GLOBAL_SERIAL,WRIST_SERIAL ^
  --count 500 ^
  --interval-sec 0.8
```

采集时每隔几张手动移动物体、旋转物体、改变夹爪位置。不要让连续 500 张几乎完全一样，否则有效数据量很低。

## 标注方法

推荐使用 CVAT、Label Studio、Roboflow 或 makesense.ai 标注。导出格式选择：

- 训练 segmentation：导出 YOLO segmentation polygon 格式。
- 训练 detection：导出 YOLO detection bbox 格式。

YOLO segmentation 的标签文件每一行格式类似：

```text
class_id x1 y1 x2 y2 x3 y3 ... xn yn
```

所有坐标都是 0 到 1 的归一化坐标。每张图片对应一个同名 `.txt`：

```text
images/train/000001_global.png
labels/train/000001_global.txt
```

标注建议：

- mask 尽量贴合可见物体轮廓，不要包含桌面。
- 被遮挡物体只标可见区域即可。
- 夹爪不要标成物体。
- 同类多个实例要分别标注多个 mask。
- 如果某张图没有目标，也可以保留空 `.txt`，用于降低误检。

## 训练脚本

新建 `tools/train_yolo.py`：

```python
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
```

CPU 训练很慢，只适合先跑通流程：

```bash
python tools/train_yolo.py ^
  --data datasets/robot_objects_seg/data.yaml ^
  --model yolo26n-seg.pt ^
  --epochs 5 ^
  --batch 2 ^
  --device cpu
```

GPU 环境正常时：

```bash
python tools/train_yolo.py ^
  --data datasets/robot_objects_seg/data.yaml ^
  --model yolo26n-seg.pt ^
  --epochs 120 ^
  --imgsz 640 ^
  --batch 8 ^
  --device 0 ^
  --name robot_yolo26n_seg_v1
```

训练完成后，最佳权重通常在：

```text
runs/robot_yolo/robot_yolo26n_seg_v1/weights/best.pt
```

## 验证模型

先对验证集看指标：

```bash
yolo segment val ^
  model=runs/robot_yolo/robot_yolo26n_seg_v1/weights/best.pt ^
  data=datasets/robot_objects_seg/data.yaml ^
  imgsz=640 ^
  device=cpu
```

再对真实日志图片做可视化预测：

```bash
yolo segment predict ^
  model=runs/robot_yolo/robot_yolo26n_seg_v1/weights/best.pt ^
  source=logs/defense_agent_real/20260510_123409/atomic_01/phase_01_approach_object_and_open_gripper/attempt_01 ^
  imgsz=640 ^
  conf=0.25 ^
  device=cpu ^
  save=True
```

重点人工检查：

- 目标是否漏检。
- 物体相邻时 mask 是否粘连。
- 腕部近距离图像是否还能识别。
- 夹爪遮挡时是否仍能给出可用可见区域。
- 容器、透明物体、反光物体是否频繁误检。

## 接入当前工作流

将训练好的权重复制到 `checkpoints/`：

```bash
copy runs\robot_yolo\robot_yolo26n_seg_v1\weights\best.pt checkpoints\robot_yolo26n_seg.pt
```

运行工作流时指定模型：

```bash
python evaluate_defense_agent_real.py ^
  --task "pick up the cube and place it in the bowl" ^
  --robot-ip 169.254.26.10 ^
  --camera-serials GLOBAL_SERIAL,WRIST_SERIAL ^
  --global-camera-base-transform calibration/t_base_global_camera.json ^
  --yolo-seg-model checkpoints/robot_yolo26n_seg.pt ^
  --yolo-target-labels cube,bowl ^
  --yolo-conf 0.25 ^
  --yolo-iou 0.5 ^
  --yolo-device cpu
```

如果你的 CUDA 版 `torch` 和 `torchvision` 匹配，可以把最后一项改成：

```bash
--yolo-device 0
```

如果再次遇到 `torchvision::nms` 的 CUDA backend 报错，先继续使用 `--yolo-device cpu`，再单独修复 PyTorch/torchvision CUDA 安装。

## 迭代策略

第一版不要追求类别很多，建议先做 2 到 5 个核心类别，例如 `cube`、`bowl`、`beaker`。跑完整机器人闭环后，把失败日志里的图片加入数据集继续标注，形成第二版：

1. 从 `logs/defense_agent_real/...` 找漏检、误检、mask 不准的图片。
2. 加入 `images/train` 或 `images/val`。
3. 标注对应 `.txt`。
4. 从上一版 `best.pt` 继续训练。
5. 用同一批真实日志图片回归测试。

继续训练命令示例：

```bash
python tools/train_yolo.py ^
  --data datasets/robot_objects_seg/data.yaml ^
  --model checkpoints/robot_yolo26n_seg.pt ^
  --epochs 80 ^
  --imgsz 640 ^
  --batch 8 ^
  --device 0 ^
  --name robot_yolo26n_seg_v2
```

## 数据质量检查清单

训练前确认：

- `images/train` 和 `labels/train` 文件名一一对应。
- `images/val` 和 `labels/val` 文件名一一对应。
- `data.yaml` 的 `names` 顺序和标注工具导出的 class id 一致。
- 每个类别在 train/val 都有样本。
- val 集不要和 train 集是连续重复帧。
- global 和 wrist 视角都进入训练集。
- 包含一些无目标或干扰物图片，用于降低误检。

上线前确认：

- 用 `--capture-only` 采集当前真实场景图片。
- 对这些图片跑 `yolo segment predict`。
- 检查 mask 是否覆盖真实可抓取物体区域。
- 再接入 `evaluate_defense_agent_real.py`，观察 `scene_state.json` 中的 `objects[].grasp_region_center_gripper_mm` 是否稳定。
