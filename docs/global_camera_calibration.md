# Global Camera To UR Base Calibration

This guide explains how to create the calibration file consumed by:

```bash
--global-camera-base-transform calibration/t_base_global_camera.json
```

The file stores `T_base_global_camera`, the 4x4 transform that maps a 3D point
from the fixed global RealSense camera frame into the UR base frame:

```text
P_base = T_base_global_camera * P_global_camera
```

Once this file exists, the workflow can convert global RGB-D object positions
into approximate object/grasp-region distances relative to the current gripper.

## What You Need

- UR7e in Remote Control mode.
- Fixed global RealSense camera. Do not move it after calibration.
- `pyrealsense2`, `numpy`, `imageio`, and `ur-rtde` installed.
- A visible calibration point on the table. Good options:
  - a printed cross,
  - a small dot sticker,
  - the corner of a checkerboard,
  - the center of a small colored marker.
- At least 6 calibration points, preferably 8-12, spread across the workspace.

Avoid transparent, reflective, or very dark points because RealSense depth may
be noisy or missing there.

## Basic Command

List RealSense cameras:

```bash
python calibrate_global_camera.py --list-cameras
```

Run interactive calibration:

```bash
python calibrate_global_camera.py \
  --robot-ip 169.254.26.10 \
  --camera-serial GLOBAL_SERIAL \
  --num-points 8 \
  --output calibration/t_base_global_camera.json
```

Replace `GLOBAL_SERIAL` with the serial number of the fixed global camera.

If OpenCV with GUI support is installed, the script opens each captured image
and lets you click the calibration pixel. If OpenCV is missing GUI support
(for example `opencv-python-headless`) or GUI windows are inconvenient, the
script falls back to typed pixel input automatically. You can also force typed
input with:

```bash
python calibrate_global_camera.py \
  --robot-ip 169.254.26.10 \
  --camera-serial GLOBAL_SERIAL \
  --num-points 8 \
  --no-click \
  --output calibration/t_base_global_camera.json
```

With `--no-click`, the script saves an RGB image for each point and asks you to
type the pixel coordinate as `u v`.

## Interactive Procedure

For each calibration point:

1. Place or choose a visible point in the global camera view.
2. Move the robot TCP/gripper tip exactly to that physical point.
3. Press Enter in the calibration script.
4. The script captures RGB-D and saves files under `calibration/samples/`.
5. Click the same physical point in the captured RGB image, or type its pixel.
6. The script reads:
   - `P_global_camera`: from clicked pixel plus depth.
   - `P_base`: from `controller.get_current_tcp_pose()[:3]`.

After all points are collected, the script fits `T_base_global_camera` and writes
the output JSON.

## Point Placement Advice

Good calibration points should cover the robot workspace:

- front-left
- front-right
- back-left
- back-right
- center
- several points near expected manipulation targets

Do not use points that are all on one tiny area. Wider point spread improves the
rotation estimate.

Most tabletop points are coplanar, which is acceptable for a first practical
calibration, but adding a few points at different heights improves robustness.
For example, touch the top center of a small block at two or three positions.

## Output File

The output looks like:

```json
{
  "schema": "defense_ei_agents.T_base_global_camera.v1",
  "usage": "P_base = T_base_global_camera * P_global_camera",
  "matrix": [
    [0.999, 0.001, 0.002, 0.412],
    [-0.001, 0.998, -0.004, -0.183],
    [-0.002, 0.004, 0.999, 0.732],
    [0.0, 0.0, 0.0, 1.0]
  ],
  "rmse_m": 0.008,
  "point_count": 8
}
```

Only `matrix` is required by the workflow. The other fields help you audit the
quality of the calibration.

## Quality Check

The script prints RMSE after fitting:

- `< 10 mm`: good for most tabletop guidance.
- `10-20 mm`: usually usable for coarse gripper-relative hints.
- `> 20 mm`: recheck point clicks, TCP placement, camera stability, and depth.

Common causes of high error:

- clicked pixel does not match the physical point touched by TCP,
- TCP tip is not actually at the selected point,
- RealSense depth is missing/noisy at the clicked point,
- global camera moved during collection,
- too few points or points clustered too closely.

## Using The Calibration

After calibration:

```bash
python evaluate_defense_agent_real.py \
  --task "pick up the red block and place it into the bowl" \
  --robot-ip 169.254.26.10 \
  --camera-serials GLOBAL_SERIAL,WRIST_SERIAL \
  --global-camera-base-transform calibration/t_base_global_camera.json \
  --yolo-seg-model models/best.pt
```

Each run will save `current_scene_state.json` and post-phase
`phase_XX_<slug>_scene_state.json` files. These include fields such as:

```json
{
  "objects": [
    {
      "label": "red",
      "center_base_m": [0.42, -0.13, 0.08],
      "center_gripper_mm": [35.0, -18.0, 142.0],
      "grasp_region_center_gripper_mm": [35.0, -18.0, 142.0]
    }
  ]
}
```

The coder uses those distances as conservative motion hints. The judger uses
post-execution distances as supporting evidence when deciding whether the phase
succeeded.

## Offline Refit

If you want to edit or remove bad points, create a JSON file with:

```json
{
  "points": [
    {
      "camera_point_m": [0.1, -0.2, 0.8],
      "base_point_m": [0.45, -0.12, 0.03]
    }
  ]
}
```

Then refit without reconnecting hardware:

```bash
python calibrate_global_camera.py \
  --from-correspondences calibration/my_points.json \
  --output calibration/t_base_global_camera.json
```

You need at least 3 non-collinear points; 6-12 well-spread points are better.
