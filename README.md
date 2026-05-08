# defense_ei_agents Real UR7e Workflow

This workflow runs four DefenseAgent ReAct agents around a real UR7e:

1. `planner` decomposes the natural-language task into primitive-level atomic tasks.
2. `supervisor` turns each atomic task into execution-relevant information and visual done criteria.
3. `coder` generates restricted Python for exactly one phase of one atomic task.
4. `judger` evaluates each post-phase global/wrist RGB image pair and returns `SUCCESS` or `FAIL`.

The orchestrator is `evaluate_defense_agent_real.py`. It executes real robot code only; there is no dry-run or generate-only mode.

## Safety Notes

- Use this only with the robot in a supervised lab setting.
- Put the UR e-Series robot in Remote Control mode before running.
- Keep E-stop reachable.
- Test camera capture and a tiny free-space motion before object interaction.
- Generated code is statically checked before execution. It may call only:
  `move_x`, `move_y`, `move_z`, `rotate_x`, `rotate_y`, `rotate_z`, `sleep`,
  and `gripper_control`.
- Primitive names such as `pick_place`, `push`, `pull`, `press`, `open`,
  `close`, and `pour` are labels only. The coder must expand them into
  axis-wise motion and `gripper_control` phases.

## Required Environment Variables

DefenseAgent resolves LLM settings from profile values first, then environment variables. The checked-in profiles intentionally leave `provider`, `model`, `base_url`, and `api_key` blank.

Recommended OpenAI-compatible setup:

```bash
export AGENT_LAB_LLM_PROVIDER="openai"
export LLM_MODEL_ID="gpt-5.4"
export LLM_BASE_URL="https://api.qnaigc.com"
export LLM_API_KEY="..."
```

Provider-specific variables also work when `LLM_*` is unset:

```bash
export AGENT_LAB_LLM_PROVIDER="openai"
export OPENAI_MODEL="gpt-5.4"
export OPENAI_BASE_URL="https://api.qnaigc.com"
export OPENAI_API_KEY="..."
```

Optional embedding variables are currently accepted by the CLI for future memory/RAG use. The real workflow now runs with memory disabled, so these are not required:

```bash
export EMBEDDING_MODEL="text-embedding-3-small"
export EMBEDDING_API_KEY="..."
export EMBEDDING_BASE_URL="https://api.qnaigc.com"
export EMBEDDING_DIMS="1536"
```

## Dependencies

Install the project dependencies, DefenseAgent, RealSense bindings, and UR RTDE support in the active Python environment:

```bash
pip install imageio pyyaml numpy openai ur-rtde pyrealsense2 ultralytics
pip install DefenseAgent
```

Installing DefenseAgent with `pip install -e ...` is important because the source tree also needs package dependencies such as `ms-agent` and `omegaconf`.

If DefenseAgent lives outside this copied folder, either install it into the active environment or set:

```bash
export DEFENSE_AGENT_ROOT="/path/to/DefenseAgent"
```

For the original sibling checkout layout, the script also auto-detects `../defense_EI/third_party/DefenseAgent`.

If you use the repository-wide environment from the top-level README, make sure `ur-rtde`, `pyrealsense2`, `imageio`, and `pyyaml` are present.

## Hardware Inputs

The runtime uses:

- URScript primary socket: port `30003`
- URScript secondary socket: port `30002`
- RTDE receive: port `30004`
- Robotiq socket: port `63352`
- Two Intel RealSense D435i RGB-D streams, global and wrist

If only one RealSense stream is found, it is reused as both global and wrist input. For reliable judging, use two cameras and pass serials explicitly:

```bash
--camera-serials GLOBAL_SERIAL,WRIST_SERIAL
```

`GLOBAL_SERIAL` and `WRIST_SERIAL` are placeholders. To list the serials detected by `pyrealsense2`, run:

```bash
python evaluate_defense_agent_real.py --list-cameras
```

If listing cameras reports access denied on Windows, close Intel RealSense Viewer or other camera apps and check Windows camera privacy permissions.

## RGB-D Scene State

The workflow now saves depth sidecars and a structured `scene_state.json` for
the initial observation and every post-phase capture. The scene state is passed
to both coder and judger as approximate support for object/grasp-region
distances relative to the current gripper. Object estimation uses YOLO
segmentation masks plus RealSense depth.

For calibrated base-frame coordinates, provide the fixed global-camera extrinsic:

```bash
python evaluate_defense_agent_real.py \
  --task "pick up the red block and place it into the bowl" \
  --robot-ip 169.254.26.10 \
  --camera-serials GLOBAL_SERIAL,WRIST_SERIAL \
  --global-camera-base-transform calibration/t_base_global_camera.json \
  --yolo-seg-model models/best.pt
```

The transform file should contain either a top-level 4x4 JSON array or
`{"matrix": [[...], [...], [...], [...]]}` for `T_base_global_camera`. If no
global-camera transform is supplied, the run still saves RGB-D sidecars and
camera-frame estimates, but base/gripper object coordinates are `null`.

The wrist camera defaults to an approximate gripper transform with a `-0.02 m`
offset along gripper Y. Override it with:

```bash
--wrist-camera-gripper-transform calibration/t_gripper_wrist_camera.json
```

Use a YOLO segmentation model, not a detection-only model. For Ultralytics this
means a `*-seg.pt` model or your own trained segmentation checkpoint. The mask
pixels are combined with aligned depth, then converted into
`objects[].grasp_region_center_gripper_mm`.

To limit scene_state to specific YOLO classes, pass comma-separated labels:

```bash
--yolo-target-labels block,bowl
```

If labels are omitted, all YOLO segmentation detections are included. You can
tune thresholds with:

```bash
--yolo-conf 0.35 --yolo-iou 0.5
```

To create `calibration/t_base_global_camera.json`, use
`calibrate_global_camera.py`. See
[`docs/global_camera_calibration.md`](docs/global_camera_calibration.md) for the
full step-by-step procedure.

## Capture Images Only

Use this to verify camera wiring without connecting to the robot:

```bash
python evaluate_defense_agent_real.py \
  --task "capture test" \
  --capture-only \
  --camera-serials GLOBAL_SERIAL,WRIST_SERIAL
```

The command writes `current_global_rgb.png`, `current_wrist_rgb.png`, depth
sidecars, intrinsics JSON files, and `summary.json` under
`logs/defense_agent_real/<timestamp>/`.

## Run The Full Workflow

Normal run with live initial capture:

```bash
python evaluate_defense_agent_real.py \
  --task "pick up the red block and place it into the bowl" \
  --robot-ip 169.254.26.10 \
  --camera-serials GLOBAL_SERIAL,WRIST_SERIAL \
  --max-attempts-per-atomic 3
```

<!-- python evaluate_defense_agent_real.py --task "pick up the cube and place it in the beaker" --robot-ip 169.254.26.10 --camera-serials 405622072940,420222071502 --max-steps 8 --max-attempts-per-atomic 3 -->

Run from existing initial images while still using RealSense for post-execution feedback:

```bash
python evaluate_defense_agent_real.py \
  --task "pick up the red block and place it into the bowl" \
  --current-global-image logs/example/current_global_rgb.png \
  --current-wrist-image logs/example/current_wrist_rgb.png \
  --robot-ip 169.254.26.10 \
  --camera-serials GLOBAL_SERIAL,WRIST_SERIAL
```

When existing initial images are supplied, both paths must be provided. The orchestrator still opens the RealSense cameras so the judger receives fresh after-action images. `global_image` is the whole-scene view; `wrist_image` is the gripper view and may show part of the gripper along the bottom.

## Robotiq Fallback

By default, arm control can continue if the Robotiq socket is unavailable unless `--strict-gripper-connection` is set. If your setup requires URScript fallback definitions, pass:

```bash
--robotiq-urscript-defs-path /path/to/robotiq_defs.script
```

Use strict mode when gripper motion is required for the task:

```bash
--strict-gripper-connection
```

## Outputs

Each run creates a timestamped directory under `logs/defense_agent_real/` containing:

- `real_*_profile.yaml`: resolved profile copies for the run
- `defense_ei_plan.json`: planner output
- `defense_ei_atomic_task_info.json`: supervisor output
- `atomic_XX/phase_NN_<slug>/attempt_YY/real_phase_actions.py`: generated code
  for one phase only
- `atomic_XX/phase_NN_<slug>/attempt_YY/syntax_check.json`: syntax and runtime
  API validation for that phase
- `atomic_XX/phase_NN_<slug>/attempt_YY/real_execution.json`: robot execution
  report for that phase
- `atomic_XX/phase_NN_<slug>/attempt_YY/phase_NN_<slug>_global_rgb.png` and
  `phase_NN_<slug>_wrist_rgb.png`: post-phase validation images, resized
  proportionally to 128 px wide
- `atomic_XX/phase_NN_<slug>/attempt_YY/phase_NN_<slug>_scene_state.json`:
  post-phase RGB-D scene-state estimates for coder/judger context
- `atomic_XX/phase_NN_<slug>/attempt_YY/real_atomic_judge_phase_NN.json`:
  per-phase judger result; the next phase is generated only after this phase is
  judged `SUCCESS`
- `atomic_XX/phase_NN_<slug>/attempt_YY/real_atomic_judge.json`: final result
  for the current phase attempt
- `summary.json`: final status summary

Exit code is `0` when all atomic tasks complete and `2` when the workflow stops after failed attempts.
`--max-attempts-per-atomic` now limits retries for each phase within an atomic
task.

## Profiles And Contracts

Profiles live in:

- `planner/profile.yaml`
- `supervisor/profile.yaml`
- `coder/profile.yaml`
- `judger/profile.yaml`

The coder contracts are the most safety-critical:

- `coder/prompts/system.md`
- `coder/skills/atomic-code-contract/SKILL.md`
- `coder/skills/real-robot-code-contract/SKILL.md`
- `coder/skills/primitive-skill-contract/SKILL.md`
- `coder/python_tools/atomic_runtime_contract.py`

If you add new runtime APIs, update all of these plus `make_real_runtime_api()` and the orchestrator's static runtime-call allowlist together.
