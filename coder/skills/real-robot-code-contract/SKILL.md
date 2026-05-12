---
name: real-robot-code-contract
description: Real UR7e runtime code contract for defense_ei_agents with Intel RealSense RGB feedback.
tags: [defense_ei_agents, atomic, codegen, runtime, ur7e, real-robot]
---

# Real-Robot Code Contract

The code is executed on `UR7eVectorController`. Generate code for exactly one
requested phase of one atomic task; the orchestrator executes and judges that
phase before requesting the next one. There is no MuJoCo scene and no evobody
dependency.

Allowed runtime APIs:
- `move_x(distance, velocity=0.04, acceleration=0.18)`
- `move_y(distance, velocity=0.04, acceleration=0.18)`
- `move_z(distance, velocity=0.04, acceleration=0.18)`
- `rotate_x(angle_rad, velocity=0.04, acceleration=0.18)`
- `rotate_y(angle_rad, velocity=0.04, acceleration=0.18)`
- `rotate_z(angle_rad, velocity=0.04, acceleration=0.18)`
- `look_at_operated_object(max_angle_rad=0.35, velocity=0.035, acceleration=0.14, max_refine_steps=3)`
  - rotate wrist-camera/gripper +Z toward the detected operated-object grasp
    region, immediately capture fresh RGB-D perception, and repeat bounded gaze
    corrections until the wrist camera detects the operated object or the
    bounded refine limit is reached.
- `sleep(seconds)`
- `gripper_control(value, delay)` - Robotiq command, 0=open and 255=closed; delay in ms.

Generation rules:
- Output exactly one Python code block.
- Do not call simulation tools.
- Do not generate quaternion code. Avoid names or keys such as `quat`,
  `quaternion`, `wxyz`, or `xyzw`; use 6D Cartesian TCP pose vectors instead.
- Use millimeters for every `move_x`/`move_y`/`move_z` translation. Examples: use `10` for
  10 mm, `50` for 5 cm, and `100` for 10 cm. Do not write `0.05` when you mean
  5 cm.
- Use radians for every `rotate_x`/`rotate_y`/`rotate_z` rotation increment. Examples: `0.1745` is
  about 10 degrees and `1.5708` is about 90 degrees.
- Use only the allowed axis-wise movement and rotation APIs for arm motion plus
  `look_at_operated_object` for gaze. Do not call `move_to_keypoint`,
  `move_ee`, `ee_pose`, `move_to`, or `print`; they are not available in
  generated-code runtime API.
- If the atomic task names a primitive skill, follow `primitive-skill-contract`
  for the required phase order, but emit only the current requested phase.
- Primitive skill names are labels, not runtime APIs. Do not call
  `pick_place`, `pick_and_place`, `push`, `pull`, `press`, `open`, `close`, or
  `pour`; expand the current requested phase into low-level motion commands.
- Do not invent exact object coordinates from RGB images alone.
- If `keypoint_database.records` contains a named pose that clearly matches the
  current grasp/place phase, use it only as reference context for where the
  grasp/place point is. Approach through `move_x`/`move_y`/`move_z` and
  `rotate_*` commands with travel distances that match the remaining offset; do
  not call a direct keypoint movement API.
- Use relative motions, controlled contact, and explicit gripper commands.
- If `scene_state_brief` is present, it may include approximate
  `center_gripper_mm` and `grasp_region_center_gripper_mm` values from RGB-D
  perception. Use these as direction and approximate distance hints for
  gripper-frame corrections. Do not artificially shrink moves to tiny deltas
  when the reference pose or scene evidence implies a larger offset, and do not
  drive directly into contact based on depth alone.
- If neither `scene_state_brief.objects` nor `scene_state_brief.wrist_objects`
  contains the operated object, or the matching entry is `not_detected`, do not
  call `look_at_operated_object(...)`. Use keypoint references, visible context,
  prior feedback, and axis-wise moves instead.
- For `pick_place`, call `look_at_operated_object(...)` near the end of the
  first two approach phases only:
  `approach_object_above_and_open_gripper` and
  `descend_to_source_prepare_to_grasp`, and only when the operated object is
  present in scene_state/scene_state_brief. Later phases may skip gaze because
  the grasped object can occlude the wrist camera.
- Overall motion must be controlled enough to protect real objects. Long
  free-space alignment moves may use appropriately sized axis-wise distances.
- Near-object interaction phases must be slower than transit phases: approach,
  descend, grasp, push/pull/press contact, placement, release, and retraction
  from a container should slow down near contact and use brief `sleep(...)`
  pauses.
- Add a short `sleep(...)` after every move command so each hardware motion has
  time to finish and settle before the next command.
- If the atomic task info includes a concrete 6D TCP pose vector, treat it only
  as contextual guidance. Do not call pose-reading or absolute-pose APIs from
  generated code.
- If feedback says the previous attempt for this phase failed, address the
  specific failure while preserving already completed phase state.

Coordinate frame for axis-wise movement:

- Wrist image right is gripper +X, so `move_x(+distance)`
  moves toward the right side of the wrist image.
- Wrist image down is gripper +Y, so `move_y(+distance)`
  moves toward the lower side of the wrist image.
- Wrist camera viewing direction is gripper +Z, so `move_z(+distance)`
  moves along the wrist camera forward direction; use the opposite sign to move
  away from that direction.

Orientation reference for axis-wise rotation:

- `rotate_x`/`rotate_y`/`rotate_z` are relative increments, not absolute target angles.
- `[0, 0, 0]` keeps the current gripper orientation.
- `rotate_y(+1.5708)` is the first reference increment for pitching a horizontal
  gripper toward a vertical-down grasp; use `[0, -1.5708, 0]` for the opposite
  pitch direction if the observed wrist image motion is reversed.
- `rotate_y(-1.5708)` is the first reference increment for pitching a vertical-down
  gripper back toward a horizontal gripper.
- `rotate_z(+1.5708)` rotates about the wrist image forward / gripper +Z axis.
- `rotate_x(+1.5708)` rolls the gripper around the wrist image right / gripper +X
  axis.
