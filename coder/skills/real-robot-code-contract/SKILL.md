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
- Use only the allowed axis-wise movement and rotation APIs for incremental arm
  motion. Do not call `move_ee`, `ee_pose`, `move_to`, or `print`; they are not
  available in generated-code runtime API.
- If the atomic task names a primitive skill, follow `primitive-skill-contract`
  for the required phase order, but emit only the current requested phase.
- Primitive skill names are labels, not runtime APIs. Do not call
  `pick_place`, `pick_and_place`, `push`, `pull`, `press`, `open`, `close`, or
  `pour`; expand the current requested phase into low-level motion commands.
- Do not invent exact object coordinates from RGB images alone.
- Use small relative motions, slow contact, and explicit gripper commands.
- Overall motion must be slow enough to protect real objects. Prefer many small
  axis-wise increments and low `velocity`/`acceleration` over a single large move.
- Near-object interaction phases must be slower than transit phases: approach,
  descend, grasp, push/pull/press contact, placement, release, and retraction
  from a container should use smaller deltas, lower `velocity`/`acceleration`,
  and brief `sleep(...)` pauses.
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
