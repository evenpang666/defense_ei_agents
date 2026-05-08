---
name: primitive-skill-contract
description: Motion templates for defense_ei_agents primitive_skill code generation on a real UR7e.
tags: [defense_ei_agents, primitive_skill, pick_place, real-robot, codegen]
---

# Primitive Skill Contract

Use this contract when the atomic task info contains a `primitive`,
`primitive_skill`, or equivalent action label such as `pick_place`.

## Global Rules

- Implement only the current requested phase of the named primitive skill. The
  orchestrator executes this phase, captures images, and asks the judger for
  phase feedback before requesting the next phase.
- `move_x(distance, velocity=0.04, acceleration=0.18)`, `move_y(...)`, and `move_z(...)` use millimeters in the gripper/wrist-image frame. `rotate_x(angle_rad, ...)`, `rotate_y(...)`, and `rotate_z(...)` use radians. Do not pass `steps`; real hardware uses velocity and acceleration.
- Use the wrist-image/gripper frame for relative moves: wrist image right is
  gripper +X, wrist image down is gripper +Y, and the wrist image viewing
  direction is gripper +Z.
- Prefer vertical and horizontal Cartesian translations for pick/place style
  manipulation. Avoid diagonal shortcuts near objects, containers, rims, or
  clutter.
- All primitive skills must be object-safe and slow. Use conservative travel
  distances, split large moves into smaller segments, lower
  `velocity`/`acceleration`, and add short `sleep(...)` pauses before/after
  object contact.
- Add a short `sleep(...)` after every move command so the real robot has time
  to complete and settle.
- Interaction phases must be especially slow: final approach, descent to grasp,
  gripper close, initial lift, descent to target, release, and retraction from
  containers.
- Use only `move_x`, `move_y`, `move_z`, `rotate_x`, `rotate_y`, and `rotate_z` for incremental arm motion. Do not call `move_ee`, `ee_pose`, `move_to`, or `print`; generated code cannot use those APIs. With RGB-only
  observations, use conservative relative moves and make the intended phase
  explicit.
- The generated code for a phase must start with exactly one
  `# === DEFENSE_EI_PHASE: <slug> | <short goal> ===`.
- Keep the gripper state explicit before and after contact.
- Primitive names are never callable runtime APIs. Do not call `pick_place`,
  `pick_and_place`, `push`, `pull`, `press`, `open`, `close`, or `pour`; use
  the pseudocode templates below and emit real axis-wise motion, `gripper_control`, and `sleep` calls.

## `pick_place` / `pick_and_place`

`pick_place` is a composition of vertical and horizontal translations:

1. Make gripper head down (axis z should point down at the table).
2. Move to a safe pose above the operated object.
3. Open the gripper.
4. Move vertically downward to the operated object grasp height slowly, using
   smaller deltas and lower `velocity`/`acceleration`.
5. Close the gripper on the operated object and pause briefly to let the grasp
   settle.
6. Move vertically upward to a safe carry height slowly at first, then continue
   to the final safe height.
7. Determine all objects between the operated object and the target position
   from the atomic task info and visual notes. The safe carry height must be
   higher than every intervening object's height. If exact heights are unknown,
   choose a conservative extra lift and state the reason in the phase goal.
8. Keep the gripper closed and translate horizontally to a pose above the target
   position. Do not lower during this transfer.
9. Move vertically downward to the target placement position slowly.
10. Open the gripper to release and pause briefly.
11. Retract vertically upward to a safe height slowly before any horizontal
    motion away.

Required phase order:

- `point_down`
- `approach_source_above`
- `open_gripper`
- `descend_to_source`
- `grasp_source`
- `lift_to_safe_height`
- `transfer_above_target`
- `descend_to_target`
- `release_at_target`
- `retract_up`

Phase success criteria for staged execution:

- `point_down`: the gripper is oriented for a vertical-down approach to the
  table/object, without contacting the object or colliding with the scene.
- `approach_source_above`: the gripper is safely above the operated object and
  laterally aligned for grasping, while still clear of the object and clutter.
- `open_gripper`: the gripper fingers are visibly open enough for the object,
  and the object has not been disturbed.
- `descend_to_source`: the gripper has moved vertically to the source grasp
  height, remains aligned around or immediately beside the object, and has not
  pushed the object away.
- `grasp_source`: the gripper is closed on the correct object with a stable
  grasp; the object is not missed, dropped, knocked over, or squeezed out.
- `lift_to_safe_height`: the grasped object rises with the gripper to a safe
  carry height, clear of the table, source area, rims, and intervening objects.
- `transfer_above_target`: the object remains held while the gripper translates
  horizontally to a pose above the target region/opening at safe height.
- `descend_to_target`: the held object descends vertically into or onto the
  target placement region without hitting rims, obstacles, or missing the target.
- `release_at_target`: the gripper opens and leaves the correct object at the
  target position; the object is no longer held and has not rolled or fallen out
  of place.
- `retract_up`: the gripper retracts vertically to a safe height without
  disturbing the placed object; the full `pick_place` atomic task target state
  is satisfied.

For container targets such as beakers, cups, bins, or bowls:

- Transfer to above the opening/rim first.
- Lower slowly and vertically inside the opening.
- Release gently after the object is below the rim or clearly inside the target
  region.
- Retract vertically before any horizontal motion away from the container.

Do not skip `open_gripper` before descent or `lift_to_safe_height` before
horizontal transfer.

Per-phase pseudocode templates:

```python
# === DEFENSE_EI_PHASE: point_down | orient gripper downwards ===
rotate_x/y(...); sleep(...)
# === DEFENSE_EI_PHASE: approach_source_above | align above source using small wrist-frame corrections ===
move_x/y(...); sleep(...)
# === DEFENSE_EI_PHASE: open_gripper | prepare for grasp ===
gripper_control(0, 120); sleep(...)
# === DEFENSE_EI_PHASE: descend_to_source | slow vertical descent ===
move_z(...); sleep(...)
# === DEFENSE_EI_PHASE: grasp_source | close on object and settle ===
gripper_control(255, 250); sleep(...)
# === DEFENSE_EI_PHASE: lift_to_safe_height | lift before horizontal transfer ===
move_z(...); sleep(...)
# === DEFENSE_EI_PHASE: transfer_above_target | horizontal transfer at safe height ===
move_x/y(...); sleep(...)
# === DEFENSE_EI_PHASE: descend_to_target | slow vertical placement ===
move_z(...); sleep(...)
# === DEFENSE_EI_PHASE: release_at_target | release object ===
gripper_control(0, 180); sleep(...)
# === DEFENSE_EI_PHASE: retract_up | retreat vertically ===
move_z(...); sleep(...)
```

## `push`

Push is a contact translation using a closed or partially closed gripper as a
pusher. Required phase order:

- `approach_push_start`
- `set_push_contact_shape`
- `descend_or_advance_to_contact`
- `push_slowly`
- `retract_from_contact`

Phase success criteria for staged execution:

- `approach_push_start`: the gripper is near the intended push start pose,
  aligned behind or beside the correct object along the intended push direction,
  without contacting or moving it yet.
- `set_push_contact_shape`: the gripper forms a stable pusher shape such as
  closed or partially closed fingers, with no unwanted object motion.
- `descend_or_advance_to_contact`: the gripper makes gentle, controlled contact
  with the correct object at the intended push face without tipping, lifting, or
  displacing it prematurely.
- `push_slowly`: the correct object moves in the intended direction by the
  required amount while staying stable and avoiding collisions with unintended
  objects.
- `retract_from_contact`: the gripper moves away from the object after the push
  without dragging it back; the pushed object remains at the target state for
  the atomic task.

Per-phase pseudocode templates:

```python
# === DEFENSE_EI_PHASE: approach_push_start | align gripper before contact ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: set_push_contact_shape | make a stable pushing surface ===
gripper_control(180, 120); sleep(...)
# === DEFENSE_EI_PHASE: descend_or_advance_to_contact | slow contact approach ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: push_slowly | translate object in small increments ===
move_x(...); sleep(...)
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: retract_from_contact | back away from object ===
move_x(...); sleep(...)
```

## `pull`

Pull requires contact or grasp before moving opposite the intended contact
direction. Required phase order:

- `approach_pull_contact`
- `grasp_or_hook`
- `pull_slowly`
- `release_if_grasped`
- `retract_from_contact`

Phase success criteria for staged execution:

- `approach_pull_contact`: the gripper is aligned with the object's pullable
  feature or contact side, close enough to engage it without disturbing the
  object.
- `grasp_or_hook`: the gripper visibly grasps, hooks, or otherwise secures the
  correct object/feature so it can be pulled without slipping immediately.
- `pull_slowly`: the correct object moves in the intended pull direction by the
  required amount while remaining controlled and not colliding with unrelated
  objects.
- `release_if_grasped`: if the object was grasped or hooked, the gripper
  releases it cleanly at the pulled target state without moving it back.
- `retract_from_contact`: the gripper retreats from the object without further
  contact; the pulled object remains at the desired final state.

Per-phase pseudocode templates:

```python
# === DEFENSE_EI_PHASE: approach_pull_contact | align to pull feature ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: grasp_or_hook | secure contact before pulling ===
gripper_control(255, 220); sleep(...)
# === DEFENSE_EI_PHASE: pull_slowly | pull with short conservative increments ===
move_x(...); sleep(...)
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: release_if_grasped | let go after pull ===
gripper_control(0, 150); sleep(...)
# === DEFENSE_EI_PHASE: retract_from_contact | retreat from manipulated object ===
move_x(...); sleep(...)
```

## `press`

Press is a short approach, slow normal contact, brief hold, and retraction.
Required phase order:

- `approach_press_target`
- `set_press_contact_shape`
- `press_slowly`
- `hold_press`
- `retract_from_press`

Phase success criteria for staged execution:

- `approach_press_target`: the gripper is aligned with the correct press target
  and close to it, with no unintended contact or activation yet.
- `set_press_contact_shape`: the gripper forms a stable contact surface for
  pressing while remaining aligned with the correct target.
- `press_slowly`: the gripper presses the correct target along its apparent
  normal far enough to actuate or depress it, without sliding off or striking
  nearby objects.
- `hold_press`: the gripper maintains the press long enough for the target
  effect to register, with the target still visibly depressed or actuated if
  observable.
- `retract_from_press`: the gripper retracts along the approach direction
  without additional disturbance; the pressed target shows the desired final
  state if visible.

Per-phase pseudocode templates:

```python
# === DEFENSE_EI_PHASE: approach_press_target | align to button or target surface ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: set_press_contact_shape | close gripper for stable contact ===
gripper_control(255, 120); sleep(...)
# === DEFENSE_EI_PHASE: press_slowly | press along observed normal in small increments ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: hold_press | keep contact briefly ===
sleep(...)
# === DEFENSE_EI_PHASE: retract_from_press | move back along approach direction ===
move_x(...); sleep(...)
```

## `open` / `close`

Open and close articulated objects with a grasp/hook, small rotational or arc
increments, then release. Required phase order:

- `approach_handle_or_lid`
- `grasp_or_hook_articulation`
- `articulate_slowly`
- `release_articulation`
- `retract_from_articulation`

Phase success criteria for staged execution:

- `approach_handle_or_lid`: the gripper is aligned with the handle, lid, edge,
  or other articulated feature without disturbing the object.
- `grasp_or_hook_articulation`: the gripper visibly grasps or hooks the
  articulated feature securely enough to move it without slipping immediately.
- `articulate_slowly`: the articulated object moves toward the requested
  open/closed state through controlled increments without collision, overtravel,
  or loss of contact.
- `release_articulation`: the gripper releases the feature only after the object
  appears to have reached the requested open/closed state, without rebounding
  away from that state.
- `retract_from_articulation`: the gripper retreats clear of the articulated
  object; the object remains in the requested open/closed state for the atomic
  task.

Per-phase pseudocode templates:

```python
# === DEFENSE_EI_PHASE: approach_handle_or_lid | align to articulated feature ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: grasp_or_hook_articulation | secure the feature ===
gripper_control(255, 220); sleep(...)
# === DEFENSE_EI_PHASE: articulate_slowly | move through small arc increments ===
move_x(...); sleep(...)
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: release_articulation | release after target state is reached ===
gripper_control(0, 150); sleep(...)
# === DEFENSE_EI_PHASE: retract_from_articulation | retreat without bumping object ===
move_x(...); sleep(...)
```

## `pour`

Pour is grasp, lift, transfer above target, tilt in small increments, pause,
untilt, then place/release or retreat according to the atomic task. Required
phase order:

- `approach_source_above`
- `grasp_source`
- `lift_to_pour_height`
- `transfer_above_target`
- `tilt_to_pour`
- `hold_pour`
- `untilt`
- `return_or_release_source`
- `retract_up`

Phase success criteria for staged execution:

- `approach_source_above`: the gripper is safely above and aligned with the
  source container or object to be poured, without contacting or disturbing it
  yet.
- `grasp_source`: the gripper holds the correct source container securely enough
  for lifting and tilting, without crushing, slipping, or tipping it
  unintentionally.
- `lift_to_pour_height`: the grasped source lifts to a safe pour height, clear
  of the table, nearby objects, and the receiving container rim.
- `transfer_above_target`: the source remains held while moving to a controlled
  pose above the receiving target/opening, without spilling or colliding.
- `tilt_to_pour`: the source tilts over the receiving target enough for pouring
  while remaining positioned over the opening and avoiding spills outside the
  target.
- `hold_pour`: the source remains tilted long enough for the intended contents
  to transfer into the receiving target if visible, without losing grasp
  stability.
- `untilt`: the source rotates back toward upright while still held securely,
  with pouring stopped and no collision with the target rim.
- `return_or_release_source`: the source is returned, placed, or released
  according to the atomic task target state, with the gripper no longer applying
  unintended force.
- `retract_up`: the gripper retreats to a safe height without disturbing the
  source or receiving target; the full `pour` atomic task target state holds.

Per-phase pseudocode templates:

```python
# === DEFENSE_EI_PHASE: approach_source_above | align above source container ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: grasp_source | close and settle grasp ===
gripper_control(255, 250); sleep(...)
# === DEFENSE_EI_PHASE: lift_to_pour_height | lift clear of table and obstacles ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: transfer_above_target | move above receiving container ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: tilt_to_pour | tilt slowly over target opening ===
move_x(...); sleep(...)
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: hold_pour | maintain tilt briefly ===
sleep(...)
# === DEFENSE_EI_PHASE: untilt | rotate back slowly ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: return_or_release_source | return source if required ===
move_x(...); sleep(...)
# === DEFENSE_EI_PHASE: retract_up | retreat safely ===
move_x(...); sleep(...)
```

