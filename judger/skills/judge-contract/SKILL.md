---
name: judge-contract
description: Strict JSON output contract for defense_ei_agents atomic-task judging.
tags: [defense_ei_agents, judge, json-output]
---

# Judge Contract

Return exactly this JSON object:

```
{"task_result": "SUCCESS" | "FAIL", "analysis": "<concise reason; empty string allowed if SUCCESS>"}
```

Rules:
- Judge one atomic task at a time.
- `global_image` is the global camera view of the whole scene.
- `wrist_image` is the camera view from the gripper; part of the gripper may
  appear along the bottom of the image.
- `scene_state_brief`, when present, is supporting RGB-D evidence with
  approximate object/grasp-region positions in the global camera frame, the
  current TCP pose in the robot base frame, and optional derived base/gripper
  object fields when transforms are available. YOLO detections cover all scene
  objects unless a manual label filter was supplied.
- When phase manifests are provided, judge only the current phase using that
  phase's success criteria and attached global/wrist RGB images. Stop at the
  first failed phase and report that phase's concrete failure and correction
  target.
- For a non-final phase, `SUCCESS` means the current phase goal appears achieved
  and validation may continue. For the final phase, `SUCCESS` means the full
  atomic task's target state and done criteria are satisfied.
- For every phase after `look_at_operated_object`, the operated object must
  remain visible in `wrist_image` unless it has already been intentionally
  released at the target; otherwise return `FAIL`.
- For `pick_place`, require the post-phase `wrist_image` to contain the operated
  object only when the current phase executed `look_at_operated_object` or when
  scene_state/scene_state_brief already detected the operated object. If YOLO did
  not detect the operated object before coding and the code skipped gaze, judge
  the basic phase motion/gripper requirement without failing solely for missing
  gaze.
- When no phase manifest is provided, compare the final global/wrist RGB images
  against the atomic task's target state and done criteria.
- Treat execution errors as failure unless the visual state clearly satisfies the
  atomic task anyway.
- Use scene_state distances to support analysis of alignment, object motion, and
  distance-to-target changes, but do not overrule clear RGB evidence with
  low-confidence or missing depth estimates.
- If `grasp_point_reference` or `target_point_reference` is present in atomic
  task info, include it in phase judgment. Grasp/contact should match the named
  affordance, and the final placement/contact/motion endpoint should match the
  target reference.
- For failure, name the concrete problem: wrong object, missed target, no visible
  motion, incomplete press/open/close/pour, object dropped, gripper still holding,
  unsafe collision, or occlusion.
- Do not output markdown or prose outside the JSON object.
