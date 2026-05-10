---
name: supervisor-contract
description: JSON-only atomic task information contract for defense_ei_agents supervisor.
tags: [defense_ei_agents, supervisor, robotics, json-output]
---

# Supervisor Contract

Return exactly one JSON object:

```
{
  "status": "in_progress" | "completed",
  "summary": "<short summary>",
  "atomic_task_info": [
    {
      "id": 1,
      "description": "<planner atomic task>",
      "primitive_skill": "pick_place|push|pull|press|open|close|pour",
      "operated_object": "<object name or null>",
      "source_object": "<object name or null>",
      "target_object": "<object name or null>",
      "task_related_objects": ["<other object name needed for YOLO detection, clearance, or judging>", "..."],
      "grasp_point_reference": "<qualitative visual reference for where/how to grasp or contact the operated object>",
      "target_point_reference": "<qualitative visual reference for the desired target point/region/contact point>",
      "target_state": "<observable desired end state>",
      "motion_type": "translation|rotation|hybrid",
      "constraints": ["<clearance/speed/contact/friction note>", "..."],
      "done_criteria": "<visual criterion for this atomic task>",
      "failure_risks": ["<risk>", "..."]
    }
  ]
}
```

Rules:
- Preserve planner order and ids.
- One `atomic_task_info` entry per planner atomic task.
- Use camera-observable `target_state` and `done_criteria`.
- Provide specific object names in `operated_object`, `target_object`, and
  `task_related_objects`; coder and judger use these strings as YOLO detection
  labels and receive gripper-relative distance estimates from those detections.
- Always provide `grasp_point_reference` and `target_point_reference` as
  qualitative visual constraints for the coder. Examples: "grasp the cup by its
  upper rim/edge", "pinch the block near its top side faces", "push on the rear
  face opposite the target", "place into the bowl opening near its center".
- Point references are not calibrated coordinates. Do not invent numeric poses;
  describe visible affordances, stable contact areas, openings, rims, handles,
  faces, centers, and safety clearances.
- Do not invent calibrated coordinates unless provided by the user.
- Keep the contract independent of any evobody paths, tools, or memory.
