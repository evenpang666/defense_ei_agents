"""Run the defense_ei_agents real-world DefenseAgent workflow on a UR7e setup."""

from __future__ import annotations

import argparse
import ast
import asyncio
import json
import os
import re
import sys
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence

import imageio.v2 as imageio
import yaml


SCRIPT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_ROOT


def _prepend_sys_path(path: Path) -> None:
    path_text = str(path)
    if path.exists() and path_text not in sys.path:
        sys.path.insert(0, path_text)


_prepend_sys_path(SCRIPT_ROOT)
for defense_agent_root in (
    Path(os.environ["DEFENSE_AGENT_ROOT"]).expanduser()
    if os.environ.get("DEFENSE_AGENT_ROOT")
    else None,
    PROJECT_ROOT / "third_party" / "DefenseAgent",
    PROJECT_ROOT.parent / "third_party" / "DefenseAgent",
    PROJECT_ROOT.parent / "defense_EI" / "third_party" / "DefenseAgent",
):
    if defense_agent_root is not None:
        _prepend_sys_path(defense_agent_root.resolve())

from ur7e_controller import ROBOT_IP, UR7eVectorController, make_real_runtime_api  # noqa: E402


DEFAULT_PROFILE_ROOT = PROJECT_ROOT
DEFAULT_PLANNER_PROFILE = DEFAULT_PROFILE_ROOT / "planner" / "profile.yaml"
DEFAULT_SUPERVISOR_PROFILE = DEFAULT_PROFILE_ROOT / "supervisor" / "profile.yaml"
DEFAULT_CODER_PROFILE = DEFAULT_PROFILE_ROOT / "coder" / "profile.yaml"
DEFAULT_JUDGER_PROFILE = DEFAULT_PROFILE_ROOT / "judger" / "profile.yaml"
DEFAULT_LOG_ROOT = PROJECT_ROOT / "logs" / "defense_agent_real"
ALLOWED_RUNTIME_CALLS = {
    "gripper_control",
    "move_x",
    "move_y",
    "move_z",
    "rotate_x",
    "rotate_y",
    "rotate_z",
    "sleep",
}
ALLOWED_BUILTIN_CALLS = {
    "range",
    "len",
    "min",
    "max",
    "abs",
    "float",
    "int",
    "str",
    "list",
    "dict",
    "tuple",
    "bool",
    "enumerate",
    "zip",
    "round",
    "sum",
    "isinstance",
}
FORBIDDEN_RUNTIME_CALLS = {
    "pick_and_place",
    "pick_place",
    "push",
    "pull",
    "press",
    "open",
    "close",
    "pour",
    "move_to",
    "move_ee",
    "ee_pose",
    "print",
}


@dataclass
class RealDefenseOutputs:
    plan: dict[str, Any]
    atomic_task_info: dict[str, Any]
    attempts: list[dict[str, Any]]
    completed: bool
    code: str
    execution: dict[str, Any]
    judge: dict[str, Any]
    raw_planner_text: str
    raw_supervisor_text: str
    raw_coder_text: str
    raw_judger_text: str


@dataclass(frozen=True)
class GeneratedPhaseBlock:
    index: int
    slug: str
    goal: str
    start_line: int
    end_line: int
    code: str


@dataclass(frozen=True)
class PrimitivePhaseSpec:
    index: int
    slug: str
    goal: str
    success_criteria: str


def _log(msg: str) -> None:
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{stamp}] [evaluate_defense_agent_real] {msg}", flush=True)


def _list_realsense_cameras() -> list[dict[str, str]]:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise ImportError(
            "Missing dependency 'pyrealsense2'. Install Intel RealSense SDK Python bindings first."
        ) from exc
    except Exception as exc:
        raise RuntimeError(
            "Unable to initialize pyrealsense2. Close Intel RealSense Viewer or any other "
            "camera process, check Windows camera privacy permissions, then try again."
        ) from exc

    cameras: list[dict[str, str]] = []
    try:
        devices = list(rs.context().query_devices())
    except Exception as exc:
        raise RuntimeError(
            "Unable to query Intel RealSense devices. Close Intel RealSense Viewer or any "
            "other camera process, check Windows camera privacy permissions, then try again."
        ) from exc

    for dev in devices:
        entry: dict[str, str] = {}
        for key, label in (
            (rs.camera_info.name, "name"),
            (rs.camera_info.serial_number, "serial"),
            (rs.camera_info.firmware_version, "firmware"),
            (rs.camera_info.usb_type_descriptor, "usb"),
        ):
            try:
                entry[label] = dev.get_info(key)
            except Exception:
                pass
        cameras.append(entry)
    return cameras


class RealSenseRGBPair:
    PLACEHOLDER_SERIALS = {
        "GLOBAL_SERIAL",
        "FRONT_SERIAL",
        "WRIST_SERIAL",
        "SERIAL",
        "CAMERA_SERIAL",
    }

    def __init__(
        self,
        serials: Sequence[str],
        width: int,
        height: int,
        fps: int,
        warmup_frames: int,
    ) -> None:
        try:
            import numpy as np
            import pyrealsense2 as rs
        except ImportError as exc:
            raise ImportError(
                "Missing dependency 'pyrealsense2' or 'numpy'. "
                "Install Intel RealSense SDK Python bindings first."
            ) from exc

        self._np = np
        self._rs = rs
        self._serials = [s.strip() for s in serials if s.strip()]
        self._width = int(width)
        self._height = int(height)
        self._fps = int(fps)
        self._warmup_frames = max(0, int(warmup_frames))
        self._pipelines: list[Any] = []

    def __enter__(self) -> "RealSenseRGBPair":
        placeholders = [
            serial
            for serial in self._serials
            if serial.upper() in self.PLACEHOLDER_SERIALS or serial.upper().endswith("_SERIAL")
        ]
        if placeholders:
            detected = self._describe_devices()
            raise RuntimeError(
                "RealSense camera serial placeholders were passed as real serials: "
                f"{', '.join(placeholders)}. Replace them with actual serial numbers, "
                "omit --camera-serials to auto-discover cameras, or run with --list-cameras. "
                f"Detected devices: {detected or 'none'}"
            )

        serials = self._serials or self._discover_serials()
        if not serials:
            raise RuntimeError(
                "No Intel RealSense devices were found. Check USB/power, confirm the camera "
                "appears in Intel RealSense Viewer, or run this script with --list-cameras."
            )
        if len(serials) == 1:
            _log("Only one RealSense RGB stream found; global image will also be used as wrist image.")

        for serial in serials[:2]:
            pipeline = self._rs.pipeline()
            cfg = self._rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(
                self._rs.stream.color,
                self._width,
                self._height,
                self._rs.format.rgb8,
                self._fps,
            )
            try:
                pipeline.start(cfg)
            except RuntimeError as exc:
                detected = self._describe_devices()
                raise RuntimeError(
                    f"Failed to start RealSense RGB stream for serial '{serial}'. "
                    f"Detected devices: {detected or 'none'}. "
                    "If you copied the README example, replace GLOBAL_SERIAL,WRIST_SERIAL "
                    "with real camera serial numbers or omit --camera-serials to auto-discover."
                ) from exc
            self._pipelines.append(pipeline)

        for _ in range(self._warmup_frames):
            self.capture()
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        for pipeline in self._pipelines:
            try:
                pipeline.stop()
            except Exception:
                pass
        self._pipelines.clear()

    def _discover_serials(self) -> list[str]:
        ctx = self._rs.context()
        out: list[str] = []
        try:
            devices = list(ctx.query_devices())
        except Exception as exc:
            raise RuntimeError(
                "Unable to query Intel RealSense devices. Close Intel RealSense Viewer or any "
                "other camera process, check Windows camera privacy permissions, then try again."
            ) from exc
        for dev in devices:
            try:
                out.append(dev.get_info(self._rs.camera_info.serial_number))
            except Exception:
                continue
        return out

    def _describe_devices(self) -> str:
        ctx = self._rs.context()
        devices: list[str] = []
        try:
            queried_devices = list(ctx.query_devices())
        except Exception:
            return "unable to query devices"
        for dev in queried_devices:
            try:
                serial = dev.get_info(self._rs.camera_info.serial_number)
            except Exception:
                serial = "unknown-serial"
            try:
                name = dev.get_info(self._rs.camera_info.name)
            except Exception:
                name = "RealSense"
            devices.append(f"{name} ({serial})")
        return ", ".join(devices)

    def capture(self) -> tuple[Any, Any]:
        if not self._pipelines:
            raise RuntimeError("RealSense pipelines are not started.")

        images: list[Any] = []
        for pipeline in self._pipelines:
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                raise RuntimeError("Failed to read a RealSense RGB frame.")
            images.append(self._np.asarray(color.get_data(), dtype=self._np.uint8).copy())
        if len(images) == 1:
            images.append(images[0].copy())
        return images[0], images[1]


def _resolve_path(path_text: str | Path) -> Path:
    path = Path(str(path_text)).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def _save_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _save_text(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(payload, encoding="utf-8")


def _read_optional_text(path: Path, *, max_chars: int = 12000) -> str:
    try:
        text = path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""
    if len(text) <= max_chars:
        return text
    return text[:max_chars].rstrip() + "\n\n[truncated]"


def _prepare_real_profile_copy(profile_path: Path, log_dir: Path, role: str) -> Path:
    """Write a temporary profile with paths resolved inside the run directory."""
    profile_path = _resolve_path(profile_path)
    raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or not isinstance(raw.get("agent"), dict):
        raise ValueError(f"Invalid DefenseAgent profile: {profile_path}")

    agent = raw["agent"]
    tools = agent.get("tools")
    if not isinstance(tools, dict):
        return profile_path

    source_dir = profile_path.parent

    def abs_ref(ref: str) -> str:
        p = Path(str(ref))
        return str(p if p.is_absolute() else (source_dir / p).resolve())

    tools["skills"] = [abs_ref(ref) for ref in tools.get("skills", [])]
    tools["python"] = [
        abs_ref(ref.rsplit(":", 1)[0]) + ":" + ref.rsplit(":", 1)[1]
        for ref in tools.get("python", [])
        if ":" in ref
    ]
    tools["mcp"] = []
    tools["allow_skill_execution"] = False
    if role == "coder":
        # Some OpenAI-compatible endpoints occasionally emit invalid JSON for
        # code-bearing tool arguments. The orchestrator validates code locally,
        # so the real-robot coder is more reliable as direct code generation.
        tools["skills"] = []
        tools["python"] = []

    prompt = agent.get("prompt")
    if isinstance(prompt, dict) and isinstance(prompt.get("path"), str):
        prompt_path = Path(prompt["path"])
        if not prompt_path.is_absolute():
            prompt["path"] = str((source_dir / prompt_path).resolve())

    out_path = log_dir / f"real_{role}_profile.yaml"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return out_path


def _extract_json_object(raw_text: str) -> dict[str, Any]:
    text = (raw_text or "").strip()
    if not text:
        raise ValueError("empty model response")

    fenced = re.search(r"```json\s*(.*?)```", text, flags=re.IGNORECASE | re.DOTALL)
    if fenced:
        payload = json.loads(fenced.group(1).strip())
        if isinstance(payload, dict):
            return payload
        raise ValueError("json payload is not an object")

    left = text.find("{")
    right = text.rfind("}")
    if left != -1 and right != -1 and right > left:
        payload = json.loads(text[left : right + 1])
        if isinstance(payload, dict):
            return payload
    raise ValueError("no json object found in model response")


def _extract_python_code(raw_text: str) -> str:
    text = (raw_text or "").strip()
    if not text:
        raise ValueError("empty model response")

    fenced = re.search(r"```python\s*(.*?)```", text, flags=re.IGNORECASE | re.DOTALL)
    if fenced:
        return fenced.group(1).strip()

    generic = re.search(r"```(.*?)```", text, flags=re.DOTALL)
    if generic:
        return generic.group(1).strip()
    return text


def _resolve_memory_backend_from_profile(
    profile_path: Path,
    *,
    embedding_model: str,
    embedding_api_key: str | None,
    embedding_base_url: str | None,
    embedding_dims: int,
) -> MemoryBackendConfig:
    from DefenseAgent.config import AgentProfile
    from DefenseAgent.memory import MemoryBackendConfig

    profile = AgentProfile.from_yaml(profile_path)
    llm_provider = (profile.llm.provider or "").strip().lower()
    llm_model = (profile.llm.model or "").strip()
    llm_api_key = (profile.llm.api_key or "").strip()
    llm_base_url = (profile.llm.base_url or "").strip()

    if not llm_provider or not llm_model or not llm_api_key:
        raise ValueError(
            f"coder profile llm config incomplete in {profile_path}; "
            "need provider/model/api_key for mem0 backend."
        )

    return MemoryBackendConfig(
        llm_provider=llm_provider,
        llm_api_key=llm_api_key,
        llm_model=llm_model,
        llm_base_url=llm_base_url,
        embedding_provider="openai",
        embedding_api_key=(embedding_api_key or "").strip() or llm_api_key,
        embedding_model=embedding_model.strip(),
        embedding_base_url=(embedding_base_url or "").strip() or llm_base_url,
        embedding_dims=int(embedding_dims),
    )


def _build_react_agent(
    profile_path: Path,
    *,
    enable_memory: bool,
    memory_backend: MemoryBackendConfig | None = None,
) -> ReActAgent:
    try:
        from DefenseAgent import AgentConfig, ReActAgent
    except ModuleNotFoundError as exc:
        missing = exc.name or "unknown"
        raise ModuleNotFoundError(
            f"Missing Python dependency while importing DefenseAgent: {missing!r}. "
            "Install DefenseAgent with its dependencies, for example: "
            "pip install -e third_party/DefenseAgent "
            "or set DEFENSE_AGENT_ROOT to the DefenseAgent source tree and install "
            "that package in the active environment."
        ) from exc

    return ReActAgent(
        AgentConfig(
            profile=profile_path,
            use_memory=enable_memory,
            use_reflection=enable_memory,
            use_compressor=enable_memory,
            memory_backend=memory_backend,
        )
    )


def _validate_generated_code_runtime_calls(tree: ast.AST) -> dict[str, Any]:
    called_names: list[str] = []
    forbidden_hits: list[str] = []
    unknown_calls: list[str] = []
    allowed = ALLOWED_RUNTIME_CALLS | ALLOWED_BUILTIN_CALLS

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Name):
            name = node.func.id
            called_names.append(name)
            if name in FORBIDDEN_RUNTIME_CALLS:
                forbidden_hits.append(name)
            elif name not in allowed:
                unknown_calls.append(name)
        elif isinstance(node.func, ast.Attribute):
            unknown_calls.append(f"attribute_call:{node.func.attr}")

    return {
        "ok": not forbidden_hits and not unknown_calls,
        "called_names": sorted(set(called_names)),
        "forbidden_runtime_calls": sorted(set(forbidden_hits)),
        "unknown_calls": sorted(set(unknown_calls)),
    }


def _check_generated_code(code: str) -> dict[str, Any]:
    lowered = (code or "").lower()
    forbidden_text = [
        tok
        for tok in (
            "quat",
            "quaternion",
            "wxyz",
            "xyzw",
            "ee_pose",
            "move_ee",
            "move_to",
            "print(",
            "pick_and_place",
            "pick_place",
            "push(",
            "pull(",
            "press(",
            "open(",
            "close(",
            "pour(",
        )
        if tok in lowered
    ]
    if forbidden_text:
        raise ValueError(f"generated code contains forbidden tokens: {forbidden_text}")

    tree = ast.parse(code, mode="exec")
    hits: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            hits.append("def")
        elif isinstance(node, ast.ClassDef):
            hits.append("class")
        elif isinstance(node, ast.Import):
            hits.append("import")
        elif isinstance(node, ast.ImportFrom):
            hits.append("from")
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in {"exec", "eval"}:
            hits.append(node.func.id)
        elif isinstance(node, ast.Name) and node.id in {"subprocess", "os", "sys"}:
            hits.append(node.id)
    if hits:
        raise ValueError(f"generated code contains forbidden syntax: {sorted(set(hits))}")
    runtime_report = _validate_generated_code_runtime_calls(tree)
    if not runtime_report["ok"]:
        raise ValueError(f"generated code calls unsupported runtime tools: {runtime_report}")
    compile(tree, "<real_robot_generated_code>", "exec")
    return runtime_report


def _safe_real_exec_globals(controller: UR7eVectorController) -> dict[str, Any]:
    safe_builtins = {
        "range": range,
        "len": len,
        "min": min,
        "max": max,
        "abs": abs,
        "float": float,
        "int": int,
        "str": str,
        "list": list,
        "dict": dict,
        "tuple": tuple,
        "bool": bool,
        "enumerate": enumerate,
        "zip": zip,
        "round": round,
        "sum": sum,
        "isinstance": isinstance,
        "__name__": "__real_atomic__",
    }
    exec_globals: dict[str, Any] = {"__builtins__": safe_builtins}
    exec_globals.update(make_real_runtime_api(controller))
    return exec_globals


def _sanitize_phase_slug(value: str, fallback: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9_.-]+", "_", str(value or "").strip()).strip("._-")
    return slug[:80] or fallback


def _parse_generated_phase_blocks(code: str) -> list[GeneratedPhaseBlock]:
    phase_re = re.compile(
        r"^\s*#\s*===\s*DEFENSE_EI_PHASE:\s*(?P<slug>[^|=]+?)"
        r"(?:\s*\|\s*(?P<goal>.*?))?\s*===\s*$"
    )
    lines = code.splitlines()
    markers: list[tuple[int, str, str]] = []
    for line_no, line in enumerate(lines, start=1):
        match = phase_re.match(line)
        if not match:
            continue
        fallback = f"phase_{len(markers) + 1:02d}"
        slug = _sanitize_phase_slug(match.group("slug"), fallback)
        goal = str(match.group("goal") or "").strip()
        markers.append((line_no, slug, goal))

    if not markers:
        return [
            GeneratedPhaseBlock(
                index=1,
                slug="full_execution",
                goal="generated code without DEFENSE_EI_PHASE markers",
                start_line=1,
                end_line=max(1, len(lines)),
                code=code,
            )
        ]

    blocks: list[GeneratedPhaseBlock] = []
    for marker_index, (start_line, slug, goal) in enumerate(markers):
        next_start = markers[marker_index + 1][0] if marker_index + 1 < len(markers) else len(lines) + 1
        block_start = 1 if marker_index == 0 else start_line
        block_code = "\n".join(lines[block_start - 1 : next_start - 1])
        blocks.append(
            GeneratedPhaseBlock(
                index=len(blocks) + 1,
                slug=slug,
                goal=goal,
                start_line=block_start,
                end_line=next_start - 1,
                code=block_code,
            )
        )
    return blocks


def _resize_image_width(image: Any, target_width: int = 128) -> Any:
    import numpy as np

    arr = np.asarray(image)
    if arr.ndim < 2:
        return arr
    height, width = int(arr.shape[0]), int(arr.shape[1])
    if width <= 0 or height <= 0 or width == int(target_width):
        return arr
    target_width = max(1, int(target_width))
    target_height = max(1, int(round(height * (target_width / width))))
    y_idx = np.clip(np.round(np.linspace(0, height - 1, target_height)).astype(int), 0, height - 1)
    x_idx = np.clip(np.round(np.linspace(0, width - 1, target_width)).astype(int), 0, width - 1)
    return arr[y_idx][:, x_idx]


def _resize_image_file_width(path: Path, target_width: int = 128) -> None:
    image = imageio.imread(path)
    resized = _resize_image_width(image, target_width=target_width)
    imageio.imwrite(path, resized)


def _execute_real_code(
    *,
    code: str,
    controller: UR7eVectorController | None,
) -> dict[str, Any]:
    _check_generated_code(code)
    if controller is None:
        raise RuntimeError("controller is required for real execution")

    exec_globals = _safe_real_exec_globals(controller)

    started = time.time()
    try:
        exec(compile(code, "<real_robot_generated_code>", "exec"), exec_globals)
        return {
            "ok": True,
            "signal": "REAL_EXECUTION_OK",
            "elapsed_s": round(time.time() - started, 3),
        }
    except Exception:
        return {
            "ok": False,
            "signal": "REAL_EXECUTION_FAILED",
            "elapsed_s": round(time.time() - started, 3),
            "error": traceback.format_exc(),
        }


def _execute_real_code_by_phase(
    *,
    code: str,
    controller: UR7eVectorController | None,
    phase_capture_fn: Callable[[GeneratedPhaseBlock], tuple[Path, Path]] | None,
) -> dict[str, Any]:
    _check_generated_code(code)
    if controller is None:
        raise RuntimeError("controller is required for real execution")

    exec_globals = _safe_real_exec_globals(controller)
    phases = _parse_generated_phase_blocks(code)
    phase_reports: list[dict[str, Any]] = []
    started = time.time()

    for phase in phases:
        phase_started = time.time()
        phase_report: dict[str, Any] = {
            "index": phase.index,
            "slug": phase.slug,
            "goal": phase.goal,
            "start_line": phase.start_line,
            "end_line": phase.end_line,
        }
        try:
            exec(
                compile(phase.code, f"<real_robot_generated_code:{phase.slug}>", "exec"),
                exec_globals,
            )
            phase_report.update(
                {
                    "ok": True,
                    "signal": "REAL_PHASE_EXECUTION_OK",
                    "elapsed_s": round(time.time() - phase_started, 3),
                }
            )
        except Exception:
            phase_report.update(
                {
                    "ok": False,
                    "signal": "REAL_PHASE_EXECUTION_FAILED",
                    "elapsed_s": round(time.time() - phase_started, 3),
                    "error": traceback.format_exc(),
                }
            )

        if phase_capture_fn is not None:
            try:
                global_path, wrist_path = phase_capture_fn(phase)
                phase_report["global_image"] = str(global_path)
                phase_report["wrist_image"] = str(wrist_path)
                phase_report["global_rgb"] = str(global_path)
                phase_report["wrist_rgb"] = str(wrist_path)
                phase_report["image_width_px"] = 128
            except Exception:
                phase_report["capture_error"] = traceback.format_exc()

        phase_reports.append(phase_report)
        if not bool(phase_report.get("ok")):
            return {
                "ok": False,
                "signal": "REAL_EXECUTION_FAILED",
                "elapsed_s": round(time.time() - started, 3),
                "failed_phase": phase.slug,
                "phases": phase_reports,
                "error": str(phase_report.get("error", "")),
            }

    return {
        "ok": True,
        "signal": "REAL_EXECUTION_OK",
        "elapsed_s": round(time.time() - started, 3),
        "phases": phase_reports,
    }


def _snapshot_robot_initial_state(
    controller: UR7eVectorController,
) -> dict[str, Any]:
    return {
        "joints": controller.get_current_joints(),
        "gripper_open_ratio": controller.get_gripper_open_ratio(),
    }


def _restore_robot_initial_state(
    controller: UR7eVectorController | None,
    snapshot: dict[str, Any],
) -> dict[str, Any]:
    if controller is None:
        return {"ok": False, "signal": "REAL_RESTORE_NO_CONTROLLER"}
    try:
        joints = snapshot.get("joints")
        if isinstance(joints, list) and len(joints) == 6:
            controller.move_joints(joints, acceleration=0.8, velocity=0.25)
        gripper_ratio = snapshot.get("gripper_open_ratio")
        if isinstance(gripper_ratio, (int, float)) and controller.is_gripper_available():
            controller.set_gripper(float(gripper_ratio))
        return {"ok": True, "signal": "REAL_RESTORE_OK"}
    except Exception:
        return {
            "ok": False,
            "signal": "REAL_RESTORE_FAILED",
            "error": traceback.format_exc(),
        }


def _capture_or_copy_images(
    *,
    capture_images_fn: Callable[[Path, Path], None] | None,
    target_global: Path,
    target_wrist: Path,
    resize_width: int | None = None,
) -> None:
    target_global.parent.mkdir(parents=True, exist_ok=True)
    if capture_images_fn is not None:
        capture_images_fn(target_global, target_wrist)
        if resize_width is not None:
            _resize_image_file_width(target_global, resize_width)
            _resize_image_file_width(target_wrist, resize_width)
        return
    raise RuntimeError("camera capture callback is required for real execution")


def _atomic_infos_from_supervisor(payload: dict[str, Any]) -> list[dict[str, Any]]:
    infos = payload.get("atomic_task_info", payload.get("atomic_tasks", []))
    if not isinstance(infos, list):
        raise ValueError("supervisor output must include atomic_task_info list")
    return [info for info in infos if isinstance(info, dict)]


def _is_success_judge(judge: dict[str, Any]) -> bool:
    return str(judge.get("task_result", "")).strip().upper() == "SUCCESS"


def _primitive_name(atomic_info: dict[str, Any]) -> str:
    return str(
        atomic_info.get("primitive_skill")
        or atomic_info.get("primitive")
        or atomic_info.get("action")
        or ""
    ).strip().lower()


def _primitive_phase_specs(atomic_info: dict[str, Any]) -> list[PrimitivePhaseSpec]:
    primitive = _primitive_name(atomic_info)
    phase_specs_by_primitive: dict[str, list[tuple[str, str, str]]] = {
        "pick_place": [
            (
                "point_down",
                "orient gripper downwards",
                "The wrist/global views should show the gripper oriented for a vertical-down approach to the table/object, with no object contact or collision.",
            ),
            (
                "approach_source_above",
                "align above source using small wrist-frame corrections",
                "The gripper should be safely above the operated object, laterally aligned for grasping while still clear of the object and surrounding clutter.",
            ),
            (
                "open_gripper",
                "prepare for grasp",
                "The gripper fingers should be visibly open enough to fit around the operated object, and the gripper should remain above the source without disturbing it.",
            ),
            (
                "descend_to_source",
                "slow vertical descent to source grasp height",
                "The gripper should descend vertically to the object's grasp height, remain aligned around or immediately beside the object, and avoid pushing it away.",
            ),
            (
                "grasp_source",
                "close on object and settle",
                "The gripper should be closed on the correct object with a stable grasp; the object should not be visibly missed, knocked over, or squeezed out.",
            ),
            (
                "lift_to_safe_height",
                "lift before horizontal transfer",
                "The grasped object should rise with the gripper to a safe carry height, clear of the table, source area, rims, and any intervening obstacles.",
            ),
            (
                "transfer_above_target",
                "horizontal transfer at safe height",
                "With the object still held, the gripper should move horizontally to a position above the target region/opening while staying at safe height.",
            ),
            (
                "descend_to_target",
                "slow vertical placement",
                "The held object should descend vertically into or onto the target placement region without hitting rims, obstacles, or missing the target.",
            ),
            (
                "release_at_target",
                "release object",
                "The gripper should open and leave the correct object at the target position; the object should no longer be held and should not roll or fall out of place.",
            ),
            (
                "retract_up",
                "retreat vertically",
                "The gripper should retract vertically to a safe height without disturbing the placed object; the full pick_place atomic task target state should now hold.",
            ),
        ],
        "push": [
            (
                "approach_push_start",
                "align gripper before contact",
                "The gripper should be near the intended push start pose, aligned behind or beside the correct object along the intended push direction, without contacting or moving it yet.",
            ),
            (
                "set_push_contact_shape",
                "make a stable pushing surface",
                "The gripper should form a stable pusher shape such as closed or partially closed fingers, with no unwanted object motion.",
            ),
            (
                "descend_or_advance_to_contact",
                "slow contact approach",
                "The gripper should make gentle, controlled contact with the correct object at the intended push face without tipping, lifting, or displacing it prematurely.",
            ),
            (
                "push_slowly",
                "translate object in small increments",
                "The correct object should move in the intended direction by the required amount while staying stable and avoiding collisions with unintended objects.",
            ),
            (
                "retract_from_contact",
                "back away from object",
                "The gripper should move away from the object after the push without dragging it back; the pushed object should remain at the target state for the atomic task.",
            ),
        ],
        "pull": [
            (
                "approach_pull_contact",
                "align to pull feature",
                "The gripper should be aligned with the object's pullable feature or contact side, close enough to engage it without disturbing the object.",
            ),
            (
                "grasp_or_hook",
                "secure contact before pulling",
                "The gripper should visibly grasp, hook, or otherwise secure the correct object/feature so it can be pulled without slipping immediately.",
            ),
            (
                "pull_slowly",
                "pull with short conservative increments",
                "The correct object should move in the intended pull direction by the required amount while remaining controlled and not colliding with unrelated objects.",
            ),
            (
                "release_if_grasped",
                "let go after pull",
                "If the object was grasped or hooked, the gripper should release it cleanly at the pulled target state without moving it back.",
            ),
            (
                "retract_from_contact",
                "retreat from manipulated object",
                "The gripper should retreat from the object without further contact; the pulled object should remain at the desired final state.",
            ),
        ],
        "press": [
            (
                "approach_press_target",
                "align to button or target surface",
                "The gripper should be aligned with the correct press target and close to it, with no unintended contact or activation yet.",
            ),
            (
                "set_press_contact_shape",
                "close gripper for stable contact",
                "The gripper should form a stable contact surface for pressing while remaining aligned with the correct target.",
            ),
            (
                "press_slowly",
                "press along observed normal in small increments",
                "The gripper should press the correct target along its apparent normal far enough to actuate or depress it, without sliding off or striking nearby objects.",
            ),
            (
                "hold_press",
                "keep contact briefly",
                "The gripper should maintain the press long enough for the target effect to register, with the target still visibly depressed or actuated if observable.",
            ),
            (
                "retract_from_press",
                "move back along approach direction",
                "The gripper should retract along the approach direction without additional disturbance; the pressed target should show the desired final state if visible.",
            ),
        ],
        "open": [
            (
                "approach_handle_or_lid",
                "align to articulated feature",
                "The gripper should be aligned with the handle, lid, edge, or other articulated feature without disturbing the object.",
            ),
            (
                "grasp_or_hook_articulation",
                "secure the feature",
                "The gripper should visibly grasp or hook the articulated feature securely enough to move it without slipping immediately.",
            ),
            (
                "articulate_slowly",
                "move through small arc increments",
                "The articulated object should move toward the open state through controlled increments without collision, overtravel, or loss of contact.",
            ),
            (
                "release_articulation",
                "release after target state is reached",
                "The gripper should release the feature only after the object appears open to the requested state, without letting it rebound closed.",
            ),
            (
                "retract_from_articulation",
                "retreat without bumping object",
                "The gripper should retreat clear of the articulated object; the object should remain open as required by the atomic task.",
            ),
        ],
        "close": [
            (
                "approach_handle_or_lid",
                "align to articulated feature",
                "The gripper should be aligned with the handle, lid, edge, or other articulated feature without disturbing the object.",
            ),
            (
                "grasp_or_hook_articulation",
                "secure the feature",
                "The gripper should visibly grasp or hook the articulated feature securely enough to move it without slipping immediately.",
            ),
            (
                "articulate_slowly",
                "move through small arc increments",
                "The articulated object should move toward the closed state through controlled increments without collision, overtravel, or loss of contact.",
            ),
            (
                "release_articulation",
                "release after target state is reached",
                "The gripper should release the feature only after the object appears closed to the requested state, without letting it rebound open.",
            ),
            (
                "retract_from_articulation",
                "retreat without bumping object",
                "The gripper should retreat clear of the articulated object; the object should remain closed as required by the atomic task.",
            ),
        ],
        "pour": [
            (
                "approach_source_above",
                "align above source container",
                "The gripper should be safely above and aligned with the source container or object to be poured, without contacting or disturbing it yet.",
            ),
            (
                "grasp_source",
                "close and settle grasp",
                "The gripper should hold the correct source container securely enough for lifting and tilting, without crushing, slipping, or tipping it unintentionally.",
            ),
            (
                "lift_to_pour_height",
                "lift clear of table and obstacles",
                "The grasped source should lift to a safe pour height, clear of the table, nearby objects, and the receiving container rim.",
            ),
            (
                "transfer_above_target",
                "move above receiving container",
                "The source should remain held while moving to a controlled pose above the receiving target/opening, without spilling or colliding.",
            ),
            (
                "tilt_to_pour",
                "tilt slowly over target opening",
                "The source should tilt over the receiving target enough for pouring while remaining positioned over the opening and avoiding spills outside the target.",
            ),
            (
                "hold_pour",
                "maintain tilt briefly",
                "The source should remain tilted long enough for the intended contents to transfer into the receiving target if visible, without losing grasp stability.",
            ),
            (
                "untilt",
                "rotate back slowly",
                "The source should rotate back toward upright while still held securely, with pouring stopped and no collision with the target rim.",
            ),
            (
                "return_or_release_source",
                "return or release source according to task",
                "The source should be returned, placed, or released according to the atomic task target state, with the gripper no longer applying unintended force.",
            ),
            (
                "retract_up",
                "retreat safely",
                "The gripper should retreat to a safe height without disturbing the source or receiving target; the full pour atomic task target state should hold.",
            ),
        ],
    }
    if primitive == "pick_and_place":
        primitive = "pick_place"
    entries = phase_specs_by_primitive.get(primitive, [])
    return [
        PrimitivePhaseSpec(index, slug, goal, success_criteria)
        for index, (slug, goal, success_criteria) in enumerate(entries, start=1)
    ]


def _phase_history_for_prompt(history: list[dict[str, Any]]) -> list[dict[str, Any]]:
    compact: list[dict[str, Any]] = []
    for item in history:
        compact.append(
            {
                "phase_index": item.get("phase_index"),
                "phase_slug": item.get("phase_slug"),
                "code_path": item.get("code_path"),
                "global_image": item.get("after_global_image", item.get("after_global_rgb")),
                "wrist_image": item.get("after_wrist_image", item.get("after_wrist_rgb")),
                "judge": item.get("judge"),
            }
        )
    return compact


def _validate_single_expected_phase(code: str, expected: PrimitivePhaseSpec) -> GeneratedPhaseBlock:
    phases = _parse_generated_phase_blocks(code)
    if len(phases) != 1:
        raise ValueError(f"generated code must contain exactly one phase block; found {len(phases)}")
    phase = phases[0]
    if phase.slug != expected.slug:
        raise ValueError(
            f"generated phase slug must be {expected.slug!r} for current stage; got {phase.slug!r}"
        )
    return GeneratedPhaseBlock(
        index=expected.index,
        slug=phase.slug,
        goal=phase.goal or expected.goal,
        start_line=phase.start_line,
        end_line=phase.end_line,
        code=phase.code,
    )


async def run_defense_agent_real_once(
    *,
    task: str,
    current_global_image: Path,
    current_wrist_image: Path,
    planner_profile: Path,
    supervisor_profile: Path,
    coder_profile: Path,
    judger_profile: Path,
    log_dir: Path,
    controller: UR7eVectorController,
    max_steps: int | None,
    max_attempts_per_atomic: int,
    embedding_model: str,
    embedding_api_key: str | None,
    embedding_base_url: str | None,
    embedding_dims: int,
    capture_images_fn: Callable[[Path, Path], None] | None = None,
) -> RealDefenseOutputs:
    del embedding_model, embedding_api_key, embedding_base_url, embedding_dims

    planner = _build_react_agent(planner_profile, enable_memory=False)
    supervisor = _build_react_agent(supervisor_profile, enable_memory=False)
    coder = _build_react_agent(coder_profile, enable_memory=False)
    judger = _build_react_agent(judger_profile, enable_memory=False)

    initial_robot_state = _snapshot_robot_initial_state(controller)
    _save_json(log_dir / "initial_robot_state.json", initial_robot_state)

    observation = {
        "environment": "real_robot",
        "robot": "UR7e",
        "cameras": {
            "global_image": str(current_global_image),
            "wrist_image": str(current_wrist_image),
            "global_rgb": str(current_global_image),
            "wrist_rgb": str(current_wrist_image),
        },
        "image_roles": {
            "global_image": "Global camera image showing the whole scene.",
            "wrist_image": (
                "Wrist camera image from the gripper viewpoint; part of the gripper "
                "may appear along the bottom of the image."
            ),
        },
        "runtime_note": (
            "This is a real-robot one-pass validation. The only visual state inputs "
            "are the global_image and wrist_image Intel RealSense D435i RGB images. "
            "Do not assume a MuJoCo scene or reconstructed object pose JSON exists."
        ),
    }
    coder_contracts = {
        "runtime_api": {
            "allowed_calls": sorted(ALLOWED_RUNTIME_CALLS),
            "allowed_builtins": sorted(ALLOWED_BUILTIN_CALLS),
            "forbidden_calls": sorted(FORBIDDEN_RUNTIME_CALLS),
            "move_x": "move_x(distance, velocity=0.04, acceleration=0.18); distance is millimeters in gripper/wrist-image +X.",
            "move_y": "move_y(distance, velocity=0.04, acceleration=0.18); distance is millimeters in gripper/wrist-image +Y.",
            "move_z": "move_z(distance, velocity=0.04, acceleration=0.18); distance is millimeters in gripper/wrist-image +Z.",
            "rotate_x": "rotate_x(angle_rad, velocity=0.04, acceleration=0.18); angle is radians about gripper/wrist-image +X.",
            "rotate_y": "rotate_y(angle_rad, velocity=0.04, acceleration=0.18); angle is radians about gripper/wrist-image +Y.",
            "rotate_z": "rotate_z(angle_rad, velocity=0.04, acceleration=0.18); angle is radians about gripper/wrist-image +Z.",
            "gripper_control": "gripper_control(value, delay); value 0 opens and 255 closes; delay is milliseconds.",
            "sleep": "sleep(seconds) pauses between hardware commands.",
            "wrist_frame": (
                "wrist image right is gripper +X, wrist image down is gripper +Y, "
                "and wrist camera viewing direction is gripper +Z."
            ),
        },
        "real_robot_code_contract": _read_optional_text(
            PROJECT_ROOT / "coder" / "skills" / "real-robot-code-contract" / "SKILL.md"
        ),
        "primitive_skill_contract": _read_optional_text(
            PROJECT_ROOT / "coder" / "skills" / "primitive-skill-contract" / "SKILL.md"
        ),
    }

    planner_task = (
        "DEFENSE_EI_AGENTS_REAL_PLANNING\n"
        "Task:\n"
        f"{task}\n\n"
        "Real observation JSON:\n"
        f"{json.dumps(observation, ensure_ascii=False, indent=2)}\n\n"
        "Decompose the task into primitive_skill-level atomic tasks. Return JSON only."
    )
    planner_result = await planner.run(
        planner_task,
        max_steps=max_steps,
        images=[current_global_image, current_wrist_image],
    )
    raw_planner_text = planner_result.final_answer
    plan = _extract_json_object(raw_planner_text)
    _save_json(log_dir / "defense_ei_plan.json", plan)
    _save_text(log_dir / "defense_ei_plan_raw.txt", raw_planner_text)

    if str(plan.get("status", "")).strip().lower() == "completed":
        return RealDefenseOutputs(
            plan=plan,
            atomic_task_info={"status": "completed", "summary": plan.get("summary", ""), "atomic_task_info": []},
            attempts=[],
            completed=True,
            code="",
            execution={"ok": True, "signal": "PLAN_ALREADY_COMPLETED"},
            judge={"task_result": "SUCCESS", "analysis": ""},
            raw_planner_text=raw_planner_text,
            raw_supervisor_text="",
            raw_coder_text="",
            raw_judger_text="",
        )

    supervisor_task = (
        "DEFENSE_EI_AGENTS_REAL_SUPERVISION\n"
        "Task:\n"
        f"{task}\n\n"
        "Planner JSON:\n"
        f"{json.dumps(plan, ensure_ascii=False, indent=2)}\n\n"
        "Real observation JSON:\n"
        f"{json.dumps(observation, ensure_ascii=False, indent=2)}\n\n"
        "Produce atomic_task_info for every planner atomic task. Return JSON only."
    )
    supervisor_result = await supervisor.run(
        supervisor_task,
        max_steps=max_steps,
        images=[current_global_image, current_wrist_image],
    )
    raw_supervisor_text = supervisor_result.final_answer
    atomic_task_info = _extract_json_object(raw_supervisor_text)
    _save_json(log_dir / "defense_ei_atomic_task_info.json", atomic_task_info)
    _save_text(log_dir / "defense_ei_atomic_task_info_raw.txt", raw_supervisor_text)

    atomic_infos = _atomic_infos_from_supervisor(atomic_task_info)
    if not atomic_infos:
        return RealDefenseOutputs(
            plan=plan,
            atomic_task_info=atomic_task_info,
            attempts=[],
            completed=True,
            code="",
            execution={"ok": True, "signal": "NO_ATOMIC_TASKS"},
            judge={"task_result": "SUCCESS", "analysis": ""},
            raw_planner_text=raw_planner_text,
            raw_supervisor_text=raw_supervisor_text,
            raw_coder_text="",
            raw_judger_text="",
        )

    attempts: list[dict[str, Any]] = []
    latest_global = current_global_image
    latest_wrist = current_wrist_image
    last_code = ""
    last_execution: dict[str, Any] = {}
    last_judge: dict[str, Any] = {}
    raw_coder_text = ""
    raw_judger_text = ""
    completed = True

    for atomic_index, atomic_info in enumerate(atomic_infos, start=1):
        phase_specs = _primitive_phase_specs(atomic_info)
        atomic_done = False
        phase_history: list[dict[str, Any]] = []
        if not phase_specs:
            phase_specs = [
                PrimitivePhaseSpec(
                    1,
                    "full_execution",
                    "complete the atomic task",
                    "The final global/wrist views should satisfy the atomic task target state and done criteria.",
                )
            ]

        for phase_spec in phase_specs:
            feedback: dict[str, Any] | None = None
            phase_done = False
            for attempt_index in range(1, max(1, int(max_attempts_per_atomic)) + 1):
                attempt_dir = (
                    log_dir
                    / f"atomic_{atomic_index:02d}"
                    / f"phase_{phase_spec.index:02d}_{phase_spec.slug}"
                    / f"attempt_{attempt_index:02d}"
                )
                attempt_dir.mkdir(parents=True, exist_ok=True)
                _save_json(attempt_dir / "atomic_task_info.json", atomic_info)
                _save_json(attempt_dir / "current_phase.json", phase_spec.__dict__)

                phase_context = {
                    "index": phase_spec.index,
                    "slug": phase_spec.slug,
                    "goal": phase_spec.goal,
                    "success_criteria": phase_spec.success_criteria,
                    "is_final_phase": phase_spec.index == len(phase_specs),
                    "required_phase_order": [spec.slug for spec in phase_specs],
                    "completed_phase_history": _phase_history_for_prompt(phase_history),
                }
                coder_task = (
                    "DEFENSE_EI_AGENTS_REAL_PHASE_CODING\n"
                    "The real-robot code contracts are embedded below. Do not call tools; "
                    "do not call simulation APIs. The orchestrator will validate syntax "
                    "and runtime calls after your answer.\n\n"
                    "Generate code for exactly the current phase, not the full atomic task. "
                    "The code must contain exactly one DEFENSE_EI_PHASE marker and its slug "
                    "must match current_phase.slug. After this phase code executes, the "
                    "orchestrator will capture global/wrist RGB images and ask the judger "
                    "whether this phase meets current_phase.success_criteria. Only after "
                    "phase SUCCESS will the next phase be requested.\n\n"
                    "Embedded coder contracts JSON:\n"
                    f"{json.dumps(coder_contracts, ensure_ascii=False, indent=2)}\n\n"
                    "High-level task:\n"
                    f"{task}\n\n"
                    "Current atomic task info JSON:\n"
                    f"{json.dumps(atomic_info, ensure_ascii=False, indent=2)}\n\n"
                    "Current phase JSON:\n"
                    f"{json.dumps(phase_context, ensure_ascii=False, indent=2)}\n\n"
                    "Current observation JSON:\n"
                    f"{json.dumps({'global_image': str(latest_global), 'wrist_image': str(latest_wrist), 'global_rgb': str(latest_global), 'wrist_rgb': str(latest_wrist), 'image_roles': observation['image_roles']}, ensure_ascii=False, indent=2)}\n\n"
                    "Prior judger feedback JSON for this same phase, or null:\n"
                    f"{json.dumps(feedback, ensure_ascii=False, indent=2)}\n\n"
                    "Return exactly one fenced Python code block and nothing else."
                )
                coder_result = await coder.run(
                    coder_task,
                    max_steps=max_steps,
                    images=[latest_global, latest_wrist],
                )
                raw_coder_text = coder_result.final_answer
                last_code = _extract_python_code(raw_coder_text)
                _save_text(attempt_dir / "real_phase_actions.py", last_code)
                _save_text(attempt_dir / "real_phase_actions_raw.txt", raw_coder_text)
                _save_text(log_dir / "real_atomic_actions.py", last_code)

                expected_phase: GeneratedPhaseBlock | None = None
                try:
                    expected_phase = _validate_single_expected_phase(last_code, phase_spec)
                    runtime_report = _check_generated_code(last_code)
                    syntax_report = {
                        "ok": True,
                        "signal": "SYNTAX_PHASE_AND_RUNTIME_TOOL_CHECK_OK",
                        "runtime_tool_check": runtime_report,
                    }
                except Exception:
                    syntax_report = {
                        "ok": False,
                        "signal": "SYNTAX_PHASE_OR_RUNTIME_TOOL_CHECK_FAILED",
                        "error": traceback.format_exc(),
                    }
                _save_json(attempt_dir / "syntax_check.json", syntax_report)

                phase_global = attempt_dir / f"phase_{phase_spec.index:02d}_{phase_spec.slug}_global_rgb.png"
                phase_wrist = attempt_dir / f"phase_{phase_spec.index:02d}_{phase_spec.slug}_wrist_rgb.png"
                if not bool(syntax_report.get("ok")):
                    last_execution = {"ok": False, "signal": "SYNTAX_PHASE_OR_RUNTIME_TOOL_CHECK_FAILED"}
                    _capture_or_copy_images(
                        capture_images_fn=capture_images_fn,
                        target_global=phase_global,
                        target_wrist=phase_wrist,
                        resize_width=128,
                    )
                else:
                    last_execution = _execute_real_code(
                        code=last_code,
                        controller=controller,
                    )
                    _capture_or_copy_images(
                        capture_images_fn=capture_images_fn,
                        target_global=phase_global,
                        target_wrist=phase_wrist,
                        resize_width=128,
                    )
                    last_execution["phases"] = [
                        {
                            "index": phase_spec.index,
                            "slug": phase_spec.slug,
                            "goal": expected_phase.goal if expected_phase else phase_spec.goal,
                            "success_criteria": phase_spec.success_criteria,
                            "ok": bool(last_execution.get("ok")),
                            "global_image": str(phase_global),
                            "wrist_image": str(phase_wrist),
                            "global_rgb": str(phase_global),
                            "wrist_rgb": str(phase_wrist),
                            "image_width_px": 128,
                            "error": last_execution.get("error"),
                        }
                    ]
                _save_json(attempt_dir / "real_execution.json", last_execution)
                if not bool(last_execution.get("ok")):
                    _save_text(attempt_dir / "real_execution_error.txt", str(last_execution.get("error", "")))

                current_phase_manifest = {
                    "index": phase_spec.index,
                    "slug": phase_spec.slug,
                    "goal": phase_spec.goal,
                    "success_criteria": phase_spec.success_criteria,
                    "ok": bool(last_execution.get("ok")),
                    "global_image": str(phase_global),
                    "wrist_image": str(phase_wrist),
                    "global_rgb": str(phase_global),
                    "wrist_rgb": str(phase_wrist),
                    "error": last_execution.get("error"),
                    "image_width_px": 128,
                    "is_final_phase": phase_spec.index == len(phase_specs),
                    "image_roles": observation["image_roles"],
                    "completed_phase_history": _phase_history_for_prompt(phase_history),
                }
                judge_prompt = (
                    "DEFENSE_EI_AGENTS_REAL_PHASE_JUDGING\n"
                    "High-level task:\n"
                    f"{task}\n\n"
                    "Atomic task info JSON:\n"
                    f"{json.dumps(atomic_info, ensure_ascii=False, indent=2)}\n\n"
                    "Generated and executed current-phase code:\n"
                    "```python\n"
                    f"{last_code}\n"
                    "```\n\n"
                    "Real execution report:\n"
                    f"{json.dumps(last_execution, ensure_ascii=False, indent=2)}\n\n"
                    "Current phase manifest JSON. The two attached images are exactly this "
                    "phase's global_image and wrist_image, in that order. global_image shows "
                    "the whole scene; wrist_image is from the gripper viewpoint and may show "
                    "part of the gripper at the bottom:\n"
                    f"{json.dumps(current_phase_manifest, ensure_ascii=False, indent=2)}\n\n"
                    "Judge only the current phase against success_criteria. If this phase has "
                    "visibly failed, return FAIL and explain this phase's failure and the code "
                    "correction needed. If this phase succeeds and it is not the final phase, "
                    "return SUCCESS so the next phase can be coded. If this is the final phase, "
                    "return SUCCESS only if the whole atomic task is complete.\n\n"
                    "Return JSON only with keys task_result and analysis."
                )
                judger_result = await judger.run(
                    judge_prompt,
                    max_steps=max_steps,
                    images=[phase_global, phase_wrist],
                )
                raw_judger_text = judger_result.final_answer
                phase_judge = _extract_json_object(raw_judger_text)
                last_judge = phase_judge
                if not _is_success_judge(phase_judge):
                    last_judge = {
                        "task_result": "FAIL",
                        "analysis": (
                            f"Phase {phase_spec.index} ({phase_spec.slug}) failed. "
                            f"{str(phase_judge.get('analysis', '')).strip()}"
                        ).strip(),
                    }
                phase_judge_with_context = {
                    "phase_index": phase_spec.index,
                    "phase_slug": phase_spec.slug,
                    "global_image": str(phase_global),
                    "wrist_image": str(phase_wrist),
                    "global_rgb": str(phase_global),
                    "wrist_rgb": str(phase_wrist),
                    "judge": phase_judge,
                }
                _save_json(attempt_dir / f"real_atomic_judge_phase_{phase_spec.index:02d}.json", phase_judge_with_context)
                _save_text(attempt_dir / f"real_atomic_judge_phase_{phase_spec.index:02d}_raw.txt", raw_judger_text)
                _save_json(attempt_dir / "real_atomic_judge.json", last_judge)
                _save_text(attempt_dir / "real_atomic_judge_raw.txt", raw_judger_text)

                attempt_record = {
                    "atomic_id": atomic_info.get("id", atomic_index),
                    "phase_index": phase_spec.index,
                    "phase_slug": phase_spec.slug,
                    "attempt": attempt_index,
                    "syntax": syntax_report,
                    "execution": last_execution,
                    "judge": last_judge,
                    "after_global_image": str(phase_global),
                    "after_wrist_image": str(phase_wrist),
                    "after_global_rgb": str(phase_global),
                    "after_wrist_rgb": str(phase_wrist),
                    "phase_images": [current_phase_manifest],
                    "phase_judgements": [phase_judge_with_context],
                    "code_path": str(attempt_dir / "real_phase_actions.py"),
                }
                attempts.append(attempt_record)

                latest_global = phase_global
                latest_wrist = phase_wrist
                if _is_success_judge(phase_judge):
                    phase_history.append(attempt_record)
                    phase_done = True
                    break

                feedback = {
                    "task_result": "FAIL",
                    "analysis": str(last_judge.get("analysis", "")).strip(),
                    "execution": last_execution,
                    "current_phase": phase_context,
                }

            if not phase_done:
                completed = False
                restore_report = _restore_robot_initial_state(controller, initial_robot_state)
                failed_dir = log_dir / f"atomic_{atomic_index:02d}" / f"phase_{phase_spec.index:02d}_{phase_spec.slug}"
                _save_json(failed_dir / "restore_initial_robot_state.json", restore_report)
                break

        atomic_done = all(
            any(
                item.get("phase_index") == spec.index and _is_success_judge(item.get("judge", {}))
                for item in phase_history
            )
            for spec in phase_specs
        )
        if not atomic_done:
            completed = False
            break

    _save_json(log_dir / "defense_ei_attempts.json", {"attempts": attempts, "completed": completed})
    _save_json(log_dir / "real_execution.json", last_execution)
    _save_json(log_dir / "real_finished_view_judge.json", last_judge)
    _save_text(log_dir / "real_finished_view_judge_raw.txt", raw_judger_text)

    return RealDefenseOutputs(
        plan=plan,
        atomic_task_info=atomic_task_info,
        attempts=attempts,
        completed=completed,
        code=last_code,
        execution=last_execution,
        judge=last_judge,
        raw_planner_text=raw_planner_text,
        raw_supervisor_text=raw_supervisor_text,
        raw_coder_text=raw_coder_text,
        raw_judger_text=raw_judger_text,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DefenseAgent real-robot defense_ei_agents evaluation.")
    parser.add_argument("--task", default="", help="High-level task description.")
    parser.add_argument("--planner-profile", type=Path, default=DEFAULT_PLANNER_PROFILE)
    parser.add_argument("--supervisor-profile", type=Path, default=DEFAULT_SUPERVISOR_PROFILE)
    parser.add_argument("--coder-profile", type=Path, default=DEFAULT_CODER_PROFILE)
    parser.add_argument("--judger-profile", type=Path, default=DEFAULT_JUDGER_PROFILE)
    parser.add_argument("--log-root", type=Path, default=DEFAULT_LOG_ROOT)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument(
        "--max-attempts-per-atomic",
        type=int,
        default=3,
        help="Maximum retry attempts for each phase within an atomic task.",
    )

    parser.add_argument("--robot-ip", default=ROBOT_IP)
    parser.add_argument("--robotiq-urscript-defs-path", default="")
    parser.add_argument("--strict-gripper-connection", action="store_true")
    parser.add_argument(
        "--capture-only",
        action="store_true",
        help="Only capture current global/wrist RGB images, then exit before agent generation or robot connection.",
    )
    parser.add_argument("--list-cameras", action="store_true", help="List detected Intel RealSense cameras and exit.")

    parser.add_argument("--camera-serials", default="", help="Comma-separated RealSense serials: global,wrist. Auto-discovers if omitted.")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-warmup-frames", type=int, default=15)
    parser.add_argument(
        "--current-global-image",
        dest="current_global_image",
        type=Path,
        default=None,
        help="Use an existing global RGB image instead of capturing.",
    )
    parser.add_argument(
        "--current-front-image",
        dest="current_global_image",
        type=Path,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--current-wrist-image", type=Path, default=None, help="Use an existing wrist RGB image instead of capturing.")

    parser.add_argument("--embedding-model", default=os.environ.get("EMBEDDING_MODEL", "text-embedding-3-small"))
    parser.add_argument("--embedding-api-key", default=os.environ.get("EMBEDDING_API_KEY"))
    parser.add_argument("--embedding-base-url", default=os.environ.get("EMBEDDING_BASE_URL"))
    parser.add_argument("--embedding-dims", type=int, default=int(os.environ.get("EMBEDDING_DIMS", "1536")))
    return parser.parse_args()


async def _main_async() -> int:
    args = parse_args()
    if args.list_cameras:
        try:
            cameras = _list_realsense_cameras()
        except (ImportError, RuntimeError) as exc:
            print(str(exc))
            return 1
        if not cameras:
            print("No Intel RealSense devices were detected.")
            return 1
        print(json.dumps({"realsense_cameras": cameras}, ensure_ascii=False, indent=2))
        return 0
    if not str(args.task).strip():
        raise ValueError("--task is required unless --list-cameras is used.")

    run_dir = _resolve_path(args.log_root) / time.strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    current_global = run_dir / "current_global_rgb.png"
    current_wrist = run_dir / "current_wrist_rgb.png"
    serials = [s.strip() for s in str(args.camera_serials).split(",") if s.strip()]

    controller = UR7eVectorController(
        robot_ip=str(args.robot_ip),
        robotiq_urscript_defs_path=str(args.robotiq_urscript_defs_path).strip() or None,
        strict_gripper_connection=bool(args.strict_gripper_connection),
    )

    source_global = _resolve_path(args.current_global_image) if args.current_global_image else None
    source_wrist = _resolve_path(args.current_wrist_image) if args.current_wrist_image else None
    planner_profile = _prepare_real_profile_copy(_resolve_path(args.planner_profile), run_dir, "planner")
    supervisor_profile = _prepare_real_profile_copy(_resolve_path(args.supervisor_profile), run_dir, "supervisor")
    coder_profile = _prepare_real_profile_copy(_resolve_path(args.coder_profile), run_dir, "coder")
    judger_profile = _prepare_real_profile_copy(_resolve_path(args.judger_profile), run_dir, "judger")
    if source_global or source_wrist:
        if not source_global or not source_wrist:
            raise ValueError("--current-global-image and --current-wrist-image must be provided together.")
        if not source_global.exists() or not source_wrist.exists():
            raise FileNotFoundError(f"current image(s) not found: {source_global}, {source_wrist}")
        current_global = source_global
        current_wrist = source_wrist

        _log("Starting RealSense capture for post-execution feedback")
        with RealSenseRGBPair(
            serials=serials,
            width=int(args.camera_width),
            height=int(args.camera_height),
            fps=int(args.camera_fps),
            warmup_frames=int(args.camera_warmup_frames),
        ) as cameras:
            try:
                _log(f"Connecting UR7e controller at {args.robot_ip}")
                controller.connect()
                if controller.is_gripper_available():
                    _log(f"Gripper backend: {controller.get_gripper_backend()}")

                def capture_images(global_path: Path, wrist_path: Path) -> None:
                    global_done, wrist_done = cameras.capture()
                    imageio.imwrite(global_path, global_done)
                    imageio.imwrite(wrist_path, wrist_done)

                outputs = await run_defense_agent_real_once(
                    task=str(args.task).strip(),
                    current_global_image=current_global,
                    current_wrist_image=current_wrist,
                    planner_profile=planner_profile,
                    supervisor_profile=supervisor_profile,
                    coder_profile=coder_profile,
                    judger_profile=judger_profile,
                    log_dir=run_dir,
                    controller=controller,
                    max_steps=args.max_steps,
                    max_attempts_per_atomic=int(args.max_attempts_per_atomic),
                    embedding_model=str(args.embedding_model),
                    embedding_api_key=args.embedding_api_key,
                    embedding_base_url=args.embedding_base_url,
                    embedding_dims=int(args.embedding_dims),
                    capture_images_fn=capture_images,
                )
            finally:
                controller.close()

        summary = {
            "run_dir": str(run_dir),
            "current_global_image": str(current_global),
            "current_wrist_image": str(current_wrist),
            "plan_status": outputs.plan.get("status"),
            "atomic_task_count": len(_atomic_infos_from_supervisor(outputs.atomic_task_info)),
            "completed": outputs.completed,
            "attempt_count": len(outputs.attempts),
            "execution_ok": bool(outputs.execution.get("ok")),
            "execution_signal": outputs.execution.get("signal"),
            "judge_result": outputs.judge.get("task_result"),
        }
        _save_json(run_dir / "summary.json", summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0 if outputs.completed else 2

    _log("Starting RealSense capture")
    with RealSenseRGBPair(
        serials=serials,
        width=int(args.camera_width),
        height=int(args.camera_height),
        fps=int(args.camera_fps),
        warmup_frames=int(args.camera_warmup_frames),
    ) as cameras:
        global_img, wrist_img = cameras.capture()
        imageio.imwrite(current_global, global_img)
        imageio.imwrite(current_wrist, wrist_img)

        if args.capture_only:
            summary = {
                "run_dir": str(run_dir),
                "capture_only": True,
                "global_image": str(current_global),
                "wrist_image": str(current_wrist),
                "global_shape": list(global_img.shape),
                "wrist_shape": list(wrist_img.shape),
                "global_exists": current_global.exists(),
                "wrist_exists": current_wrist.exists(),
            }
            _save_json(run_dir / "summary.json", summary)
            print(json.dumps(summary, ensure_ascii=False, indent=2))
            return 0

        try:
            _log(f"Connecting UR7e controller at {args.robot_ip}")
            controller.connect()
            if controller.is_gripper_available():
                _log(f"Gripper backend: {controller.get_gripper_backend()}")

            def capture_images(global_path: Path, wrist_path: Path) -> None:
                global_done, wrist_done = cameras.capture()
                imageio.imwrite(global_path, global_done)
                imageio.imwrite(wrist_path, wrist_done)

            outputs = await run_defense_agent_real_once(
                task=str(args.task).strip(),
                current_global_image=current_global,
                current_wrist_image=current_wrist,
                planner_profile=planner_profile,
                supervisor_profile=supervisor_profile,
                coder_profile=coder_profile,
                judger_profile=judger_profile,
                log_dir=run_dir,
                controller=controller,
                max_steps=args.max_steps,
                max_attempts_per_atomic=int(args.max_attempts_per_atomic),
                embedding_model=str(args.embedding_model),
                embedding_api_key=args.embedding_api_key,
                embedding_base_url=args.embedding_base_url,
                embedding_dims=int(args.embedding_dims),
                capture_images_fn=capture_images,
            )

        finally:
            controller.close()

    summary = {
        "run_dir": str(run_dir),
        "plan_status": outputs.plan.get("status"),
        "atomic_task_count": len(_atomic_infos_from_supervisor(outputs.atomic_task_info)),
        "completed": outputs.completed,
        "attempt_count": len(outputs.attempts),
        "execution_ok": bool(outputs.execution.get("ok")),
        "execution_signal": outputs.execution.get("signal"),
        "judge_result": outputs.judge.get("task_result"),
    }
    _save_json(run_dir / "summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if outputs.completed else 2


def main() -> int:
    try:
        return asyncio.run(_main_async())
    except (ImportError, RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
