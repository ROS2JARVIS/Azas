from __future__ import annotations

import json
import time
import uuid
from pathlib import Path
from threading import Lock
from typing import Any


ROOT = Path("/home/ssu/Azas")
DEFAULT_RESUME_STATE = ROOT / "outputs" / "auto_cup_flow_resume.json"
DEFAULT_EVENTS_LOG = ROOT / "outputs" / "auto_cup_flow_events.jsonl"

FLOW_STAGES = (
    "color_scan",
    "observe",
    "open_gripper",
    "cup_pick",
    "recipe",
    "lid_shake",
    "human_handover",
)

STAGE_LABELS = {
    "color_scan": "dispenser color scan",
    "observe": "cup observe pose",
    "open_gripper": "initial gripper open",
    "cup_pick": "cup route and pick",
    "recipe": "measured dispenser recipe",
    "lid_shake": "lid close and shake",
    "human_handover": "MediaPipe palm handover",
}


def now_stamp() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def load_resume_snapshot(path: str | Path = DEFAULT_RESUME_STATE) -> dict[str, Any] | None:
    state_path = Path(path)
    if not state_path.is_file():
        return None
    try:
        payload = json.loads(state_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def safe_recipe_colors_from_snapshot(snapshot: dict[str, Any] | None) -> str:
    if not isinstance(snapshot, dict):
        return ""
    recipe = snapshot.get("recipe")
    if not isinstance(recipe, dict):
        return ""
    colors = str(recipe.get("recipe_colors") or "").strip()
    return colors


class AutoFlowResumeStore:
    """Durable stage journal for the top-level cocktail flow.

    The store records symbolic stage progress and verified facts only. It does
    not store cup/lid coordinates or synthesize robot poses.
    """

    def __init__(
        self,
        *,
        state_path: str | Path = DEFAULT_RESUME_STATE,
        events_path: str | Path = DEFAULT_EVENTS_LOG,
        mode: str = "normal",
        recipe_colors: str = "",
        recipe_id: str = "",
    ) -> None:
        self.state_path = Path(state_path)
        self.events_path = Path(events_path)
        self.mode = mode if mode in {"normal", "resume", "restart"} else "normal"
        self.recipe_colors = recipe_colors
        self.recipe_id = recipe_id
        self._lock = Lock()
        self._last_heartbeat_write = 0.0
        self.snapshot: dict[str, Any] = {}

    def prepare(self) -> bool:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        self.events_path.parent.mkdir(parents=True, exist_ok=True)
        previous = load_resume_snapshot(self.state_path)
        if self.mode == "restart":
            self.clear()
            previous = None
        if self.mode == "resume":
            if not previous:
                self.block(
                    "no_resume_state",
                    "저장된 복구 상태가 없습니다. 새 주문을 먼저 시작하세요.",
                    auto_recoverable=False,
                )
                return False
            previous_colors = safe_recipe_colors_from_snapshot(previous)
            if self.recipe_colors and previous_colors and self.recipe_colors != previous_colors:
                self.block(
                    "resume_recipe_mismatch",
                    "저장된 주문과 요청한 주문이 다릅니다. 처음부터 다시 시작해야 합니다.",
                    auto_recoverable=False,
                )
                return False
            self.recipe_colors = self.recipe_colors or previous_colors
            self.recipe_id = self.recipe_id or str((previous.get("recipe") or {}).get("recipe_id") or "")
            self.snapshot = previous
            self.snapshot["status"] = "running"
            self.snapshot["resume_mode"] = "resume"
            self.snapshot["heartbeat_at"] = now_stamp()
            self.snapshot["updated_at"] = now_stamp()
            self._write_snapshot()
            self._append_event("resume_loaded", {"next_stage": self.next_stage()})
            return True

        self.snapshot = self._new_snapshot(status="running")
        self._write_snapshot()
        self._append_event("run_started", {"mode": self.mode})
        return True

    def clear(self) -> None:
        try:
            self.state_path.unlink()
        except FileNotFoundError:
            pass

    def _new_snapshot(self, *, status: str) -> dict[str, Any]:
        return {
            "version": 1,
            "run_id": uuid.uuid4().hex,
            "status": status,
            "resume_mode": self.mode,
            "stage": None,
            "step": None,
            "next_stage": FLOW_STAGES[0],
            "completed_stages": [],
            "recipe": {
                "recipe_id": self.recipe_id,
                "recipe_colors": self.recipe_colors,
            },
            "held_objects": {
                "cup": "unknown",
                "lid": "unknown",
            },
            "verified": {
                "color_map": False,
                "cup_picked": False,
                "dispenser_sequence_done": False,
                "cup_in_holder": False,
                "lid_grasped": False,
                "lid_closed": False,
                "shake_done": False,
                "human_handover_done": False,
            },
            "stop_reason": None,
            "blocker": None,
            "auto_recoverable": True,
            "required_user_action": None,
            "created_at": now_stamp(),
            "updated_at": now_stamp(),
            "heartbeat_at": now_stamp(),
        }

    def _write_snapshot(self) -> None:
        with self._lock:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            self.state_path.write_text(
                json.dumps(self.snapshot, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

    def _append_event(self, event: str, fields: dict[str, Any] | None = None) -> None:
        payload = {
            "event": event,
            "run_id": self.snapshot.get("run_id"),
            "stage": self.snapshot.get("stage"),
            "status": self.snapshot.get("status"),
            "created_at": now_stamp(),
        }
        if fields:
            payload.update(fields)
        with self._lock:
            self.events_path.parent.mkdir(parents=True, exist_ok=True)
            with self.events_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def next_stage(self) -> str:
        completed = set(self.snapshot.get("completed_stages") or [])
        for stage in FLOW_STAGES:
            if stage not in completed:
                return stage
        return "complete"

    def should_skip(self, stage: str) -> bool:
        return self.mode == "resume" and stage in set(self.snapshot.get("completed_stages") or [])

    def start_stage(self, stage: str, *, step: str | None = None) -> None:
        self.snapshot["status"] = "running"
        self.snapshot["stage"] = stage
        self.snapshot["step"] = step or stage
        self.snapshot["next_stage"] = stage
        self.snapshot["stop_reason"] = None
        self.snapshot["blocker"] = None
        self.snapshot["required_user_action"] = None
        self.snapshot["auto_recoverable"] = True
        self.snapshot["updated_at"] = now_stamp()
        self.snapshot["heartbeat_at"] = now_stamp()
        self._write_snapshot()
        self._append_event("stage_started", {"stage_label": STAGE_LABELS.get(stage, stage)})

    def complete_stage(
        self,
        stage: str,
        *,
        verified: dict[str, bool] | None = None,
        held_objects: dict[str, str] | None = None,
    ) -> None:
        completed = list(self.snapshot.get("completed_stages") or [])
        if stage not in completed:
            completed.append(stage)
        self.snapshot["completed_stages"] = completed
        if verified:
            current_verified = dict(self.snapshot.get("verified") or {})
            current_verified.update(verified)
            self.snapshot["verified"] = current_verified
        if held_objects:
            current_held = dict(self.snapshot.get("held_objects") or {})
            current_held.update(held_objects)
            self.snapshot["held_objects"] = current_held
        self.snapshot["status"] = "running"
        self.snapshot["stage"] = stage
        self.snapshot["step"] = f"{stage}_done"
        self.snapshot["next_stage"] = self.next_stage()
        self.snapshot["updated_at"] = now_stamp()
        self.snapshot["heartbeat_at"] = now_stamp()
        self._write_snapshot()
        self._append_event("stage_completed", {"next_stage": self.snapshot["next_stage"]})

    def heartbeat(self, *, process_label: str | None = None) -> None:
        now = time.monotonic()
        if now - self._last_heartbeat_write < 2.0:
            return
        self._last_heartbeat_write = now
        self.snapshot["heartbeat_at"] = now_stamp()
        self.snapshot["updated_at"] = now_stamp()
        if process_label:
            self.snapshot["last_process_label"] = process_label
        self._write_snapshot()

    def update_progress(
        self,
        stage: str,
        step: str,
        *,
        verified: dict[str, bool] | None = None,
        held_objects: dict[str, str] | None = None,
    ) -> None:
        if self.snapshot.get("stage") == stage and self.snapshot.get("step") == step:
            self.heartbeat()
            return
        current_verified = dict(self.snapshot.get("verified") or {})
        if verified:
            current_verified.update(verified)
            self.snapshot["verified"] = current_verified
        current_held = dict(self.snapshot.get("held_objects") or {})
        if held_objects:
            current_held.update(held_objects)
            self.snapshot["held_objects"] = current_held
        self.snapshot["status"] = "running"
        self.snapshot["stage"] = stage
        self.snapshot["step"] = step
        self.snapshot["next_stage"] = stage
        self.snapshot["updated_at"] = now_stamp()
        self.snapshot["heartbeat_at"] = now_stamp()
        self._write_snapshot()
        self._append_event("stage_progress", {"step": step})

    def fail_stage(self, stage: str, reason: str, *, auto_recoverable: bool = True) -> None:
        self.snapshot["status"] = "stopped"
        self.snapshot["stage"] = stage
        self.snapshot["step"] = f"{stage}_failed"
        self.snapshot["next_stage"] = stage
        self.snapshot["stop_reason"] = reason
        self.snapshot["blocker"] = reason
        self.snapshot["auto_recoverable"] = auto_recoverable
        self.snapshot["required_user_action"] = (
            "하드웨어 상태를 확인한 뒤 '복구 다시 확인' 또는 '이어서 해줘'라고 말하세요."
        )
        self.snapshot["updated_at"] = now_stamp()
        self.snapshot["heartbeat_at"] = now_stamp()
        self._write_snapshot()
        self._append_event("stage_failed", {"reason": reason, "auto_recoverable": auto_recoverable})

    def block(self, reason: str, required_user_action: str, *, auto_recoverable: bool) -> None:
        if not self.snapshot:
            self.snapshot = self._new_snapshot(status="blocked")
        self.snapshot["status"] = "blocked"
        self.snapshot["stop_reason"] = reason
        self.snapshot["blocker"] = reason
        self.snapshot["auto_recoverable"] = auto_recoverable
        self.snapshot["required_user_action"] = required_user_action
        self.snapshot["updated_at"] = now_stamp()
        self.snapshot["heartbeat_at"] = now_stamp()
        self._write_snapshot()
        self._append_event("blocked", {"reason": reason, "auto_recoverable": auto_recoverable})

    def complete_run(self) -> None:
        self.snapshot["status"] = "completed"
        self.snapshot["stage"] = "complete"
        self.snapshot["step"] = "complete"
        self.snapshot["next_stage"] = "complete"
        self.snapshot["completed_stages"] = list(FLOW_STAGES)
        self.snapshot["updated_at"] = now_stamp()
        self.snapshot["heartbeat_at"] = now_stamp()
        self._write_snapshot()
        self._append_event("run_completed")
