#!/usr/bin/env python3
"""Structured run artifacts for industrial G2 workflows."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
from datetime import datetime
import json
from pathlib import Path
import time
import traceback
import uuid


def _now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _jsonable(value):
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(item) for item in value]
    if isinstance(value, BaseException):
        return {
            "type": type(value).__name__,
            "message": str(value),
            "traceback": traceback.format_exception(type(value), value, value.__traceback__),
        }
    return repr(value)


class RunRecorder:
    """Write JSONL events, checkpoint, and final report for a workflow run."""

    def __init__(
        self,
        *,
        name: str,
        log_file: Path | None,
        event_file: Path | None,
        checkpoint_file: Path | None,
        report_file: Path | None,
    ):
        self.name = name
        self.log_file = log_file
        self.event_file = event_file
        self.checkpoint_file = checkpoint_file
        self.report_file = report_file
        self.run_id = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
        self.started_at = _now_iso()
        self.started_monotonic = time.time()
        self.last_checkpoint = None
        self.events_written = 0

    def event(self, event_type: str, **fields):
        payload = {
            "ts": _now_iso(),
            "run_id": self.run_id,
            "workflow": self.name,
            "event": event_type,
            **fields,
        }
        if self.event_file is not None:
            self.event_file.parent.mkdir(parents=True, exist_ok=True)
            with self.event_file.open("a", encoding="utf-8") as f:
                f.write(json.dumps(_jsonable(payload), ensure_ascii=False, sort_keys=True) + "\n")
        self.events_written += 1
        return payload

    def checkpoint(self, **fields):
        payload = {
            "ts": _now_iso(),
            "run_id": self.run_id,
            "workflow": self.name,
            "log_file": self.log_file,
            "event_file": self.event_file,
            **fields,
        }
        self.last_checkpoint = _jsonable(payload)
        if self.checkpoint_file is not None:
            self.checkpoint_file.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = self.checkpoint_file.with_suffix(self.checkpoint_file.suffix + ".tmp")
            with tmp_path.open("w", encoding="utf-8") as f:
                json.dump(self.last_checkpoint, f, ensure_ascii=False, indent=2, sort_keys=True)
                f.write("\n")
            tmp_path.replace(self.checkpoint_file)
        return self.last_checkpoint

    def report(self, status: str, *, error: BaseException | None = None, **fields):
        payload = {
            "ts": _now_iso(),
            "run_id": self.run_id,
            "workflow": self.name,
            "status": status,
            "started_at": self.started_at,
            "elapsed_s": round(time.time() - self.started_monotonic, 3),
            "log_file": self.log_file,
            "event_file": self.event_file,
            "checkpoint_file": self.checkpoint_file,
            "events_written": self.events_written,
            "last_checkpoint": self.last_checkpoint,
            **fields,
        }
        if error is not None:
            payload["error"] = error
        json_payload = _jsonable(payload)
        if self.report_file is not None:
            self.report_file.parent.mkdir(parents=True, exist_ok=True)
            with self.report_file.open("w", encoding="utf-8") as f:
                json.dump(json_payload, f, ensure_ascii=False, indent=2, sort_keys=True)
                f.write("\n")
        return json_payload
