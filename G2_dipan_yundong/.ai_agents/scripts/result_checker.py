#!/usr/bin/env python3
"""Validate an AI-B result JSON against its source task JSON.

The checker is intentionally strict:
- every recorded command must be declared in task.commands, except the matching
  agent_guard.py command for the same task JSON;
- every changed/created file must be declared in task.allowed_files;
- result.status must be one of the workflow statuses.
"""

from __future__ import annotations

import json
import shlex
import sys
from pathlib import Path
from typing import Any


VALID_STATUSES = {"done", "blocked", "failed"}


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def require_list(obj: dict[str, Any], key: str, errors: list[str]) -> list[Any]:
    value = obj.get(key, [])
    if not isinstance(value, list):
        errors.append(f"{key} must be a list")
        return []
    return value


def repo_root_for_task(task_path: Path) -> Path:
    resolved = task_path.resolve()
    for parent in (resolved.parent, *resolved.parents):
        if parent.name == ".ai_agents":
            return parent.parent
    return Path.cwd().resolve()


def is_matching_guard_command(command: str, task_path: Path) -> bool:
    try:
        parts = shlex.split(command)
    except ValueError:
        return False

    if len(parts) != 3:
        return False
    if parts[0] != "python3":
        return False

    repo_root = repo_root_for_task(task_path)
    guard_path = (repo_root / ".ai_agents/scripts/agent_guard.py").resolve()

    try:
        command_guard_path = (repo_root / parts[1]).resolve()
        command_task_path = (repo_root / parts[2]).resolve()
    except OSError:
        return False

    return command_guard_path == guard_path and command_task_path == task_path.resolve()


def validate(task_path: Path, result_path: Path) -> list[str]:
    errors: list[str] = []
    task = load_json(task_path)
    result = load_json(result_path)

    if not isinstance(task, dict):
        return ["task JSON root must be an object"]
    if not isinstance(result, dict):
        return ["result JSON root must be an object"]

    declared_commands = set(require_list(task, "commands", errors))
    allowed_files = set(require_list(task, "allowed_files", errors))

    status = result.get("status")
    if status not in VALID_STATUSES:
        errors.append(f"status must be one of {sorted(VALID_STATUSES)}, got {status!r}")

    commands_run = require_list(result, "commands_run", errors)
    for index, command in enumerate(commands_run):
        if not isinstance(command, str):
            errors.append(f"commands_run[{index}] must be a string")
            continue
        if command in declared_commands:
            continue
        if is_matching_guard_command(command, task_path):
            continue
        errors.append(
            "commands_run[{index}] not declared in task.commands and not the matching "
            "agent_guard.py command: {command}".format(index=index, command=command)
        )

    for field_name in ("outputs_created", "files_changed"):
        entries = require_list(result, field_name, errors)
        for index, entry in enumerate(entries):
            if not isinstance(entry, str):
                errors.append(f"{field_name}[{index}] must be a string")
                continue
            if entry not in allowed_files:
                errors.append(
                    f"{field_name}[{index}] is outside task.allowed_files: {entry}"
                )

    return errors


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(
            "FAIL: usage: result_checker.py <task_json_path> <result_json_path>",
            file=sys.stderr,
        )
        return 2

    task_path = Path(argv[1])
    result_path = Path(argv[2])

    try:
        errors = validate(task_path, result_path)
    except FileNotFoundError as exc:
        print(f"FAIL: missing file: {exc.filename}")
        return 1
    except json.JSONDecodeError as exc:
        print(f"FAIL: invalid JSON in {exc.doc!r}: {exc}")
        return 1

    if errors:
        print("FAIL: result validation failed")
        for error in errors:
            print(f"- {error}")
        return 1

    print("OK: result stays within declared commands, allowed files, and status set")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
