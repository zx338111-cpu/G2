#!/usr/bin/env python3
import json
import re
import sys
from pathlib import Path

REQUIRED = [
    "task_id",
    "title",
    "role",
    "risk_level",
    "objective",
    "context",
    "allowed_files",
    "forbidden_files",
    "steps",
    "commands",
    "expected_outputs",
    "success_criteria",
    "stop_conditions",
    "max_time_minutes",
    "requires_human_approval",
]

FORBIDDEN_COMMAND_PATTERNS = [
    r"\bsudo\b",
    r"\brm\s+-rf\b",
    r"\bmkfs\b",
    r"\bdd\s+if=",
    r"\bchmod\s+-R\s+777\b",
    r"\bchown\s+-R\b",
    r"\bcurl\b.*\|\s*(bash|sh)",
    r"\bwget\b.*\|\s*(bash|sh)",
    r"\bapt(-get)?\s+install\b",
    r"\bsnap\s+install\b",
    r"\byum\s+install\b",
    r"\bpacman\s+-S\b",
    r"\bsystemctl\b",
    r"\breboot\b",
    r"\bshutdown\b",
    r"\bros2\s+launch\b",
    r"\broslaunch\b",
    r"\bros2\s+run\b",
    r"\bgit\s+push\s+origin\s+(main|master)\b",
    r"\bpython3?\s+.*(move_arm|move_waist|industrial_|offset_move|controller|driver|hardware).*\.py",
]

SECRET_WORDS = [
    ".env",
    "id_rsa",
    "id_ed25519",
    ".ssh",
    "credential",
    "credentials",
    "secret",
    "token",
    "api_key",
    "apikey",
]

DANGEROUS_ALLOWED_TARGETS = [
    "/",
    ".",
    "./",
    "~",
    "datasets",
    "dataset",
    "checkpoints",
    "rosbags",
    ".git",
    ".ssh",
]

def fail(msg: str) -> None:
    print("[FAIL]", msg)
    sys.exit(1)

def ok(msg: str) -> None:
    print("[OK]", msg)

def is_bad_path(path_value: str) -> bool:
    p = str(path_value).strip()
    parts = Path(p).parts
    if p.startswith("/"):
        return True
    if ".." in parts:
        return True
    if p in DANGEROUS_ALLOWED_TARGETS:
        return True
    low = p.lower().strip("/")
    for target in DANGEROUS_ALLOWED_TARGETS:
        if low == target or low.startswith(target + "/"):
            return True
    return False

def main() -> None:
    if len(sys.argv) != 2:
        fail("Usage: python3 .ai_agents/scripts/agent_guard.py path/to/task.json")

    task_path = Path(sys.argv[1])

    try:
        task = json.loads(task_path.read_text(encoding="utf-8"))
    except Exception as e:
        fail(f"Cannot read JSON: {e}")

    missing = [k for k in REQUIRED if k not in task]
    if missing:
        fail(f"Missing required fields: {missing}")

    if task["role"] != "execution_engineer":
        fail("role must be execution_engineer")

    if task["risk_level"] not in ["low", "medium", "high"]:
        fail("risk_level must be low, medium, or high")

    for key in [
        "allowed_files",
        "forbidden_files",
        "steps",
        "commands",
        "expected_outputs",
        "success_criteria",
        "stop_conditions",
    ]:
        if not isinstance(task[key], list):
            fail(f"{key} must be a list")

    if task["risk_level"] == "high" and not task["requires_human_approval"]:
        fail("High-risk tasks require human approval")

    try:
        max_time = int(task["max_time_minutes"])
    except Exception:
        fail("max_time_minutes must be an integer")

    if max_time > 120 and not task["requires_human_approval"]:
        fail("Tasks over 120 minutes require human approval")

    for cmd in task["commands"]:
        for pat in FORBIDDEN_COMMAND_PATTERNS:
            if re.search(pat, cmd):
                fail(f"Forbidden command pattern detected: {pat} in command: {cmd}")

    # Important:
    # forbidden_files may contain secrets; allowed_files must not.
    for p in task["allowed_files"]:
        low = str(p).lower()
        for word in SECRET_WORDS:
            if word in low:
                fail(f"Secret-like path is not allowed in allowed_files: {p}")
        if is_bad_path(str(p)):
            fail(f"Dangerous allowed file target: {p}")

    if not task["stop_conditions"]:
        fail("stop_conditions cannot be empty")

    if not task["success_criteria"]:
        fail("success_criteria cannot be empty")

    ok(f"{task['task_id']} passed guard checks")

if __name__ == "__main__":
    main()
