#!/usr/bin/env python3
"""Cancel a stale PNC task after a chassis run.

This utility is intentionally narrow: it only touches PNC task state, and it
requires --confirm-live before sending cancel_task().
"""

from __future__ import annotations

import argparse
import time


ENDED_STATES = (0, 3, 6, 7, 8, 9)


def read_task_with_retry(pnc, label: str, retries: int, wait_s: float):
    last_error = None
    for attempt in range(1, retries + 1):
        try:
            task = pnc.get_task_state()
            state = getattr(task, "state", None)
            task_id = getattr(task, "id", None)
            task_type = getattr(task, "type", None)
            message = getattr(task, "message", "")
            print(
                f"{label} attempt={attempt} state={state} "
                f"id={task_id} type={task_type} message={message}",
                flush=True,
            )
            return task
        except Exception as exc:
            last_error = exc
            print(f"{label}_read_failed attempt={attempt} error={exc}", flush=True)
            if attempt < retries:
                time.sleep(wait_s)
    raise RuntimeError(f"{label} failed after {retries} attempts: {last_error}")


def run(args):
    import agibot_gdk

    result = agibot_gdk.gdk_init()
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    if gdk_res is not None and result not in (None, gdk_res.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")

    try:
        pnc = agibot_gdk.Pnc()
        time.sleep(args.init_wait_s)
        task = read_task_with_retry(pnc, "task_before_cancel", args.retries, args.wait_s)
        state = getattr(task, "state", None)
        task_id = getattr(task, "id", None)
        if task_id is None:
            print("cancel_skipped reason=no_task_id", flush=True)
            return
        if state in ENDED_STATES:
            print(f"cancel_skipped reason=ended_state state={state} id={task_id}", flush=True)
            return
        if not args.confirm_live:
            print(
                f"cancel_skipped reason=missing_confirm_live state={state} id={task_id}",
                flush=True,
            )
            return

        try:
            pnc.cancel_task(task_id)
            print(f"cancel_sent id={task_id} state={state}", flush=True)
        except RuntimeError as exc:
            if "Task is not in RUNNING or PAUSED state" not in str(exc):
                raise
            print(f"cancel_ignored id={task_id} error={exc}", flush=True)

        time.sleep(args.post_wait_s)
        read_task_with_retry(pnc, "task_after_cancel", args.retries, args.wait_s)
    finally:
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass


def parse_args():
    parser = argparse.ArgumentParser(description="Cancel stale G2 PNC task")
    parser.add_argument("--confirm-live", action="store_true")
    parser.add_argument("--retries", type=int, default=8)
    parser.add_argument("--wait-s", type=float, default=0.5)
    parser.add_argument("--init-wait-s", type=float, default=0.8)
    parser.add_argument("--post-wait-s", type=float, default=0.5)
    args = parser.parse_args()
    if args.retries <= 0:
        raise SystemExit("--retries must be positive")
    if args.wait_s < 0.0:
        raise SystemExit("--wait-s must be >= 0")
    if args.init_wait_s < 0.0:
        raise SystemExit("--init-wait-s must be >= 0")
    if args.post_wait_s < 0.0:
        raise SystemExit("--post-wait-s must be >= 0")
    return args


if __name__ == "__main__":
    run(parse_args())
