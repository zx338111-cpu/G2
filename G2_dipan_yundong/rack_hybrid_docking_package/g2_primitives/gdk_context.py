"""Small helpers for scoped GDK sessions.

Every primitive opens and releases GDK inside a short context manager.  This is
slightly more verbose than sharing one global object, but it makes live
debugging safer: a failed primitive cannot leave a stale GDK session hidden
inside the next step, and every step log shows its own release result.

Usage rule:

Primitive classes should open GDK with this context manager unless they are
wrapping an older controller that already owns GDK initialization. The mission
runner itself should stay free of raw ``agibot_gdk.gdk_init`` calls except for
read-only preflight helpers.
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import Iterator


@contextmanager
def gdk_session() -> Iterator[object]:
    """Initialize GDK, yield the module, and release it best-effort."""

    import agibot_gdk

    result = agibot_gdk.gdk_init()
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    if gdk_res is not None and result not in (None, gdk_res.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")
    try:
        yield agibot_gdk
    finally:
        try:
            agibot_gdk.gdk_release()
            print("GDK release ok", flush=True)
        except Exception as exc:
            print(f"GDK release failed: {exc}", flush=True)
