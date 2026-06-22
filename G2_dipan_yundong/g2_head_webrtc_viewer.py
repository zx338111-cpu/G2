#!/usr/bin/env python3
"""Low-latency WebRTC H.264 viewer for the G2 head camera.

This is a read-only viewer. It opens the GDK head color camera, encodes frames
as H.264 with GStreamer, and serves a browser WebRTC page. It does not command
chassis, arm, waist, gripper, or robot interaction motion APIs.

Run on the robot after sourcing the app environment, for example:

  source /home/agi/app/env.sh
  cd /home/agi/app/gdk/examples/python
  python3 ./g2_head_webrtc_viewer.py --host 0.0.0.0 --port 5060 --fps 20

The existing MJPEG/audio viewer can keep running on port 5055 while this runs
on a different port.
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass, asdict
import json
import signal
import sys
import threading
import time
import uuid
from typing import Any

try:
    import agibot_gdk
except ImportError as exc:  # pragma: no cover - runs on robot.
    raise SystemExit("agibot_gdk is required; run: source /home/agi/app/env.sh") from exc

try:
    import cv2
    import numpy as np
except ImportError as exc:  # pragma: no cover - runs on robot.
    raise SystemExit(f"OpenCV/numpy is required: {exc}") from exc

try:
    from aiohttp import web
except ImportError as exc:  # pragma: no cover - runs on robot.
    raise SystemExit(f"aiohttp is required: {exc}") from exc

try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstWebRTC", "1.0")
    gi.require_version("GstSdp", "1.0")
    from gi.repository import GLib, Gst, GstSdp, GstWebRTC
except (ImportError, ValueError) as exc:  # pragma: no cover - runs on robot.
    raise SystemExit(f"GStreamer Python WebRTC bindings are required: {exc}") from exc


HTML = r"""
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>G2 Head WebRTC</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0f1215;
      --panel: #171d22;
      --line: #2b353d;
      --text: #eef3f7;
      --muted: #9da8b3;
      --ok: #49d17c;
      --bad: #ff6b6b;
      --warn: #f3bd4e;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      font-family: Arial, "Microsoft YaHei", sans-serif;
    }
    .shell {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 330px;
      gap: 12px;
      min-height: 100vh;
      padding: 12px;
    }
    .stage {
      min-width: 0;
      min-height: calc(100vh - 24px);
      position: relative;
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
      background: #050607;
    }
    video {
      width: 100%;
      height: 100%;
      min-height: calc(100vh - 24px);
      display: block;
      object-fit: contain;
      background: #050607;
    }
    .hud {
      position: absolute;
      left: 12px;
      top: 12px;
      display: flex;
      gap: 8px;
      align-items: center;
      padding: 7px 10px;
      border: 1px solid rgba(255, 255, 255, 0.14);
      border-radius: 6px;
      background: rgba(8, 10, 12, 0.80);
      font-size: 14px;
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 50%;
      background: var(--warn);
    }
    .dot.ok { background: var(--ok); }
    .dot.bad { background: var(--bad); }
    .side {
      display: grid;
      grid-template-rows: auto auto minmax(0, 1fr);
      gap: 12px;
      min-height: calc(100vh - 24px);
    }
    .panel {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      padding: 14px;
      min-width: 0;
    }
    h1, h2 {
      margin: 0;
      letter-spacing: 0;
      font-weight: 700;
    }
    h1 { font-size: 18px; }
    h2 {
      color: var(--muted);
      font-size: 15px;
      margin-bottom: 10px;
    }
    .metric {
      display: grid;
      grid-template-columns: 92px minmax(0, 1fr);
      gap: 8px;
      padding: 6px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.06);
      font-size: 14px;
    }
    .metric:last-child { border-bottom: 0; }
    .label { color: var(--muted); }
    .value {
      min-width: 0;
      overflow-wrap: anywhere;
    }
    button {
      width: 100%;
      min-height: 38px;
      border: 1px solid #387f59;
      border-radius: 6px;
      background: #1f7048;
      color: var(--text);
      font-size: 14px;
      font-weight: 700;
      cursor: pointer;
    }
    button:hover { background: #248252; }
    pre {
      margin: 0;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      font-size: 12px;
      line-height: 1.45;
      color: var(--muted);
    }
    @media (max-width: 900px) {
      .shell {
        grid-template-columns: 1fr;
        grid-template-rows: 64vh auto;
      }
      .stage, video {
        min-height: 64vh;
      }
      .side {
        min-height: 0;
        grid-template-rows: auto auto 260px;
      }
    }
  </style>
</head>
<body>
  <main class="shell">
    <section class="stage">
      <video id="video" autoplay playsinline muted></video>
      <div class="hud"><span id="dot" class="dot"></span><span id="headline">WebRTC H.264</span></div>
    </section>
    <aside class="side">
      <section class="panel">
        <h1>G2 Head WebRTC</h1>
      </section>
      <section class="panel">
        <h2>状态</h2>
        <button id="connect" type="button">连接</button>
        <div class="metric"><div class="label">连接</div><div id="conn" class="value">未连接</div></div>
        <div class="metric"><div class="label">ICE</div><div id="ice" class="value">-</div></div>
        <div class="metric"><div class="label">相机</div><div id="camera" class="value">等待中</div></div>
        <div class="metric"><div class="label">会话</div><div id="session" class="value">-</div></div>
      </section>
      <section class="panel">
        <h2>诊断</h2>
        <pre id="diag"></pre>
      </section>
    </aside>
  </main>
  <script>
    const video = document.getElementById("video");
    const connectButton = document.getElementById("connect");
    const conn = document.getElementById("conn");
    const ice = document.getElementById("ice");
    const camera = document.getElementById("camera");
    const session = document.getElementById("session");
    const diag = document.getElementById("diag");
    const dot = document.getElementById("dot");
    const headline = document.getElementById("headline");
    let pc = null;
    let sessionId = "";

    function setDot(state) {
      dot.classList.remove("ok", "bad");
      if (state === "ok") dot.classList.add("ok");
      if (state === "bad") dot.classList.add("bad");
    }

    function waitForIceGathering(peer, timeoutMs = 1500) {
      if (peer.iceGatheringState === "complete") return Promise.resolve();
      return new Promise((resolve) => {
        const timer = setTimeout(resolve, timeoutMs);
        function check() {
          if (peer.iceGatheringState === "complete") {
            clearTimeout(timer);
            peer.removeEventListener("icegatheringstatechange", check);
            resolve();
          }
        }
        peer.addEventListener("icegatheringstatechange", check);
      });
    }

    async function connect() {
      connectButton.disabled = true;
      conn.textContent = "创建连接";
      setDot("");
      if (pc) {
        pc.close();
        pc = null;
      }
      pc = new RTCPeerConnection({iceServers: []});
      pc.addTransceiver("video", {direction: "recvonly"});
      pc.ontrack = (event) => {
        video.srcObject = event.streams[0];
        headline.textContent = "WebRTC H.264 online";
        setDot("ok");
      };
      pc.onconnectionstatechange = () => {
        conn.textContent = pc.connectionState;
        if (["failed", "closed", "disconnected"].includes(pc.connectionState)) {
          setDot("bad");
        }
      };
      pc.oniceconnectionstatechange = () => {
        ice.textContent = pc.iceConnectionState;
      };

      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);
      await waitForIceGathering(pc);
      const response = await fetch("/offer", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(pc.localDescription),
      });
      if (!response.ok) {
        const text = await response.text();
        throw new Error(`${response.status}: ${text}`);
      }
      const answer = await response.json();
      sessionId = answer.session_id || "";
      session.textContent = sessionId || "-";
      await pc.setRemoteDescription(answer);
      connectButton.disabled = false;
      connectButton.textContent = "重新连接";
    }

    async function refreshStatus() {
      try {
        const response = await fetch("/status", {cache: "no-store"});
        const data = await response.json();
        camera.textContent = data.camera.ok
          ? `${data.camera.width}x${data.camera.height} age=${data.camera.age_ms}ms frames=${data.camera.frames}`
          : data.camera.error || "not ready";
        const active = data.sessions[sessionId];
        if (active) {
          diag.textContent = JSON.stringify(active, null, 2);
        } else {
          diag.textContent = JSON.stringify(data, null, 2);
        }
      } catch (error) {
        camera.textContent = String(error);
      }
    }

    connectButton.addEventListener("click", () => {
      connect().catch((error) => {
        conn.textContent = String(error);
        connectButton.disabled = false;
        setDot("bad");
      });
    });
    connect().catch((error) => {
      conn.textContent = String(error);
      connectButton.disabled = false;
      setDot("bad");
    });
    refreshStatus();
    setInterval(refreshStatus, 800);
  </script>
</body>
</html>
"""


@dataclass
class CameraStats:
    ok: bool = False
    width: int = 0
    height: int = 0
    frames: int = 0
    errors: int = 0
    age_ms: int | None = None
    fps: float = 0.0
    error: str = "not started"


class GdkHeadFrameSource:
    """Background reader that keeps only the newest decoded head-color frame."""

    def __init__(self, fps: float):
        self.target_fps = max(1.0, min(30.0, fps))
        self.camera: Any | None = None
        self.frame: np.ndarray | None = None
        self.width = 0
        self.height = 0
        self.frames = 0
        self.errors = 0
        self.error = "not started"
        self.last_frame_time = 0.0
        self.first_frame_time = 0.0
        self.running = False
        self.lock = threading.Lock()
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if self.thread is not None:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, name="gdk-head-frame-source", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.camera is not None:
            try:
                self.camera.close_camera()
            except Exception:
                pass
            self.camera = None

    def latest_frame(self) -> np.ndarray | None:
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def stats(self) -> CameraStats:
        with self.lock:
            now = time.time()
            age_ms = int((now - self.last_frame_time) * 1000) if self.last_frame_time else None
            elapsed = max(0.001, now - self.first_frame_time) if self.first_frame_time else 0.001
            fps = round(self.frames / elapsed, 1) if self.frames else 0.0
            return CameraStats(
                ok=self.frame is not None and age_ms is not None and age_ms < 1000,
                width=self.width,
                height=self.height,
                frames=self.frames,
                errors=self.errors,
                age_ms=age_ms,
                fps=fps,
                error="" if self.frame is not None else self.error,
            )

    def _loop(self) -> None:
        frame_period = 1.0 / self.target_fps
        try:
            self.camera = agibot_gdk.Camera()
            time.sleep(0.5)
            self.error = ""
        except Exception as exc:
            with self.lock:
                self.error = f"{type(exc).__name__}: {exc}"
                self.errors += 1
            return

        while self.running:
            started = time.monotonic()
            try:
                image = self.camera.get_latest_image(agibot_gdk.CameraType.kHeadColor, 1000.0)
                if image is None:
                    raise RuntimeError("no head_color frame")
                decoded = self._decode_to_bgr(image)
                if decoded is None:
                    raise RuntimeError(f"failed to decode {image.encoding} {image.color_format}")
                with self.lock:
                    self.frame = decoded
                    self.height, self.width = decoded.shape[:2]
                    self.frames += 1
                    if not self.first_frame_time:
                        self.first_frame_time = time.time()
                    self.last_frame_time = time.time()
                    self.error = ""
            except Exception as exc:
                with self.lock:
                    self.errors += 1
                    self.error = f"{type(exc).__name__}: {exc}"
            sleep_s = frame_period - (time.monotonic() - started)
            if sleep_s > 0:
                time.sleep(sleep_s)

    @staticmethod
    def _decode_to_bgr(image: Any) -> np.ndarray | None:
        raw = bytes(image.data)
        encoding = str(image.encoding)
        color_format = str(image.color_format)
        if "JPEG" in encoding or "PNG" in encoding:
            return cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if "RGB" in color_format:
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((int(image.height), int(image.width), 3))
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if "BGR" in color_format:
            return np.frombuffer(raw, dtype=np.uint8).reshape((int(image.height), int(image.width), 3))
        if "GRAY8" in color_format:
            gray = np.frombuffer(raw, dtype=np.uint8).reshape((int(image.height), int(image.width)))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return None


def make_pipeline_description(*, width: int, height: int, fps: int, bitrate_kbps: int) -> str:
    """Build a conservative H.264 WebRTC pipeline.

    x264 is used for the default path because appsrc provides CPU memory frames.
    The robot also has NVIDIA H.264 elements, but those need extra NVMM caps and
    should be enabled only after a separate latency/CPU comparison.
    """

    key_int = max(10, fps)
    return (
        "webrtcbin name=webrtc bundle-policy=max-bundle latency=0 "
        "appsrc name=video_src is-live=true block=false format=time do-timestamp=false "
        f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 "
        "! queue max-size-buffers=1 leaky=downstream "
        "! videoconvert "
        "! video/x-raw,format=I420 "
        f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} "
        f"key-int-max={key_int} byte-stream=true threads=2 "
        "! video/x-h264,profile=baseline,stream-format=byte-stream,alignment=au "
        "! h264parse config-interval=-1 "
        "! rtph264pay config-interval=-1 pt=96 mtu=1200 "
        "! application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 "
        "! webrtc."
    )


class WebRtcSession:
    """One browser WebRTC peer backed by one GStreamer pipeline."""

    def __init__(
        self,
        session_id: str,
        frame_source: GdkHeadFrameSource,
        fps: int,
        bitrate_kbps: int,
    ):
        self.session_id = session_id
        self.frame_source = frame_source
        self.fps = fps
        self.bitrate_kbps = bitrate_kbps
        self.pipeline: Gst.Pipeline | None = None
        self.webrtc: Gst.Element | None = None
        self.appsrc: Gst.Element | None = None
        self.created_at = time.time()
        self.last_push_time = 0.0
        self.pushed_frames = 0
        self.push_errors = 0
        self.state = "created"
        self.error = ""
        self.ice_candidates: list[str] = []
        self.ice_lock = threading.Lock()
        self._pts = 0
        self._duration = int(Gst.SECOND / max(1, fps))
        self._push_period_s = 1.0 / max(1, fps)
        self._next_push_time = 0.0
        self._closed = False

    def create_pipeline(self, width: int, height: int) -> None:
        desc = make_pipeline_description(
            width=width,
            height=height,
            fps=self.fps,
            bitrate_kbps=self.bitrate_kbps,
        )
        self.pipeline = Gst.parse_launch(desc)
        self.webrtc = self.pipeline.get_by_name("webrtc")
        self.appsrc = self.pipeline.get_by_name("video_src")
        if self.pipeline is None or self.webrtc is None or self.appsrc is None:
            raise RuntimeError("failed to create WebRTC pipeline")
        self.appsrc.connect("need-data", self._on_need_data)
        self.webrtc.connect("on-ice-candidate", self._on_ice_candidate)
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def close(self) -> None:
        self._closed = True
        self.state = "closed"
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None
        self.webrtc = None
        self.appsrc = None

    def status(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "age_s": round(time.time() - self.created_at, 1),
            "state": self.state,
            "error": self.error,
            "pushed_frames": self.pushed_frames,
            "push_errors": self.push_errors,
            "last_push_age_ms": int((time.time() - self.last_push_time) * 1000) if self.last_push_time else None,
            "ice_candidates": len(self.ice_candidates),
            "fps": self.fps,
            "bitrate_kbps": self.bitrate_kbps,
        }

    async def handle_offer(self, sdp_text: str) -> str:
        stats = self.frame_source.stats()
        if not stats.ok:
            raise RuntimeError(f"camera not ready: {stats.error}")
        self.create_pipeline(width=stats.width, height=stats.height)
        assert self.pipeline is not None
        assert self.webrtc is not None

        loop = asyncio.get_running_loop()
        answer_ready: asyncio.Future[str] = loop.create_future()

        def on_answer_created(promise: Gst.Promise, _user_data: object | None = None) -> None:
            try:
                reply = promise.get_reply()
                if reply is None:
                    raise RuntimeError("create-answer returned no reply")
                answer = reply.get_value("answer")
                if answer is None:
                    raise RuntimeError(f"create-answer returned no answer: {reply.to_string()}")
                self.webrtc.emit("set-local-description", answer, Gst.Promise.new())
                answer_text = answer.sdp.as_text()
                if not answer_ready.done():
                    loop.call_soon_threadsafe(answer_ready.set_result, answer_text)
            except Exception as exc:
                if not answer_ready.done():
                    loop.call_soon_threadsafe(answer_ready.set_exception, exc)

        ret, sdp_msg = GstSdp.SDPMessage.new()
        if ret != GstSdp.SDPResult.OK:
            raise RuntimeError(f"failed to allocate SDP message: {ret}")
        parse_ret = GstSdp.sdp_message_parse_buffer(sdp_text.encode("utf-8"), sdp_msg)
        if parse_ret != GstSdp.SDPResult.OK:
            raise RuntimeError(f"failed to parse browser offer SDP: {parse_ret}")
        offer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.OFFER, sdp_msg)

        self.pipeline.set_state(Gst.State.PLAYING)
        remote_promise = Gst.Promise.new()
        self.webrtc.emit("set-remote-description", offer, remote_promise)
        remote_promise.wait()
        remote_reply = remote_promise.get_reply()
        if remote_reply is not None and "error" in remote_reply.to_string().lower():
            raise RuntimeError(f"set-remote-description failed: {remote_reply.to_string()}")
        promise = Gst.Promise.new_with_change_func(on_answer_created, None)
        self.webrtc.emit("create-answer", None, promise)
        self.state = "playing"
        answer_text = await asyncio.wait_for(answer_ready, timeout=5.0)

        # The page uses a single HTTP /offer exchange rather than trickle ICE.
        # Wait briefly for local host candidates and fold them into the answer
        # SDP so the browser can establish the media path immediately.
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            with self.ice_lock:
                if self.ice_candidates:
                    break
            await asyncio.sleep(0.05)
        return self._answer_sdp_with_candidates(answer_text)

    def _on_ice_candidate(self, _webrtc: Gst.Element, _mline_index: int, candidate: str) -> None:
        if not candidate:
            return
        with self.ice_lock:
            if candidate not in self.ice_candidates:
                self.ice_candidates.append(candidate)

    def _answer_sdp_with_candidates(self, answer_text: str) -> str:
        with self.ice_lock:
            candidates = list(self.ice_candidates)
        if not candidates:
            return answer_text
        newline = "\r\n" if "\r\n" in answer_text else "\n"
        stripped = answer_text.rstrip("\r\n")
        extra = [f"a={candidate}" for candidate in candidates]
        extra.append("a=end-of-candidates")
        return stripped + newline + newline.join(extra) + newline

    def _on_need_data(self, appsrc: Gst.Element, _length: int) -> None:
        if self._closed:
            return
        now = time.monotonic()
        if self._next_push_time > now:
            time.sleep(min(0.2, self._next_push_time - now))
            now = time.monotonic()
        if self._next_push_time <= 0.0 or self._next_push_time < now - self._push_period_s:
            self._next_push_time = now + self._push_period_s
        else:
            self._next_push_time += self._push_period_s

        frame = self.frame_source.latest_frame()
        if frame is None:
            return
        try:
            data = frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.pts = self._pts
            buf.dts = self._pts
            buf.duration = self._duration
            self._pts += self._duration
            result = appsrc.emit("push-buffer", buf)
            if result != Gst.FlowReturn.OK:
                raise RuntimeError(f"push-buffer returned {result}")
            self.pushed_frames += 1
            self.last_push_time = time.time()
        except Exception as exc:
            self.push_errors += 1
            self.error = f"{type(exc).__name__}: {exc}"

    def _on_bus_message(self, _bus: Gst.Bus, message: Gst.Message) -> None:
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.error = f"{err}: {debug}"
            self.state = "error"
        elif message.type == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            self.error = f"warning: {err}: {debug}"
        elif message.type == Gst.MessageType.EOS:
            self.state = "eos"


class WebRtcServer:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.frame_source = GdkHeadFrameSource(fps=args.fps)
        self.sessions: dict[str, WebRtcSession] = {}
        self.sessions_lock = threading.Lock()
        self.glib_loop = GLib.MainLoop()
        self.glib_thread = threading.Thread(target=self.glib_loop.run, name="glib-mainloop", daemon=True)
        self.app = web.Application()
        self.app.add_routes(
            [
                web.get("/", self.index),
                web.get("/status", self.status),
                web.post("/offer", self.offer),
            ]
        )

    def start_background(self) -> None:
        Gst.init(None)
        self.glib_thread.start()
        self.frame_source.start()

    def stop_background(self) -> None:
        with self.sessions_lock:
            sessions = list(self.sessions.values())
            self.sessions.clear()
        for session in sessions:
            session.close()
        self.frame_source.stop()
        if self.glib_loop.is_running():
            self.glib_loop.quit()

    async def index(self, _request: web.Request) -> web.Response:
        return web.Response(text=HTML, content_type="text/html")

    async def status(self, _request: web.Request) -> web.Response:
        with self.sessions_lock:
            sessions = {sid: session.status() for sid, session in self.sessions.items()}
        return web.json_response(
            {
                "camera": asdict(self.frame_source.stats()),
                "sessions": sessions,
                "server": {
                    "port": self.args.port,
                    "fps": self.args.fps,
                    "bitrate_kbps": self.args.bitrate_kbps,
                },
            }
        )

    async def offer(self, request: web.Request) -> web.Response:
        payload = await request.json()
        if payload.get("type") != "offer" or not isinstance(payload.get("sdp"), str):
            raise web.HTTPBadRequest(text="expected JSON WebRTC offer with type/sdp")
        self._cleanup_old_sessions(max_age_s=120.0)
        session_id = uuid.uuid4().hex[:10]
        session = WebRtcSession(
            session_id=session_id,
            frame_source=self.frame_source,
            fps=int(self.args.fps),
            bitrate_kbps=int(self.args.bitrate_kbps),
        )
        with self.sessions_lock:
            self.sessions[session_id] = session
        try:
            answer_sdp = await session.handle_offer(payload["sdp"])
        except Exception as exc:
            session.close()
            with self.sessions_lock:
                self.sessions.pop(session_id, None)
            raise web.HTTPInternalServerError(text=f"{type(exc).__name__}: {exc}") from exc
        return web.json_response({"type": "answer", "sdp": answer_sdp, "session_id": session_id})

    def _cleanup_old_sessions(self, max_age_s: float) -> None:
        now = time.time()
        expired: list[WebRtcSession] = []
        with self.sessions_lock:
            for sid, session in list(self.sessions.items()):
                if now - session.created_at > max_age_s or session.state in {"closed", "error", "eos"}:
                    expired.append(self.sessions.pop(sid))
        for session in expired:
            session.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5060)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--bitrate-kbps", type=int, default=1200)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.fps = int(max(1, min(30, round(args.fps))))
    args.bitrate_kbps = int(max(300, min(8000, args.bitrate_kbps)))
    server = WebRtcServer(args)

    def request_shutdown(_signum: int, _frame: Any) -> None:
        server.stop_background()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    server.start_background()
    print(f"serving WebRTC H.264 viewer on http://{args.host}:{args.port}", flush=True)
    try:
        web.run_app(server.app, host=args.host, port=args.port, print=None)
    finally:
        server.stop_background()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
