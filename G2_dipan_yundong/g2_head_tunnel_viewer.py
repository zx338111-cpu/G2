#!/usr/bin/env python3
"""Tunnel-friendly low-latency head-camera viewer for G2.

This viewer is designed for SSH local port forwarding.  It does not use WebRTC
or ICE, because those media packets do not naturally travel through a simple
``ssh -L`` tunnel.  Instead it sends latest-only JPEG frames over one WebSocket
TCP connection and raw head-board microphone PCM over HTTP/TCP.

Run on the robot:

  source /home/agi/app/env.sh
  cd /home/agi/app/gdk/examples/python
  python3 ./g2_head_tunnel_viewer.py --host 127.0.0.1 --port 5061 --fps 20 \
    --video-profile clear \
    --head-audio-type aec.pcm --audio-sample-rate 64000 --audio-channels 1 \
    --playback-sample-rate 16000

Open a tunnel from your workstation:

  ssh -L 15061:127.0.0.1:5061 agi@10.185.207.191

Then open:

  http://127.0.0.1:15061/

The program is read-only. It opens the GDK head-color camera and never commands
chassis, arm, waist, gripper, or interaction motion APIs.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
from dataclasses import asdict, dataclass
import json
import os
import queue
import signal
import socket
import struct
import threading
import time
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
    from aiohttp.client_exceptions import ClientConnectionResetError
except ImportError as exc:  # pragma: no cover - runs on robot.
    raise SystemExit(f"aiohttp is required: {exc}") from exc


HTML = r"""
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>G2 Tunnel Camera</title>
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
      overflow: hidden;
      background: var(--bg);
      color: var(--text);
      font-family: Arial, "Microsoft YaHei", sans-serif;
    }
    .shell {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 330px;
      gap: 12px;
      height: 100vh;
      min-height: 0;
      padding: 12px;
    }
    .stage {
      min-width: 0;
      height: calc(100vh - 24px);
      min-height: 0;
      position: relative;
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
      background: #050607;
    }
    canvas {
      width: 100%;
      height: 100%;
      min-height: 0;
      display: block;
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
      grid-template-rows: auto auto auto auto minmax(0, 1fr);
      gap: 12px;
      height: calc(100vh - 24px);
      min-height: 0;
      overflow: hidden;
    }
    .panel {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      padding: 14px;
      min-width: 0;
    }
    .side .panel:last-child {
      min-height: 0;
      overflow: auto;
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
    .segmented {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }
    .segmented button {
      min-height: 34px;
      border-color: #35424b;
      background: #202a31;
      font-weight: 600;
    }
    .segmented button.active {
      border-color: #4ca56d;
      background: #1f7048;
    }
    pre {
      margin: 0;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      font-size: 12px;
      line-height: 1.45;
      color: var(--muted);
      min-height: 0;
    }
    @media (max-width: 900px) {
      body {
        overflow: auto;
      }
      .shell {
        grid-template-columns: 1fr;
        grid-template-rows: 64vh auto;
        height: auto;
        min-height: 100vh;
      }
      .stage, canvas {
        height: 64vh;
        min-height: 0;
      }
      .side {
        height: auto;
        min-height: 0;
        overflow: visible;
        grid-template-rows: auto auto auto auto 260px;
      }
    }
  </style>
</head>
<body>
  <main class="shell">
    <section class="stage">
      <canvas id="canvas"></canvas>
      <div class="hud"><span id="dot" class="dot"></span><span id="headline">Tunnel JPEG</span></div>
    </section>
    <aside class="side">
      <section class="panel">
        <h1>G2 Tunnel Camera</h1>
      </section>
      <section class="panel">
        <h2>状态</h2>
        <button id="connect" type="button">重新连接</button>
        <div class="metric"><div class="label">连接</div><div id="conn" class="value">未连接</div></div>
        <div class="metric"><div class="label">画面</div><div id="video" class="value">等待中</div></div>
        <div class="metric"><div class="label">接收</div><div id="latency" class="value">-</div></div>
        <div class="metric"><div class="label">帧率</div><div id="fps" class="value">-</div></div>
        <div class="metric"><div class="label">模式</div><div id="profile" class="value">清晰</div></div>
        <div class="metric"><div class="label">声音</div><div id="audio" class="value">点击开启</div></div>
      </section>
      <section class="panel">
        <h2>画质</h2>
        <div class="segmented">
          <button type="button" data-profile="clear">清晰</button>
          <button type="button" data-profile="smooth">流畅</button>
          <button type="button" data-profile="saving">省流</button>
          <button type="button" data-profile="voice">语音</button>
        </div>
      </section>
      <section class="panel">
        <h2>声音</h2>
        <button id="audio-button" type="button">开启声音</button>
      </section>
      <section class="panel">
        <h2>诊断</h2>
        <pre id="diag"></pre>
      </section>
    </aside>
  </main>
  <script>
    const canvas = document.getElementById("canvas");
    const ctx = canvas.getContext("2d", {alpha: false});
    const connectButton = document.getElementById("connect");
    const conn = document.getElementById("conn");
    const video = document.getElementById("video");
    const latency = document.getElementById("latency");
    const fps = document.getElementById("fps");
    const profileValue = document.getElementById("profile");
    const diag = document.getElementById("diag");
    const dot = document.getElementById("dot");
    const headline = document.getElementById("headline");
    const audioValue = document.getElementById("audio");
    const audioButton = document.getElementById("audio-button");
    const profileButtons = Array.from(document.querySelectorAll("[data-profile]"));
    const AUDIO_ENABLED = __AUDIO_ENABLED__;
    const SOURCE_RATE = __AUDIO_PLAYBACK_RATE__;
    const SOURCE_CHANNELS = __AUDIO_PLAYBACK_CHANNELS__;
    const BYTES_PER_SAMPLE = 2;
    let ws = null;
    let frames = 0;
    let lastFrameNo = 0;
    let lastFpsTime = performance.now();
    let lastDrawTime = 0;
    let droppedFrames = 0;
    let bytesInWindow = 0;
    let drawing = false;
    let latestPayload = null;
    let audioContext = null;
    let audioNode = null;
    let audioAbort = null;
    let sourceChunks = [];
    let sourceQueued = 0;
    let activeChunk = null;
    let activeOffset = 0;
    let resamplePhase = 0;
    let lastSample = 0;
    let audioStarted = false;
    let audioBytes = 0;
    let audioUnderruns = 0;
    let videoReconnectTimer = null;
    let videoReconnectAttempt = 0;

    function setDot(state) {
      dot.classList.remove("ok", "bad");
      if (state === "ok") dot.classList.add("ok");
      if (state === "bad") dot.classList.add("bad");
    }

    function fitAndDraw(bitmap) {
      if (canvas.width !== canvas.clientWidth || canvas.height !== canvas.clientHeight) {
        canvas.width = Math.max(1, canvas.clientWidth);
        canvas.height = Math.max(1, canvas.clientHeight);
      }
      ctx.fillStyle = "#050607";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      const scale = Math.min(canvas.width / bitmap.width, canvas.height / bitmap.height);
      const w = Math.round(bitmap.width * scale);
      const h = Math.round(bitmap.height * scale);
      const x = Math.round((canvas.width - w) / 2);
      const y = Math.round((canvas.height - h) / 2);
      ctx.drawImage(bitmap, x, y, w, h);
    }

    async function drawLatest() {
      if (drawing || latestPayload === null) return;
      drawing = true;
      const payload = latestPayload;
      latestPayload = null;
      try {
        const view = new DataView(payload.buffer);
        const frameNo = view.getUint32(8, false);
        const jpeg = payload.buffer.slice(12);
        const bitmap = await createImageBitmap(new Blob([jpeg], {type: "image/jpeg"}));
        const imageWidth = bitmap.width;
        const imageHeight = bitmap.height;
        fitAndDraw(bitmap);
        bitmap.close();
        const now = performance.now();
        const receiveToDrawMs = Math.max(0, Math.round(now - payload.receivedAt));
        const intervalMs = lastDrawTime ? Math.max(0, Math.round(now - lastDrawTime)) : 0;
        if (lastFrameNo && frameNo > lastFrameNo + 1) {
          droppedFrames += frameNo - lastFrameNo - 1;
        }
        lastDrawTime = now;
        frames += 1;
        bytesInWindow += payload.byteLength;
        lastFrameNo = frameNo;
        video.textContent = `${imageWidth || 640}x${imageHeight || 400} frame=${frameNo}`;
        latency.textContent = `${receiveToDrawMs} ms 渲染 / ${intervalMs} ms 间隔`;
        headline.textContent = "Tunnel JPEG online";
        setDot("ok");
        if (now - lastFpsTime >= 1000) {
          const seconds = Math.max(0.001, (now - lastFpsTime) / 1000);
          const fpsNow = frames / seconds;
          const mbps = (bytesInWindow * 8) / seconds / 1000000;
          fps.textContent = `${fpsNow.toFixed(1)} FPS / ${mbps.toFixed(1)} Mbps / 丢=${droppedFrames}`;
          frames = 0;
          bytesInWindow = 0;
          lastFpsTime = now;
        }
      } catch (error) {
        diag.textContent = String(error);
        setDot("bad");
      } finally {
        drawing = false;
        if (latestPayload !== null) requestAnimationFrame(drawLatest);
      }
    }

    function connect() {
      if (videoReconnectTimer) {
        clearTimeout(videoReconnectTimer);
        videoReconnectTimer = null;
      }
      if (ws) {
        ws.onclose = null;
        ws.onerror = null;
        ws.close();
      }
      frames = 0;
      lastFrameNo = 0;
      lastFpsTime = performance.now();
      lastDrawTime = 0;
      droppedFrames = 0;
      bytesInWindow = 0;
      latestPayload = null;
      conn.textContent = "连接中";
      setDot("");
      const scheme = location.protocol === "https:" ? "wss" : "ws";
      const currentWs = new WebSocket(`${scheme}://${location.host}/frames`);
      ws = currentWs;
      currentWs.binaryType = "arraybuffer";
      currentWs.onopen = () => {
        videoReconnectAttempt = 0;
        conn.textContent = "已连接";
        connectButton.disabled = false;
      };
      currentWs.onmessage = (event) => {
        latestPayload = {
          buffer: event.data,
          byteLength: event.data.byteLength,
          receivedAt: performance.now(),
        };
        requestAnimationFrame(drawLatest);
      };
      currentWs.onclose = () => {
        if (ws !== currentWs) return;
        conn.textContent = "已断开，重连中";
        setDot("bad");
        const delayMs = Math.min(3000, 500 + videoReconnectAttempt * 500);
        videoReconnectAttempt += 1;
        videoReconnectTimer = setTimeout(connect, delayMs);
      };
      currentWs.onerror = () => {
        if (ws !== currentWs) return;
        conn.textContent = "连接错误";
        setDot("bad");
        currentWs.close();
      };
    }

    function resetAudioQueue() {
      sourceChunks = [];
      sourceQueued = 0;
      activeChunk = null;
      activeOffset = 0;
      resamplePhase = 0;
      lastSample = 0;
      audioStarted = false;
      audioBytes = 0;
      audioUnderruns = 0;
    }

    function pushPcmBytes(bytes) {
      const frameBytes = SOURCE_CHANNELS * BYTES_PER_SAMPLE;
      const frameCount = Math.floor(bytes.byteLength / frameBytes);
      if (!frameCount) return;
      const view = new DataView(bytes.buffer, bytes.byteOffset, frameCount * frameBytes);
      const samples = new Float32Array(frameCount);
      for (let frame = 0; frame < frameCount; frame += 1) {
        let sum = 0;
        for (let channel = 0; channel < SOURCE_CHANNELS; channel += 1) {
          const offset = (frame * SOURCE_CHANNELS + channel) * BYTES_PER_SAMPLE;
          sum += view.getInt16(offset, true) / 32768;
        }
        samples[frame] = sum / SOURCE_CHANNELS;
      }
      sourceChunks.push(samples);
      sourceQueued += samples.length;
      audioBytes += frameCount * frameBytes;
      trimAudioQueue();
    }

    function trimAudioQueue() {
      const maxSamples = Math.floor(SOURCE_RATE * 5.0);
      while (sourceQueued > maxSamples && sourceChunks.length > 1) {
        const dropped = sourceChunks.shift();
        sourceQueued -= dropped.length;
      }
    }

    function popSourceSample() {
      while (!activeChunk || activeOffset >= activeChunk.length) {
        activeChunk = sourceChunks.shift() || null;
        activeOffset = 0;
        if (!activeChunk) {
          audioUnderruns += 1;
          return 0;
        }
      }
      const value = activeChunk[activeOffset];
      activeOffset += 1;
      sourceQueued -= 1;
      return value;
    }

    function fillAudioOutput(event) {
      const output = event.outputBuffer.getChannelData(0);
      const startThreshold = Math.floor(SOURCE_RATE * 1.2);
      if (!audioStarted && sourceQueued < startThreshold) {
        output.fill(0);
      } else {
        audioStarted = true;
        const ratio = SOURCE_RATE / audioContext.sampleRate;
        for (let i = 0; i < output.length; i += 1) {
          resamplePhase += ratio;
          while (resamplePhase >= 1) {
            lastSample = popSourceSample();
            resamplePhase -= 1;
          }
          output[i] = lastSample;
        }
      }
      const queuedMs = Math.round(sourceQueued / SOURCE_RATE * 1000);
      audioValue.textContent = `${queuedMs} ms 缓冲 / ${audioUnderruns} 次空包`;
      audioButton.textContent = "关闭声音";
    }

    async function readPcmStream(signal) {
      const response = await fetch(`/audio.pcm?ts=${Date.now()}`, {cache: "no-store", signal});
      if (!response.ok || !response.body) {
        throw new Error(`audio HTTP ${response.status}`);
      }
      const reader = response.body.getReader();
      let carry = new Uint8Array(0);
      const frameBytes = SOURCE_CHANNELS * BYTES_PER_SAMPLE;
      while (true) {
        const {done, value} = await reader.read();
        if (done) break;
        let merged = value;
        if (carry.length) {
          merged = new Uint8Array(carry.length + value.length);
          merged.set(carry, 0);
          merged.set(value, carry.length);
          carry = new Uint8Array(0);
        }
        const aligned = merged.length - (merged.length % frameBytes);
        if (aligned > 0) {
          pushPcmBytes(merged.subarray(0, aligned));
        }
        if (aligned < merged.length) {
          carry = merged.subarray(aligned);
        }
      }
      if (!signal.aborted) {
        throw new Error("audio stream ended");
      }
    }

    async function startAudio() {
      if (!AUDIO_ENABLED) {
        audioValue.textContent = "服务端未启用";
        return;
      }
      await stopAudio();
      resetAudioQueue();
      const AudioContextClass = window.AudioContext || window.webkitAudioContext;
      if (!AudioContextClass) {
        audioValue.textContent = "浏览器不支持 WebAudio";
        return;
      }
      audioAbort = new AbortController();
      audioContext = new AudioContextClass({latencyHint: "interactive"});
      await audioContext.resume();
      audioNode = audioContext.createScriptProcessor(8192, 0, 1);
      audioNode.onaudioprocess = fillAudioOutput;
      audioNode.connect(audioContext.destination);
      audioButton.textContent = "声音缓冲中";
      audioValue.textContent = "连接音频流";
      readPcmStream(audioAbort.signal).catch((error) => {
        if (error.name !== "AbortError") {
          audioValue.textContent = `声音中断: ${error}`;
          audioButton.textContent = "重新开启声音";
        }
      });
    }

    async function stopAudio() {
      if (audioAbort) {
        audioAbort.abort();
        audioAbort = null;
      }
      if (audioNode) {
        audioNode.disconnect();
        audioNode = null;
      }
      if (audioContext) {
        const ctx = audioContext;
        audioContext = null;
        await ctx.close().catch(() => {});
      }
      resetAudioQueue();
      audioButton.textContent = "开启声音";
    }

    async function toggleAudio() {
      if (audioContext || audioAbort) {
        await stopAudio();
      } else {
        await startAudio();
      }
    }

    function renderVideoProfile(data) {
      const server = data.server || {};
      const profiles = server.video_profiles || {};
      const current = server.video_profile || "clear";
      const profile = profiles[current] || {};
      const label = profile.label || current;
      const targetFps = server.fps || profile.fps || "-";
      const quality = server.jpeg_quality || profile.jpeg_quality || "-";
      profileValue.textContent = `${label} ${targetFps}FPS q=${quality}`;
      profileButtons.forEach((button) => {
        button.classList.toggle("active", button.dataset.profile === current);
      });
    }

    async function setVideoProfile(profile) {
      profileButtons.forEach((button) => { button.disabled = true; });
      try {
        const response = await fetch("/video-profile", {
          method: "POST",
          cache: "no-store",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({profile}),
        });
        if (!response.ok) throw new Error(`profile HTTP ${response.status}`);
        const data = await response.json();
        renderVideoProfile(data);
      } catch (error) {
        profileValue.textContent = String(error);
      } finally {
        profileButtons.forEach((button) => { button.disabled = false; });
      }
    }

    async function refreshStatus() {
      try {
        const response = await fetch("/status", {cache: "no-store"});
        const data = await response.json();
        renderVideoProfile(data);
        if (!audioContext) {
          audioValue.textContent = data.audio.ok ? data.audio.info : data.audio.error;
        }
        diag.textContent = JSON.stringify({...data, lastFrameNo}, null, 2);
      } catch (error) {
        diag.textContent = String(error);
      }
    }

    connectButton.addEventListener("click", connect);
    audioButton.addEventListener("click", toggleAudio);
    profileButtons.forEach((button) => {
      button.addEventListener("click", () => setVideoProfile(button.dataset.profile));
    });
    if (!AUDIO_ENABLED) {
      audioButton.disabled = true;
      audioValue.textContent = "服务端未启用";
    }
    connect();
    refreshStatus();
    setInterval(refreshStatus, 1500);
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
    bytes: int = 0
    error: str = "not started"


VIDEO_PROFILES: dict[str, dict[str, int | str]] = {
    "clear": {"label": "清晰", "fps": 20, "jpeg_quality": 72},
    "smooth": {"label": "流畅", "fps": 15, "jpeg_quality": 60},
    "saving": {"label": "省流", "fps": 10, "jpeg_quality": 52},
    "voice": {"label": "语音", "fps": 2, "jpeg_quality": 32},
}


class LatestJpegSource:
    def __init__(self, *, fps: float, jpeg_quality: int):
        self.target_fps = max(1.0, min(30.0, fps))
        self.jpeg_quality = max(30, min(95, jpeg_quality))
        self.camera: Any | None = None
        self.jpeg: bytes | None = None
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
        self.thread = threading.Thread(target=self._loop, name="latest-jpeg-source", daemon=True)
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

    def latest(self) -> tuple[int, float, int, int, bytes] | None:
        with self.lock:
            if self.jpeg is None:
                return None
            return self.frames, self.last_frame_time, self.width, self.height, self.jpeg

    def set_settings(self, *, fps: int, jpeg_quality: int) -> None:
        with self.lock:
            self.target_fps = max(1.0, min(30.0, float(fps)))
            self.jpeg_quality = max(30, min(95, int(jpeg_quality)))

    def settings(self) -> dict[str, Any]:
        with self.lock:
            return {
                "fps": int(round(self.target_fps)),
                "jpeg_quality": int(self.jpeg_quality),
            }

    def stats(self) -> CameraStats:
        with self.lock:
            now = time.time()
            age_ms = int((now - self.last_frame_time) * 1000) if self.last_frame_time else None
            elapsed = max(0.001, now - self.first_frame_time) if self.first_frame_time else 0.001
            fps = round(self.frames / elapsed, 1) if self.frames else 0.0
            return CameraStats(
                ok=self.jpeg is not None and age_ms is not None and age_ms < 1000,
                width=self.width,
                height=self.height,
                frames=self.frames,
                errors=self.errors,
                age_ms=age_ms,
                fps=fps,
                bytes=len(self.jpeg or b""),
                error="" if self.jpeg is not None else self.error,
            )

    def _loop(self) -> None:
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
            with self.lock:
                target_fps = self.target_fps
            try:
                image = self.camera.get_latest_image(agibot_gdk.CameraType.kHeadColor, 1000.0)
                if image is None:
                    raise RuntimeError("no head_color frame")
                jpeg = self._to_jpeg(image)
                if jpeg is None:
                    raise RuntimeError(f"failed to encode {image.encoding} {image.color_format}")
                with self.lock:
                    self.jpeg = jpeg
                    self.width = int(image.width)
                    self.height = int(image.height)
                    self.frames += 1
                    if not self.first_frame_time:
                        self.first_frame_time = time.time()
                    self.last_frame_time = time.time()
                    self.error = ""
            except Exception as exc:
                with self.lock:
                    self.errors += 1
                    self.error = f"{type(exc).__name__}: {exc}"
            period = 1.0 / max(1.0, target_fps)
            sleep_s = period - (time.monotonic() - started)
            if sleep_s > 0:
                time.sleep(sleep_s)

    def _to_jpeg(self, image: Any) -> bytes | None:
        raw = bytes(image.data)
        encoding = str(image.encoding)
        color_format = str(image.color_format)
        with self.lock:
            jpeg_quality = self.jpeg_quality
        if "JPEG" in encoding and jpeg_quality >= 80:
            return raw
        if "JPEG" in encoding or "PNG" in encoding:
            arr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        elif "RGB" in color_format:
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((int(image.height), int(image.width), 3))
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif "BGR" in color_format:
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((int(image.height), int(image.width), 3))
        elif "GRAY8" in color_format:
            gray = np.frombuffer(raw, dtype=np.uint8).reshape((int(image.height), int(image.width)))
            arr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            return None
        ok, buf = cv2.imencode(".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
        return buf.tobytes() if ok else None


class HeadLocalPcmSource:
    """Subscribe to the head-board AISpeech PCM websocket and fan it out.

    The browser still has to click once to play audio. The source thread starts
    with the viewer so `/status` can report whether microphone PCM is arriving.
    """

    def __init__(
        self,
        *,
        host: str,
        port: int,
        stream_type: str,
        sample_rate: int,
        channels: int,
        playback_sample_rate: int,
    ):
        self.host = host
        self.port = port
        self.stream_type = stream_type
        self.sample_rate = sample_rate
        self.channels = channels
        self.playback_sample_rate = playback_sample_rate
        self.playback_channels = 1
        self.bytes_per_sample = 2
        self.running = False
        self.error = "not started"
        self.info = ""
        self.packet_count = 0
        self.byte_count = 0
        self.last_packet_time = 0.0
        self.last_text_event = ""
        self.reconnect_count = 0
        self.lock = threading.Lock()
        self.clients: set[queue.Queue[bytes]] = set()
        self.thread: threading.Thread | None = None
        self.sock: socket.socket | None = None

    def start(self) -> None:
        if self.thread is not None:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, name="head-local-pcm-source", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        with self.lock:
            self.clients.clear()
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def subscribe(self) -> queue.Queue[bytes]:
        client: queue.Queue[bytes] = queue.Queue(maxsize=512)
        with self.lock:
            self.clients.add(client)
        return client

    def unsubscribe(self, client: queue.Queue[bytes]) -> None:
        with self.lock:
            self.clients.discard(client)

    def playback_silence(self, seconds: float = 0.04) -> bytes:
        frame_count = max(1, int(self.playback_sample_rate * seconds))
        return b"\x00" * frame_count * self.playback_channels * self.bytes_per_sample

    def convert_for_playback(self, data: bytes) -> bytes:
        frame_size = self.channels * self.bytes_per_sample
        usable = len(data) - (len(data) % frame_size)
        if usable <= 0:
            return b""
        samples = np.frombuffer(data[:usable], dtype="<i2")
        if self.channels > 1:
            samples = samples.reshape((-1, self.channels)).mean(axis=1).astype("<i2")
        if self.sample_rate != self.playback_sample_rate:
            ratio = max(1, int(round(self.sample_rate / self.playback_sample_rate)))
            samples = samples[::ratio]
        return samples.astype("<i2", copy=False).tobytes()

    def status(self) -> dict[str, Any]:
        with self.lock:
            packet_count = self.packet_count
            byte_count = self.byte_count
            last_packet_time = self.last_packet_time
            client_count = len(self.clients)
            error = self.error
            info = self.info
            reconnect_count = self.reconnect_count
            last_text_event = self.last_text_event
        age = time.time() - last_packet_time if last_packet_time else None
        recent = age is not None and age < 2.0
        common = {
            "packets": packet_count,
            "bytes": byte_count,
            "last_packet_age_s": age,
            "clients": client_count,
            "reconnects": reconnect_count,
            "sample_rate": self.sample_rate,
            "channels": self.channels,
            "playback_sample_rate": self.playback_sample_rate,
            "playback_channels": self.playback_channels,
            "stream_type": self.stream_type,
            "last_text_event": last_text_event,
        }
        if error and not packet_count:
            return {"ok": False, "info": info, "error": error, **common}
        if not packet_count:
            return {
                "ok": False,
                "info": info,
                "error": f"connected; waiting for {self.stream_type} frames",
                **common,
            }
        return {
            "ok": recent,
            "info": (
                f"{self.sample_rate}Hz {self.channels}ch {self.stream_type} "
                f"playback={self.playback_sample_rate}Hz mono "
                f"packets={packet_count} last={age:.1f}s clients={client_count}"
            ),
            "error": f"last PCM frame {age:.1f}s ago" if age is not None else "",
            **common,
        }

    def _run_loop(self) -> None:
        while self.running:
            try:
                self._connect_and_stream()
            except Exception as exc:
                with self.lock:
                    self.error = f"{type(exc).__name__}: {exc}"
                    self.reconnect_count += 1
                if self.running:
                    print(f"head local PCM reconnect after: {self.error}", flush=True)
                    time.sleep(1.0)

    def _connect_and_stream(self) -> None:
        sock = socket.create_connection((self.host, self.port), timeout=3)
        sock.settimeout(1.0)
        self.sock = sock
        self._websocket_handshake(sock)
        self._send_text(
            sock,
            {
                "jsonrpc": "2.0",
                "id": "set_pcm",
                "method": "/downstream/setBinaryType",
                "params": {"type": self.stream_type},
            },
        )
        with self.lock:
            self.error = ""
            self.info = f"connected to {self.host}:{self.port} type={self.stream_type}"
        print(f"head local PCM connected: {self.info}", flush=True)

        while self.running:
            try:
                opcode, data = self._recv_frame(sock)
            except socket.timeout:
                continue
            if opcode == 1:
                self._handle_text_event(data)
            elif opcode == 2:
                self._handle_pcm_frame(data)
            elif opcode == 8:
                raise ConnectionError("websocket closed by peer")
            elif opcode == 9:
                self._send_pong(sock, data)

    def _websocket_handshake(self, sock: socket.socket) -> None:
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            "GET / HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        ).encode("ascii")
        sock.sendall(request)
        response = b""
        while b"\r\n\r\n" not in response:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("empty websocket handshake response")
            response += chunk
            if len(response) > 16384:
                raise ConnectionError("websocket handshake response too large")
        if b" 101 " not in response.split(b"\r\n", 1)[0]:
            raise ConnectionError(response[:200].decode("latin1", "replace"))

    def _handle_text_event(self, data: bytes) -> None:
        text = data.decode("utf-8", "replace")
        with self.lock:
            self.last_text_event = text[:500]
            if '"id":"set_pcm"' in text and '"code":200' not in text:
                self.error = text[:500]

    def _handle_pcm_frame(self, data: bytes) -> None:
        if not data:
            return
        now = time.time()
        with self.lock:
            self.packet_count += 1
            self.byte_count += len(data)
            self.last_packet_time = now
            clients = list(self.clients)
        for client in clients:
            self._put_drop_oldest(client, data)

    @staticmethod
    def _recv_exact(sock: socket.socket, length: int) -> bytes:
        data = b""
        while len(data) < length:
            chunk = sock.recv(length - len(data))
            if not chunk:
                raise EOFError("socket closed")
            data += chunk
        return data

    def _recv_frame(self, sock: socket.socket) -> tuple[int, bytes]:
        header = self._recv_exact(sock, 2)
        first, second = header
        opcode = first & 0x0F
        masked = bool(second & 0x80)
        length = second & 0x7F
        if length == 126:
            length = struct.unpack("!H", self._recv_exact(sock, 2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self._recv_exact(sock, 8))[0]
        mask = self._recv_exact(sock, 4) if masked else b""
        payload = self._recv_exact(sock, length) if length else b""
        if masked:
            payload = bytes(payload[i] ^ mask[i % 4] for i in range(len(payload)))
        return opcode, payload

    def _send_text(self, sock: socket.socket, obj: dict[str, Any]) -> None:
        payload = json.dumps(obj, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self._send_frame(sock, opcode=1, payload=payload)

    def _send_pong(self, sock: socket.socket, payload: bytes) -> None:
        self._send_frame(sock, opcode=10, payload=payload)

    @staticmethod
    def _send_frame(sock: socket.socket, opcode: int, payload: bytes) -> None:
        mask = os.urandom(4)
        header = bytearray([0x80 | opcode])
        length = len(payload)
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.append(0x80 | 126)
            header += struct.pack("!H", length)
        else:
            header.append(0x80 | 127)
            header += struct.pack("!Q", length)
        header += mask
        header += bytes(payload[i] ^ mask[i % 4] for i in range(length))
        sock.sendall(header)

    @staticmethod
    def _put_drop_oldest(client: queue.Queue[bytes], data: bytes) -> None:
        try:
            client.put_nowait(data)
            return
        except queue.Full:
            pass
        try:
            client.get_nowait()
        except queue.Empty:
            pass
        try:
            client.put_nowait(data)
        except queue.Full:
            pass


class TunnelViewer:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.source = LatestJpegSource(fps=args.fps, jpeg_quality=args.jpeg_quality)
        self.video_profile = args.video_profile
        self.audio: HeadLocalPcmSource | None = None
        if args.audio_source == "head-local":
            self.audio = HeadLocalPcmSource(
                host=args.head_audio_host,
                port=args.head_audio_port,
                stream_type=args.head_audio_type,
                sample_rate=args.audio_sample_rate,
                channels=args.audio_channels,
                playback_sample_rate=args.playback_sample_rate,
            )
        self.clients = 0
        self.sent_frames = 0
        self.sent_bytes = 0
        self.app = web.Application()
        self.app.add_routes(
            [
                web.get("/", self.index),
                web.get("/status", self.status),
                web.get("/frames", self.frames),
                web.get("/audio.pcm", self.audio_pcm),
                web.post("/video-profile", self.video_profile_route),
            ]
        )

    async def index(self, _request: web.Request) -> web.Response:
        html = (
            HTML.replace("__AUDIO_ENABLED__", "true" if self.audio is not None else "false")
            .replace("__AUDIO_PLAYBACK_RATE__", str(self.args.playback_sample_rate))
            .replace("__AUDIO_PLAYBACK_CHANNELS__", "1")
        )
        return web.Response(text=html, content_type="text/html", headers={"Cache-Control": "no-store"})

    async def status(self, _request: web.Request) -> web.Response:
        video_settings = self.source.settings()
        return web.json_response(
            {
                "camera": asdict(self.source.stats()),
                "server": {
                    "fps": video_settings["fps"],
                    "jpeg_quality": video_settings["jpeg_quality"],
                    "video_profile": self.video_profile,
                    "video_profiles": VIDEO_PROFILES,
                    "clients": self.clients,
                    "sent_frames": self.sent_frames,
                    "sent_bytes": self.sent_bytes,
                },
                "audio": self.audio.status()
                if self.audio is not None
                else {
                    "ok": False,
                    "info": "",
                    "error": "audio disabled",
                    "packets": 0,
                    "bytes": 0,
                    "clients": 0,
                },
            }
        )

    async def video_profile_route(self, request: web.Request) -> web.Response:
        try:
            payload = await request.json()
        except json.JSONDecodeError:
            return web.json_response({"ok": False, "error": "invalid JSON"}, status=400)
        profile_name = str(payload.get("profile", ""))
        profile = VIDEO_PROFILES.get(profile_name)
        if profile is None:
            return web.json_response({"ok": False, "error": f"unknown profile: {profile_name}"}, status=400)
        self.source.set_settings(fps=int(profile["fps"]), jpeg_quality=int(profile["jpeg_quality"]))
        self.video_profile = profile_name
        return await self.status(request)

    async def frames(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=10.0, compress=False)
        await ws.prepare(request)
        self.clients += 1
        last_sent = 0
        try:
            while not ws.closed:
                video_settings = self.source.settings()
                period = 1.0 / max(1, int(video_settings["fps"]))
                latest = self.source.latest()
                if latest is None:
                    await asyncio.sleep(0.05)
                    continue
                frame_no, wall_time, _width, _height, jpeg = latest
                if frame_no != last_sent:
                    header = struct.pack(">QI", int(wall_time * 1000), int(frame_no))
                    await ws.send_bytes(header + jpeg)
                    self.sent_frames += 1
                    self.sent_bytes += len(jpeg)
                    last_sent = frame_no
                await asyncio.sleep(period)
        except (ConnectionResetError, ClientConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            self.clients = max(0, self.clients - 1)
        return ws

    async def audio_pcm(self, request: web.Request) -> web.StreamResponse:
        if self.audio is None:
            return web.Response(status=404, text="audio disabled")

        client = self.audio.subscribe()
        chunk_seconds = 0.08
        chunk_size = (
            int(self.audio.playback_sample_rate * chunk_seconds)
            * self.audio.playback_channels
            * self.audio.bytes_per_sample
        )
        silence = self.audio.playback_silence(chunk_seconds)
        pending = bytearray()
        next_emit = time.monotonic()
        response = web.StreamResponse(
            headers={
                "Cache-Control": "no-store",
                "X-Accel-Buffering": "no",
            }
        )
        response.content_type = "application/octet-stream"
        await response.prepare(request)
        try:
            while self.audio.running:
                collect_deadline = time.monotonic() + chunk_seconds
                while len(pending) < chunk_size and time.monotonic() < collect_deadline:
                    timeout = max(0.001, min(0.02, collect_deadline - time.monotonic()))
                    try:
                        data = await asyncio.to_thread(client.get, True, timeout)
                    except queue.Empty:
                        continue
                    pending.extend(self.audio.convert_for_playback(data))
                if len(pending) >= chunk_size:
                    chunk = bytes(pending[:chunk_size])
                    del pending[:chunk_size]
                else:
                    chunk = bytes(pending) + silence[: chunk_size - len(pending)]
                    pending.clear()
                now = time.monotonic()
                if next_emit > now:
                    await asyncio.sleep(next_emit - now)
                next_emit = max(next_emit + chunk_seconds, time.monotonic())
                await response.write(chunk)
        except (ConnectionResetError, ClientConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            self.audio.unsubscribe(client)
        return response

    def start(self) -> None:
        self.source.start()
        if self.audio is not None:
            self.audio.start()

    def stop(self) -> None:
        self.source.stop()
        if self.audio is not None:
            self.audio.stop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5061)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--jpeg-quality", type=int, default=72)
    parser.add_argument("--video-profile", choices=tuple(VIDEO_PROFILES), default="clear")
    parser.add_argument("--audio-source", choices=("head-local", "none"), default="head-local")
    parser.add_argument("--head-audio-host", default="10.42.0.111")
    parser.add_argument("--head-audio-port", type=int, default=50002)
    parser.add_argument("--head-audio-type", default="aec.pcm")
    parser.add_argument("--audio-sample-rate", type=int, default=64000)
    parser.add_argument("--audio-channels", type=int, default=1)
    parser.add_argument("--playback-sample-rate", type=int, default=16000)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.video_profile in VIDEO_PROFILES:
        profile = VIDEO_PROFILES[args.video_profile]
        args.fps = int(profile["fps"])
        args.jpeg_quality = int(profile["jpeg_quality"])
    args.fps = int(max(1, min(30, args.fps)))
    args.jpeg_quality = int(max(30, min(95, args.jpeg_quality)))
    args.audio_sample_rate = int(max(8000, min(192000, args.audio_sample_rate)))
    args.audio_channels = int(max(1, min(8, args.audio_channels)))
    args.playback_sample_rate = int(max(8000, min(48000, args.playback_sample_rate)))
    viewer = TunnelViewer(args)

    def shutdown(_signum: int, _frame: Any) -> None:
        viewer.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    viewer.start()
    print(f"serving tunnel camera viewer on http://{args.host}:{args.port}", flush=True)
    try:
        web.run_app(viewer.app, host=args.host, port=args.port, print=None)
    finally:
        viewer.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
