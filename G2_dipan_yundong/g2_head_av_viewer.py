#!/usr/bin/env python3
"""Large head-camera web view with robot microphone audio and ASR.

This program is intentionally read-only for robot motion: it only reads the
head RGB camera and enables the robot interaction stack so microphone input can
be transcribed through the official ASR callback.  It can also expose raw PCM
audio if the robot-side RTC/voice stack sends microphone frames to the configured
UDP port.
"""

from __future__ import annotations

import argparse
from array import array
import base64
from collections import deque
import queue
import socket
import struct
from dataclasses import dataclass, asdict
import json
import os
import signal
import sys
import threading
import time
from typing import Any

import agibot_gdk

try:
    import cv2
    import numpy as np
except ImportError as exc:  # pragma: no cover - this runs on the robot.
    raise SystemExit(f"OpenCV/numpy is required for the web viewer: {exc}") from exc

try:
    from flask import Flask, Response, jsonify, render_template_string
except ImportError as exc:  # pragma: no cover - this runs on the robot.
    raise SystemExit(f"Flask is required for the web viewer: {exc}") from exc


HTML_TEMPLATE = """
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>G2 Head Camera + ASR</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #111418;
      --panel: #181d23;
      --line: #2b333d;
      --text: #edf2f7;
      --muted: #a7b0bd;
      --ok: #52d273;
      --warn: #f5bf4f;
      --bad: #ff6b6b;
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
      grid-template-columns: minmax(0, 1fr) 360px;
      gap: 12px;
      min-height: 100vh;
      padding: 12px;
    }
    .camera {
      min-width: 0;
      min-height: calc(100vh - 24px);
      border: 1px solid var(--line);
      background: #050607;
      border-radius: 8px;
      overflow: hidden;
      position: relative;
    }
    .camera img {
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
      border-radius: 6px;
      background: rgba(10, 12, 15, 0.78);
      border: 1px solid rgba(255, 255, 255, 0.14);
      font-size: 14px;
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 999px;
      background: var(--warn);
    }
    .dot.ok { background: var(--ok); }
    .dot.bad { background: var(--bad); }
    .side {
      min-height: calc(100vh - 24px);
      display: grid;
      grid-template-rows: auto auto auto minmax(0, 1fr);
      gap: 12px;
    }
    .panel {
      border: 1px solid var(--line);
      background: var(--panel);
      border-radius: 8px;
      padding: 14px;
      min-width: 0;
    }
    h1, h2 {
      margin: 0;
      letter-spacing: 0;
      font-weight: 700;
    }
    h1 { font-size: 18px; }
    h2 { font-size: 15px; color: var(--muted); margin-bottom: 10px; }
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
    .asr-list {
      height: 100%;
      overflow-y: auto;
      padding-right: 4px;
    }
    .asr-item {
      padding: 10px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.08);
    }
    .asr-time {
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 4px;
    }
    .asr-text {
      font-size: 17px;
      line-height: 1.45;
      overflow-wrap: anywhere;
    }
    .empty {
      color: var(--muted);
      font-size: 14px;
      line-height: 1.5;
    }
    .audio-control {
      display: grid;
      gap: 10px;
    }
    button {
      min-height: 38px;
      border: 1px solid #3b8f5a;
      border-radius: 6px;
      background: #1f7a43;
      color: var(--text);
      font-size: 14px;
      font-weight: 700;
      cursor: pointer;
    }
    button:hover { background: #248d4e; }
    audio {
      width: 100%;
      height: 36px;
    }
    @media (max-width: 900px) {
      .shell {
        grid-template-columns: 1fr;
        grid-template-rows: 64vh auto;
      }
      .camera, .camera img {
        min-height: 64vh;
      }
      .side {
        min-height: 0;
        grid-template-rows: auto auto auto 280px;
      }
    }
  </style>
</head>
<body>
  <main class="shell">
    <section class="camera">
      <img id="head-frame" src="/stream.mjpg" alt="head camera">
      <div class="hud">
        <span id="camera-dot" class="dot"></span>
        <span id="camera-status">head_color</span>
      </div>
    </section>
    <aside class="side">
      <section class="panel">
        <h1>G2 Head AV</h1>
      </section>
      <section class="panel">
        <h2>状态</h2>
        <div class="metric"><div class="label">相机</div><div id="camera-value" class="value">连接中</div></div>
        <div class="metric"><div class="label">麦克风</div><div id="mic-value" class="value">连接中</div></div>
        <div class="metric"><div class="label">实时声音</div><div id="pcm-value" class="value">等待中</div></div>
        <div class="metric"><div class="label">ASR</div><div id="asr-value" class="value">等待中</div></div>
      </section>
      <section class="panel audio-control">
        <h2>声音</h2>
        <button id="start-audio" type="button">开启实时声音</button>
        <audio id="live-audio" controls></audio>
      </section>
      <section class="panel">
        <h2>识别文本</h2>
        <div id="asr-list" class="asr-list">
          <div class="empty">暂无识别结果</div>
        </div>
      </section>
    </aside>
  </main>
  <script>
    const asrList = document.getElementById("asr-list");
    const asrValue = document.getElementById("asr-value");
    const micValue = document.getElementById("mic-value");
    const pcmValue = document.getElementById("pcm-value");
    const cameraValue = document.getElementById("camera-value");
    const cameraStatus = document.getElementById("camera-status");
    const cameraDot = document.getElementById("camera-dot");
    const startAudio = document.getElementById("start-audio");
    const liveAudio = document.getElementById("live-audio");
    const SOURCE_RATE = {{ audio_sample_rate|int }};
    const SOURCE_CHANNELS = {{ audio_channels|int }};
    const BYTES_PER_SAMPLE = 2;
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
    let underruns = 0;
    let receivedBytes = 0;

    function setDot(ok) {
      cameraDot.classList.remove("ok", "bad");
      cameraDot.classList.add(ok ? "ok" : "bad");
    }

    function renderAsr(items) {
      if (!items.length) {
        asrList.innerHTML = '<div class="empty">暂无识别结果</div>';
        return;
      }
      asrList.innerHTML = "";
      items.slice().reverse().forEach((item) => {
        const row = document.createElement("div");
        row.className = "asr-item";
        const stamp = new Date(item.wall_time * 1000).toLocaleTimeString();
        row.innerHTML = `<div class="asr-time">${stamp}</div><div class="asr-text"></div>`;
        row.querySelector(".asr-text").textContent = item.text;
        asrList.appendChild(row);
      });
    }

    async function refreshStatus() {
      try {
        const response = await fetch("/status", {cache: "no-store"});
        const data = await response.json();
        cameraValue.textContent = data.camera.ok ? data.camera.info : data.camera.error;
        micValue.textContent = data.audio.ok ? data.audio.info : data.audio.error;
        pcmValue.textContent = data.pcm.ok ? data.pcm.info : data.pcm.error;
        asrValue.textContent = data.asr.count ? `${data.asr.count} 条` : "等待中";
        cameraStatus.textContent = data.camera.ok ? "head_color online" : "head_color offline";
        setDot(data.camera.ok);
        renderAsr(data.asr.items || []);
      } catch (error) {
        cameraValue.textContent = String(error);
        micValue.textContent = "连接中断";
        pcmValue.textContent = "连接中断";
        asrValue.textContent = "连接中断";
        cameraStatus.textContent = "offline";
        setDot(false);
      }
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
      receivedBytes += frameCount * frameBytes;
      trimAudioQueue();
    }

    function trimAudioQueue() {
      const maxSamples = Math.floor(SOURCE_RATE * 1.2);
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
          underruns += 1;
          return 0;
        }
      }
      const value = activeChunk[activeOffset];
      activeOffset += 1;
      sourceQueued -= 1;
      return value;
    }

    function fillOutput(event) {
      const output = event.outputBuffer.getChannelData(0);
      const startThreshold = Math.floor(SOURCE_RATE * 0.28);
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
      for (let channel = 1; channel < event.outputBuffer.numberOfChannels; channel += 1) {
        event.outputBuffer.copyToChannel(output, channel);
      }
      startAudio.textContent = `实时声音 ${Math.round(sourceQueued / SOURCE_RATE * 1000)}ms`;
    }

    async function readPcmStream(signal) {
      const response = await fetch(`/audio.pcm?ts=${Date.now()}`, {cache: "no-store", signal});
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
    }

    async function startWebAudio() {
      if (audioAbort) {
        audioAbort.abort();
      }
      sourceChunks = [];
      sourceQueued = 0;
      activeChunk = null;
      activeOffset = 0;
      resamplePhase = 0;
      lastSample = 0;
      audioStarted = false;
      underruns = 0;
      receivedBytes = 0;
      audioAbort = new AbortController();
      const AudioContextClass = window.AudioContext || window.webkitAudioContext;
      audioContext = new AudioContextClass({latencyHint: "interactive"});
      await audioContext.resume();
      audioNode = audioContext.createScriptProcessor(4096, 0, 1);
      audioNode.onaudioprocess = fillOutput;
      audioNode.connect(audioContext.destination);
      startAudio.textContent = "实时声音缓冲中";
      readPcmStream(audioAbort.signal).catch((error) => {
        if (error.name !== "AbortError") {
          startAudio.textContent = `声音中断: ${error}`;
        }
      });
    }

    startAudio.addEventListener("click", async () => {
      if (audioAbort) {
        audioAbort.abort();
        audioAbort = null;
      }
      if (audioNode) {
        audioNode.disconnect();
        audioNode = null;
      }
      if (audioContext) {
        audioContext.close();
        audioContext = null;
      }
      sourceChunks = [];
      sourceQueued = 0;
      liveAudio.src = `/audio.wav?ts=${Date.now()}`;
      startAudio.textContent = "实时声音已打开";
      try {
        await liveAudio.play();
      } catch (error) {
        startAudio.textContent = "点击后再试一次";
      }
    });

    refreshStatus();
    setInterval(refreshStatus, 800);
  </script>
</body>
</html>
"""


@dataclass
class AsrEvent:
    text: str
    wall_time: float


class PcmUdpBridge:
    """Fan out UDP PCM microphone packets to browser WAV streams."""

    def __init__(self, host: str, port: int, sample_rate: int, channels: int):
        self.host = host
        self.port = port
        self.sample_rate = sample_rate
        self.channels = channels
        self.bytes_per_sample = 2
        self.running = False
        self.error = ""
        self.packet_count = 0
        self.byte_count = 0
        self.last_packet_time = 0.0
        self.peer = ""
        self.thread: threading.Thread | None = None
        self.sock: socket.socket | None = None
        self.clients: set[queue.Queue[bytes]] = set()
        self.lock = threading.Lock()

    def start(self) -> None:
        if self.thread is not None:
            return
        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, name="pcm-udp-bridge", daemon=True)
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

    def _recv_loop(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.settimeout(0.5)
            self.sock = sock
            self.error = ""
            print(f"listening for PCM UDP on {self.host}:{self.port}", flush=True)
        except OSError as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.running = False
            print(f"PCM UDP bind failed: {self.error}", flush=True)
            return

        while self.running:
            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError as exc:
                if self.running:
                    self.error = f"{type(exc).__name__}: {exc}"
                break
            if not data:
                continue
            now = time.time()
            with self.lock:
                self.packet_count += 1
                self.byte_count += len(data)
                self.last_packet_time = now
                self.peer = f"{addr[0]}:{addr[1]}"
                clients = list(self.clients)
            for client in clients:
                self._put_drop_oldest(client, data)

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

    def wav_header(self) -> bytes:
        block_align = self.channels * self.bytes_per_sample
        byte_rate = self.sample_rate * block_align
        data_size = 0x7FFFFFF0
        riff_size = data_size + 36
        return struct.pack(
            "<4sI4s4sIHHIIHH4sI",
            b"RIFF",
            riff_size,
            b"WAVE",
            b"fmt ",
            16,
            1,
            self.channels,
            self.sample_rate,
            byte_rate,
            block_align,
            self.bytes_per_sample * 8,
            b"data",
            data_size,
        )

    def silence(self, seconds: float = 0.1) -> bytes:
        frame_count = max(1, int(self.sample_rate * seconds))
        return b"\x00" * frame_count * self.channels * self.bytes_per_sample

    def stream(self) -> Any:
        client: queue.Queue[bytes] = queue.Queue(maxsize=128)
        with self.lock:
            self.clients.add(client)
        try:
            yield self.wav_header()
            silence = self.silence()
            while self.running:
                try:
                    yield client.get(timeout=0.2)
                except queue.Empty:
                    yield silence
        finally:
            with self.lock:
                self.clients.discard(client)

    def raw_stream(self) -> Any:
        client: queue.Queue[bytes] = queue.Queue(maxsize=256)
        with self.lock:
            self.clients.add(client)
        try:
            silence = self.silence(0.02)
            while self.running:
                try:
                    yield client.get(timeout=0.05)
                except queue.Empty:
                    yield silence
        finally:
            with self.lock:
                self.clients.discard(client)

    def status(self) -> dict[str, Any]:
        with self.lock:
            packet_count = self.packet_count
            byte_count = self.byte_count
            last_packet_time = self.last_packet_time
            peer = self.peer
            client_count = len(self.clients)
            error = self.error
        age = time.time() - last_packet_time if last_packet_time else None
        recent = age is not None and age < 2.0
        if error:
            return {
                "ok": False,
                "info": "",
                "error": error,
                "packets": packet_count,
                "bytes": byte_count,
                "last_packet_age_s": age,
                "clients": client_count,
            }
        if not packet_count:
            return {
                "ok": False,
                "info": "",
                "error": f"listening on {self.host}:{self.port}; no PCM packets yet",
                "packets": packet_count,
                "bytes": byte_count,
                "last_packet_age_s": age,
                "clients": client_count,
            }
        return {
            "ok": recent,
            "info": (
                f"{self.sample_rate}Hz {self.channels}ch packets={packet_count} "
                f"last={age:.1f}s peer={peer} clients={client_count}"
            ),
            "error": (
                f"last PCM packet {age:.1f}s ago from {peer}"
                if age is not None
                else "no recent PCM packet"
            ),
            "packets": packet_count,
            "bytes": byte_count,
            "last_packet_age_s": age,
            "clients": client_count,
        }


class HeadLocalSocketPcmBridge:
    """Stream processed microphone PCM from the head board AISpeech websocket.

    The head board exposes a local AISpeech websocket on port 50002.  It can
    publish processed PCM frames after a `/downstream/setBinaryType` request.
    `beforming.pcm` has been observed as stable 16 kHz mono signed-16 PCM.
    """

    def __init__(
        self,
        host: str,
        port: int,
        stream_type: str,
        sample_rate: int,
        channels: int,
        playback_sample_rate: int | None = None,
    ):
        self.host = host
        self.port = port
        self.stream_type = stream_type
        self.sample_rate = sample_rate
        self.channels = channels
        self.playback_sample_rate = playback_sample_rate or sample_rate
        self.bytes_per_sample = 2
        self.running = False
        self.error = ""
        self.info = ""
        self.packet_count = 0
        self.byte_count = 0
        self.last_packet_time = 0.0
        self.last_text_event = ""
        self.reconnect_count = 0
        self.thread: threading.Thread | None = None
        self.sock: socket.socket | None = None
        self.clients: set[queue.Queue[bytes]] = set()
        self.lock = threading.Lock()

    def start(self) -> None:
        if self.thread is not None:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, name="head-local-pcm-bridge", daemon=True)
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

    def wav_header(self) -> bytes:
        block_align = self.bytes_per_sample
        byte_rate = self.playback_sample_rate * block_align
        data_size = 0x7FFFFFF0
        return struct.pack(
            "<4sI4s4sIHHIIHH4sI",
            b"RIFF",
            data_size + 36,
            b"WAVE",
            b"fmt ",
            16,
            1,
            1,
            self.playback_sample_rate,
            byte_rate,
            block_align,
            self.bytes_per_sample * 8,
            b"data",
            data_size,
        )

    def silence(self, seconds: float = 0.1) -> bytes:
        frame_count = max(1, int(self.playback_sample_rate * seconds))
        return b"\x00" * frame_count * self.bytes_per_sample

    def _convert_for_playback(self, data: bytes) -> bytes:
        frame_size = self.channels * self.bytes_per_sample
        usable = len(data) - (len(data) % frame_size)
        if usable <= 0:
            return b""
        samples = array("h")
        samples.frombytes(data[:usable])
        if self.channels > 1:
            mono = array("h")
            for idx in range(0, len(samples), self.channels):
                mono.append(int(sum(samples[idx : idx + self.channels]) / self.channels))
            samples = mono
        if self.sample_rate != self.playback_sample_rate:
            ratio = max(1, int(round(self.sample_rate / self.playback_sample_rate)))
            samples = samples[::ratio]
        return samples.tobytes()

    def stream(self) -> Any:
        client: queue.Queue[bytes] = queue.Queue(maxsize=256)
        with self.lock:
            self.clients.add(client)
        try:
            yield self.wav_header()
            chunk_seconds = 0.1
            chunk_size = int(self.playback_sample_rate * chunk_seconds) * self.bytes_per_sample
            silence = self.silence(chunk_seconds)
            pending = bytearray()
            next_emit = time.monotonic()
            while self.running:
                collect_deadline = time.monotonic() + chunk_seconds
                while len(pending) < chunk_size and time.monotonic() < collect_deadline:
                    try:
                        pending.extend(self._convert_for_playback(client.get(timeout=0.02)))
                    except queue.Empty:
                        pass
                if len(pending) >= chunk_size:
                    chunk = bytes(pending[:chunk_size])
                    del pending[:chunk_size]
                else:
                    chunk = bytes(pending) + silence[: chunk_size - len(pending)]
                    pending.clear()
                now = time.monotonic()
                if next_emit > now:
                    time.sleep(next_emit - now)
                next_emit = max(next_emit + chunk_seconds, time.monotonic())
                yield chunk
        finally:
            with self.lock:
                self.clients.discard(client)

    def raw_stream(self) -> Any:
        client: queue.Queue[bytes] = queue.Queue(maxsize=512)
        with self.lock:
            self.clients.add(client)
        try:
            silence = self.silence(0.02)
            while self.running:
                try:
                    yield client.get(timeout=0.05)
                except queue.Empty:
                    yield silence
        finally:
            with self.lock:
                self.clients.discard(client)

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
        if error and not packet_count:
            return {
                "ok": False,
                "info": info,
                "error": error,
                "packets": packet_count,
                "bytes": byte_count,
                "last_packet_age_s": age,
                "clients": client_count,
                "reconnects": reconnect_count,
                "last_text_event": last_text_event,
            }
        if not packet_count:
            return {
                "ok": False,
                "info": info,
                "error": f"connected; waiting for {self.stream_type} frames",
                "packets": packet_count,
                "bytes": byte_count,
                "last_packet_age_s": age,
                "clients": client_count,
                "reconnects": reconnect_count,
                "last_text_event": last_text_event,
            }
        return {
            "ok": recent,
            "info": (
                f"{self.sample_rate}Hz {self.channels}ch {self.stream_type} "
                f"playback={self.playback_sample_rate}Hz mono "
                f"packets={packet_count} last={age:.1f}s clients={client_count}"
            ),
            "error": f"last PCM frame {age:.1f}s ago" if age is not None else "",
            "packets": packet_count,
            "bytes": byte_count,
            "last_packet_age_s": age,
            "clients": client_count,
            "reconnects": reconnect_count,
            "last_text_event": last_text_event,
        }


class HeadAvViewer:
    def __init__(
        self,
        host: str,
        port: int,
        jpeg_quality: int,
        target_fps: float,
        pcm_bridge: PcmUdpBridge,
    ):
        self.host = host
        self.port = port
        self.jpeg_quality = jpeg_quality
        self.frame_sleep = max(0.03, 1.0 / max(target_fps, 1.0))
        self.pcm_bridge = pcm_bridge
        self.app = Flask(__name__)

        self.camera: Any | None = None
        self.interaction: Any | None = None
        self.asr_events: deque[AsrEvent] = deque(maxlen=80)
        self.last_asr_text = ""
        self.lock = threading.Lock()
        self.running = True
        self.camera_error = ""
        self.audio_error = ""
        self.last_frame_time = 0.0
        self.last_frame_info = "not ready"

        self._setup_signal_handlers()
        self.pcm_bridge.start()
        self._setup_gdk()
        self._setup_routes()

    def _setup_signal_handlers(self) -> None:
        def handle_signal(signum: int, _frame: Any) -> None:
            print(f"received signal {signum}; cleaning up", flush=True)
            self.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)

    def _setup_gdk(self) -> None:
        try:
            self.camera = agibot_gdk.Camera()
            time.sleep(1.0)
        except Exception as exc:
            self.camera_error = f"{type(exc).__name__}: {exc}"
            print(f"camera init failed: {self.camera_error}", flush=True)

        try:
            self.interaction = agibot_gdk.Interaction()
            time.sleep(1.0)
            self.interaction.set_language(agibot_gdk.Language.kLanguageChinese)
            self.interaction.set_audio_switch(True)
            self.interaction.set_call_mode(True)
            self.interaction.register_callback("get_asr_text", self._on_asr_text)
            print("robot audio switch and call mode are on", flush=True)
        except Exception as exc:
            self.audio_error = f"{type(exc).__name__}: {exc}"
            print(f"audio/asr init failed: {self.audio_error}", flush=True)

    def _setup_routes(self) -> None:
        @self.app.route("/")
        def index() -> str:
            return render_template_string(
                HTML_TEMPLATE,
                audio_sample_rate=self.pcm_bridge.sample_rate,
                audio_channels=self.pcm_bridge.channels,
            )

        @self.app.route("/status")
        def status() -> Any:
            return jsonify(self._status_payload())

        @self.app.route("/snapshot.jpg")
        def snapshot() -> Response:
            frame = self._read_head_jpeg()
            if frame is None:
                return Response(status=503)
            return Response(frame, mimetype="image/jpeg")

        @self.app.route("/stream.mjpg")
        def stream() -> Response:
            return Response(self._mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

        @self.app.route("/audio.wav")
        def audio_stream() -> Response:
            headers = {
                "Cache-Control": "no-store",
                "X-Accel-Buffering": "no",
            }
            return Response(self.pcm_bridge.stream(), mimetype="audio/wav", headers=headers)

        @self.app.route("/audio.pcm")
        def audio_pcm_stream() -> Response:
            headers = {
                "Cache-Control": "no-store",
                "X-Accel-Buffering": "no",
            }
            return Response(self.pcm_bridge.raw_stream(), mimetype="application/octet-stream", headers=headers)

    def _on_asr_text(self, text: str) -> None:
        text = self._clean_asr_text(text)
        if not text:
            return
        with self.lock:
            if text == self.last_asr_text:
                return
            self.last_asr_text = text
            self.asr_events.append(AsrEvent(text=text, wall_time=time.time()))
        print(f"ASR: {text}", flush=True)

    @staticmethod
    def _clean_asr_text(text: str) -> str:
        text = str(text or "").strip()
        if not text:
            return ""
        try:
            data = json.loads(text)
            if isinstance(data, dict) and ("status" in data or "streamDataType" in data):
                return ""
        except (TypeError, ValueError):
            pass
        return text

    def _read_head_jpeg(self) -> bytes | None:
        if self.camera is None:
            return None
        try:
            image = self.camera.get_latest_image(agibot_gdk.CameraType.kHeadColor, 1000.0)
            if image is None:
                self.camera_error = "no head_color frame"
                return None
            jpeg = self._image_to_jpeg(image)
            if jpeg is None:
                self.camera_error = "failed to encode head_color frame"
                return None
            self.camera_error = ""
            self.last_frame_time = time.time()
            self.last_frame_info = (
                f"{int(image.width)}x{int(image.height)} "
                f"{str(image.encoding).split('.')[-1]} "
                f"{str(image.color_format).split('.')[-1]}"
            )
            return jpeg
        except Exception as exc:
            self.camera_error = f"{type(exc).__name__}: {exc}"
            return None

    def _image_to_jpeg(self, image: Any) -> bytes | None:
        encoding = str(image.encoding)
        color_format = str(image.color_format)
        raw = bytes(image.data)
        if "JPEG" in encoding:
            return raw
        if "PNG" in encoding:
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
        ok, buffer = cv2.imencode(".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return None
        return buffer.tobytes()

    def _mjpeg_generator(self) -> Any:
        while self.running:
            frame = self._read_head_jpeg()
            if frame is not None:
                yield b"--frame\r\nContent-Type: image/jpeg\r\nCache-Control: no-store\r\n\r\n" + frame + b"\r\n"
            time.sleep(self.frame_sleep)

    def _status_payload(self) -> dict[str, Any]:
        with self.lock:
            items = [asdict(item) for item in self.asr_events]
            asr_count = len(self.asr_events)

        audio_ok = self.interaction is not None and not self.audio_error
        audio_info = "audio switch on; call mode on; ASR callback active"
        try:
            if self.interaction is not None:
                func_status = self.interaction.get_func_status()
                audio_info = (
                    f"audio_enabled={bool(func_status.audio_enabled)} "
                    f"wakeup={func_status.wakeup_status} "
                    f"func={func_status.func_status}"
                )
        except Exception as exc:
            audio_ok = False
            self.audio_error = f"{type(exc).__name__}: {exc}"

        frame_age = time.time() - self.last_frame_time if self.last_frame_time else None
        camera_ok = frame_age is not None and frame_age < 3.0 and not self.camera_error
        return {
            "camera": {
                "ok": camera_ok,
                "info": self.last_frame_info if camera_ok else "not ready",
                "error": "" if camera_ok else (self.camera_error or "no recent frame"),
                "frame_age_s": frame_age,
            },
            "audio": {
                "ok": audio_ok,
                "info": audio_info,
                "error": "" if audio_ok else (self.audio_error or "not ready"),
            },
            "pcm": self.pcm_bridge.status(),
            "asr": {
                "count": asr_count,
                "items": items,
            },
        }

    def cleanup(self) -> None:
        self.running = False
        self.pcm_bridge.stop()
        if self.interaction is not None:
            try:
                self.interaction.unregister_callback("get_asr_text")
            except Exception:
                pass
            try:
                self.interaction.set_call_mode(False)
            except Exception:
                pass
        if self.camera is not None:
            try:
                self.camera.close_camera()
            except Exception:
                pass

    def run(self) -> None:
        print(f"serving head AV viewer on http://{self.host}:{self.port}", flush=True)
        try:
            self.app.run(host=self.host, port=self.port, debug=False, threaded=True)
        finally:
            self.cleanup()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="G2 large head camera plus robot microphone web viewer")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5055)
    parser.add_argument("--jpeg-quality", type=int, default=82)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument(
        "--audio-source",
        choices=("head-local", "udp"),
        default="head-local",
        help="head-local subscribes to the head board AISpeech websocket; udp keeps the old RTC UDP listener.",
    )
    parser.add_argument("--head-audio-host", default="10.42.0.111")
    parser.add_argument("--head-audio-port", type=int, default=50002)
    parser.add_argument("--head-audio-type", default="beforming.pcm")
    parser.add_argument("--udp-audio-host", default="0.0.0.0")
    parser.add_argument("--udp-audio-port", type=int, default=8088)
    parser.add_argument("--audio-sample-rate", type=int, default=16000)
    parser.add_argument("--audio-channels", type=int, default=1)
    parser.add_argument("--playback-sample-rate", type=int, default=16000)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sample_rate = max(8000, min(192000, args.audio_sample_rate))
    channels = max(1, min(8, args.audio_channels))
    if args.audio_source == "head-local":
        pcm_bridge = HeadLocalSocketPcmBridge(
            host=args.head_audio_host,
            port=args.head_audio_port,
            stream_type=args.head_audio_type,
            sample_rate=sample_rate,
            channels=channels,
            playback_sample_rate=max(8000, min(48000, args.playback_sample_rate)),
        )
    else:
        pcm_bridge = PcmUdpBridge(
            host=args.udp_audio_host,
            port=args.udp_audio_port,
            sample_rate=sample_rate,
            channels=channels,
        )
    viewer = HeadAvViewer(
        host=args.host,
        port=args.port,
        jpeg_quality=max(30, min(95, args.jpeg_quality)),
        target_fps=max(1.0, min(30.0, args.fps)),
        pcm_bridge=pcm_bridge,
    )
    viewer.run()


if __name__ == "__main__":
    main()
