#!/usr/bin/env bash
set -euo pipefail

# Read-only discovery helper for a live Agibot G2.
# It does not send motion commands and does not modify files on the robot.
#
# Defaults match the current robot host/user in this session. Pass the robot
# password through the environment when password-based SSH is required:
#   ROBOT_PASS='<robot-password>' ./tools/g2_robot_discovery.sh
#
# Optional:
#   HEAD_IP=10.42.0.111 ./tools/g2_robot_discovery.sh

ROBOT_HOST="${ROBOT_HOST:-10.185.207.191}"
ROBOT_USER="${ROBOT_USER:-agi}"
HEAD_IP="${HEAD_IP:-10.42.0.111}"

if [[ -z "${ROBOT_PASS:-}" ]]; then
  echo "ERROR: set ROBOT_PASS in the environment before running this script." >&2
  exit 1
fi

SSH_OPTS=(
  -o PubkeyAuthentication=no
  -o PreferredAuthentications=password
  -o NumberOfPasswordPrompts=1
  -o StrictHostKeyChecking=no
  -o ConnectTimeout=5
)

if ! command -v sshpass >/dev/null 2>&1; then
  echo "ERROR: sshpass is required when ROBOT_PASS is set." >&2
  exit 1
fi

sshpass -p "$ROBOT_PASS" ssh "${SSH_OPTS[@]}" "$ROBOT_USER@$ROBOT_HOST" \
  "HEAD_IP='$HEAD_IP' bash -s" <<'REMOTE'
set -euo pipefail

HEAD_IP="${HEAD_IP:-10.42.0.111}"

echo "## Host"
date -Is
hostname || true
uname -a || true

echo
echo "## Network"
ip -br addr || true
echo
echo "route to ${HEAD_IP}:"
ip route get "$HEAD_IP" 2>&1 || true
echo
echo "neighbor ${HEAD_IP}:"
ip neigh show "$HEAD_IP" || true

echo
echo "## Aorta/etcd"
curl -sS --max-time 3 http://127.0.0.1:2379/version || true
echo

echo
echo "## Launcher Scene / RTC"
echo "run.conf DEFAULT_LAUNCH_SCENE:"
grep -E '^export DEFAULT_LAUNCH_SCENE=' /home/agi/app/conf/sys/run.conf 2>/dev/null || true
echo "saved scene:"
cat /data/launcher/scene 2>/dev/null || true
echo
echo "camera_rtc process:"
pgrep -af 'camera_rtc' || true
echo
echo "RTC/audio listening sockets:"
ss -tunlp 2>/dev/null | grep -E '8088|8089|camera_rtc|rtc' || true
echo
echo "camera_rtc manifest entries:"
grep -nH 'camera_rtc' /home/agi/app/conf/manifest.d/*.json 2>/dev/null || true

echo
echo "RTC config summary:"
python3 - <<'PY'
import json
from pathlib import Path

paths = [
    Path("/home/agi/app/config/gx_rtc_config.json"),
    Path("/home/agi/app/config/gx_rtc_trro_config_public.json"),
    Path("/home/agi/app/config/gx_rtc_trro_config_private.json"),
    Path("/home/agi/app/config/gx_rtc_trro_public_multi.json"),
    Path("/home/agi/app/config/gx_rtc_trro_private_multi.json"),
]


def redact(value):
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            lowered = str(key).lower()
            if any(word in lowered for word in ("password", "passwd", "token", "secret", "key")):
                out[key] = "<redacted>"
            else:
                out[key] = redact(item)
        return out
    if isinstance(value, list):
        return [redact(item) for item in value]
    return value


for path in paths:
    print(f"-- {path}")
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        print("missing")
        continue
    except Exception as exc:
        print(f"cannot parse: {type(exc).__name__}: {exc}")
        continue

    if path.name == "gx_rtc_config.json":
        keys = [
            "mode",
            "audio_player",
            "audio_recoder",
            "agora_public",
            "agora_private",
            "trro_public",
            "trro_private",
        ]
        summary = {key: data.get(key) for key in keys if key in data}
        for section in ("agora_public", "agora_private"):
            url = (data.get(section) or {}).get("token_server", "")
            if not url:
                continue
            try:
                from urllib.parse import urlparse

                parsed = urlparse(url)
                port = parsed.port or (443 if parsed.scheme == "https" else 80)
                summary.setdefault(section, {})["token_server_endpoint"] = (
                    f"{parsed.scheme}://{parsed.hostname}:{port}"
                )
            except Exception:
                summary.setdefault(section, {})["token_server_endpoint"] = "<parse-error>"
    else:
        keys = [
            "device_id",
            "device_name",
            "cloud_mode",
            "sdk_mode",
            "server_ip",
            "server_port",
            "mqtt_server",
            "mqtt_port",
            "projectid",
            "cert_file",
            "audio_enable",
            "audio_external",
            "audio_receive",
        ]
        summary = {key: data.get(key) for key in keys if key in data}
        if "streams_config" in data:
            summary["streams_config_count"] = len(data.get("streams_config") or [])

    print(json.dumps(redact(summary), ensure_ascii=False, indent=2, sort_keys=True))
PY

echo
echo "RTC signaling reachability:"
for host in 10.111.102.18 124.223.148.178 124.223.149.217 ap.1441665.agora.local; do
  echo "-- ${host}"
  getent hosts "$host" 2>/dev/null || true
  resolved_host="$(getent ahostsv4 "$host" 2>/dev/null | awk 'NR == 1 {print $1}')"
  if [ -n "$resolved_host" ]; then
    ip route get "$resolved_host" 2>&1 | head -3 || true
  else
    echo "dns unresolved"
  fi
done
for target in \
  10.111.102.18:18010 \
  10.111.102.18:2883 \
  10.111.102.18:3000 \
  124.223.148.178:1883 \
  124.223.149.217:1883; do
  host="${target%:*}"
  port="${target#*:}"
  if timeout 2 bash -c "</dev/tcp/${host}/${port}" >/dev/null 2>&1; then
    echo "tcp open ${target}"
  else
    echo "tcp blocked ${target}"
  fi
done

echo
echo "Recent RTC failure signatures:"
if [ -d /data/logs/rtc ]; then
  find /data/logs/rtc -maxdepth 3 -type f 2>/dev/null \
    | xargs -r grep -IhE 'device_id or password|initGwPath|Calling connect failure|connect out time|token|UdpRecoder|Failed to bind socket' 2>/dev/null \
    | sed -E 's/([Pp]assword|[Pp]asswd|[Tt]oken|[Ss]ecret|[Kk]ey)[=:][^, ]*/\1=<redacted>/g' \
    | tail -80 || true
else
  echo "missing /data/logs/rtc"
fi

echo
python3 - <<'PY'
import base64
import collections
import json
import urllib.request

body = b'{"key":"AA==","range_end":"AA=="}'
req = urllib.request.Request(
    "http://127.0.0.1:2379/v3/kv/range",
    data=body,
    headers={"Content-Type": "application/json"},
)

try:
    obj = json.loads(urllib.request.urlopen(req, timeout=3).read())
except Exception as exc:
    print("ERROR: cannot query etcd v3:", type(exc).__name__, exc)
    raise SystemExit(0)

records = []
for kv in obj.get("kvs", []):
    key = base64.b64decode(kv.get("key", "")).decode("utf-8", "replace")
    val = base64.b64decode(kv.get("value", "")).decode("utf-8", "replace")
    try:
        data = json.loads(val)
    except Exception:
        continue
    cat = key.split("/")[1] if key.startswith("/") and len(key.split("/")) > 1 else key
    records.append((cat, key, data))

nodes = [d for cat, _, d in records if cat == "nodes"]
print("## Nodes")
print("count", len(nodes))
for d in sorted(nodes, key=lambda x: (str(x.get("host_id")), str(x.get("exe_name")))):
    print(
        "\t".join(
            [
                str(d.get("host_id", "")),
                str(d.get("host_name", "")),
                str(d.get("ip", "")),
                str(d.get("exe_name", "")),
                "pid=" + str(d.get("pid", "")),
                "node=" + str(d.get("node_name", "")),
            ]
        )
    )

print()
print("## Head / agibot_voice endpoints")
seen = set()
for cat, _, data in records:
    if data.get("exe_name") != "agibot_voice":
        continue
    if cat == "nodes":
        print(
            "NODE",
            "host_id=" + str(data.get("host_id")),
            "host_name=" + str(data.get("host_name")),
            "ip=" + str(data.get("ip")),
            "pid=" + str(data.get("pid")),
        )
        continue
    topic = data.get("topic")
    if not topic:
        continue
    item = (cat, topic, data.get("type", ""), data.get("port", ""), data.get("sock_name", ""))
    if item in seen:
        continue
    seen.add(item)
    print(
        "\t".join(
            [
                cat,
                str(topic),
                "port=" + str(data.get("port", "")),
                "type=" + str(data.get("type", "")),
                "sock=" + str(data.get("sock_name", "")),
            ]
        )
    )

print()
print("## Topic counts by process")
counter = collections.Counter()
examples = collections.defaultdict(list)
for cat, _, data in records:
    topic = data.get("topic")
    exe = data.get("exe_name")
    host = data.get("host_id")
    if topic and exe:
        key = (str(host), str(exe), cat)
        counter[key] += 1
        if len(examples[key]) < 5:
            examples[key].append(str(topic))

for (host, exe, cat), count in sorted(counter.items(), key=lambda x: (-x[1], x[0][0], x[0][1]))[:80]:
    print("\t".join([host, exe, cat, str(count), ", ".join(examples[(host, exe, cat)])]))
PY

echo
echo "## Head SSH probe"
timeout 3 bash -c "cat < /dev/tcp/${HEAD_IP}/22" 2>/dev/null | head -1 || true
ssh -vvv -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=5 "agi@${HEAD_IP}" true 2>&1 \
  | grep -E "Remote protocol|Authentications that can continue|Offering public key|Server accepts key|Permission denied|Next authentication" \
  | head -120 || true

echo
echo "## Head AISpeech localSocket probe"
HEAD_IP="$HEAD_IP" python3 - <<'PY'
import base64
import os
import socket

host = os.environ.get("HEAD_IP", "10.42.0.111")
port = 50002
key = base64.b64encode(os.urandom(16)).decode("ascii")
request = (
    "GET / HTTP/1.1\r\n"
    f"Host: {host}:{port}\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    f"Sec-WebSocket-Key: {key}\r\n"
    "Sec-WebSocket-Version: 13\r\n"
    "\r\n"
).encode("ascii")
try:
    sock = socket.create_connection((host, port), timeout=2)
    sock.settimeout(2)
    sock.sendall(request)
    response = b""
    while b"\r\n" not in response:
        chunk = sock.recv(256)
        if not chunk:
            break
        response += chunk
    first_line = response.split(b"\r\n", 1)[0].decode("latin1", "replace")
    print(f"{host}:{port} {first_line}")
    sock.close()
except Exception as exc:
    print(f"{host}:{port} {type(exc).__name__}: {exc}")
PY

echo
echo "## Head TCP reachability"
voice_ports="$(
python3 - <<'PY'
import base64
import json
import urllib.request

body = b'{"key":"AA==","range_end":"AA=="}'
req = urllib.request.Request(
    "http://127.0.0.1:2379/v3/kv/range",
    data=body,
    headers={"Content-Type": "application/json"},
)
try:
    obj = json.loads(urllib.request.urlopen(req, timeout=3).read())
except Exception:
    print("")
    raise SystemExit(0)

ports = set()
for kv in obj.get("kvs", []):
    val = base64.b64decode(kv.get("value", "")).decode("utf-8", "replace")
    if "agibot_voice" not in val:
        continue
    try:
        data = json.loads(val)
    except Exception:
        continue
    port = data.get("port")
    if isinstance(port, int) and port > 0:
        ports.add(port)
print(" ".join(str(p) for p in sorted(ports)))
PY
)"

for port in 22 50002 $voice_ports; do
  if timeout 1 bash -c "</dev/tcp/${HEAD_IP}/${port}" >/dev/null 2>&1; then
    echo "tcp open ${port}"
  else
    echo "tcp closed ${port}"
  fi
done
REMOTE
