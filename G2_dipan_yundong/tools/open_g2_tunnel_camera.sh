#!/usr/bin/env bash
set -euo pipefail

# Open a local SSH tunnel for the G2 tunnel-friendly camera viewer.
#
# This script is intentionally local-only. It does not start robot motion and it
# does not modify the robot. It only waits for SSH and then forwards:
#
#   local  127.0.0.1:${LOCAL_PORT}
#   remote 127.0.0.1:${REMOTE_PORT}
#
# When the tunnel is active, open:
#
#   http://127.0.0.1:${LOCAL_PORT}/
#
# The remote viewer should already be running as:
#
#   source /home/agi/app/env.sh
#   cd /home/agi/app/gdk/examples/python
#   python3 ./g2_head_tunnel_viewer.py --host 127.0.0.1 --port 5061 --fps 20 --jpeg-quality 72 \
#     --video-profile clear \
#     --head-audio-type aec.pcm --audio-sample-rate 64000 --audio-channels 1 \
#     --playback-sample-rate 16000

ROBOT_HOST="${ROBOT_HOST:-10.185.207.191}"
ROBOT_USER="${ROBOT_USER:-agi}"
LOCAL_PORT="${LOCAL_PORT:-15061}"
REMOTE_PORT="${REMOTE_PORT:-5061}"
CONNECT_TIMEOUT="${CONNECT_TIMEOUT:-4}"
RETRY_DELAY_S="${RETRY_DELAY_S:-3}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-0}"  # 0 means retry forever.
URL="http://127.0.0.1:${LOCAL_PORT}/"
STATUS_URL="${URL}status"

attempt=1
while true; do
  echo "[$(date -Is)] checking ${ROBOT_USER}@${ROBOT_HOST} attempt=${attempt}"
  if command -v curl >/dev/null 2>&1 && curl -fsS --max-time 1 "${STATUS_URL}" >/dev/null 2>&1; then
    echo "[$(date -Is)] tunnel already ready: ${URL}"
    exit 0
  fi

  echo "[$(date -Is)] opening tunnel; keep this terminal open, then browse: ${URL}"
  if ssh \
    -N \
    -L "${LOCAL_PORT}:127.0.0.1:${REMOTE_PORT}" \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=10 \
    -o ServerAliveCountMax=3 \
    -o ConnectTimeout="${CONNECT_TIMEOUT}" \
    "${ROBOT_USER}@${ROBOT_HOST}"; then
    exit 0
  fi

  if [[ "${MAX_ATTEMPTS}" != "0" && "${attempt}" -ge "${MAX_ATTEMPTS}" ]]; then
    echo "ERROR: failed to open tunnel after ${attempt} attempts" >&2
    exit 1
  fi

  attempt=$((attempt + 1))
  sleep "${RETRY_DELAY_S}"
done
