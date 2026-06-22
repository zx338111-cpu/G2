#!/usr/bin/env bash

# Try official camera_rtc engine/network combinations briefly and report whether
# any of them causes microphone PCM packets to arrive on UDP 8088.
# This is a no-motion probe. It only starts/stops camera_rtc and the web viewer.

SUDO_PASS="${SUDO_PASS:-1}"
VIEWER_PATTERN="[g]2_head_av_viewer.py"
COMBOS=(
  "trro public"
  "trro private"
  "agora public"
  "agora private"
  "agi public"
  "agi private"
)

stop_viewer() {
  local viewer_pids
  viewer_pids="$(pgrep -f "${VIEWER_PATTERN}" || true)"
  if [ -n "${viewer_pids}" ]; then
    echo "stopping viewer pids: ${viewer_pids}"
    kill ${viewer_pids} 2>/dev/null || true
    sleep 1
  fi
}

start_viewer() {
  echo "restarting viewer"
  source /home/agi/app/env.sh >/dev/null 2>&1
  cd /home/agi/app/gdk/examples/python || return 1
  nohup python3 /home/agi/app/gdk/examples/python/g2_head_av_viewer.py \
    --host 0.0.0.0 \
    --port 5055 \
    --udp-audio-host 0.0.0.0 \
    --udp-audio-port 8088 \
    --audio-sample-rate 16000 \
    --audio-channels 2 \
    >/tmp/g2_head_av_viewer_5055.log 2>&1 </dev/null &
  echo "viewer pid: $!"
}

run_combo() {
  local engine="$1"
  local network="$2"
  local tag="${engine}_${network}"
  local pcap="/tmp/rtc_8088_${tag}.pcap"
  local tcpdump_log="/tmp/tcpdump_8088_${tag}.log"
  local rtc_log="/tmp/camera_rtc_${tag}.log"

  printf "%s\n" "${SUDO_PASS}" | sudo -S -p "" rm -f "${pcap}" "${tcpdump_log}" "${rtc_log}"
  echo "=== ${engine} ${network}"

  printf "%s\n" "${SUDO_PASS}" | sudo -S -p "" timeout --kill-after=2s 10s \
    tcpdump -i xfi0.20 -s 0 -U -w "${pcap}" udp port 8088 \
    >"${tcpdump_log}" 2>&1 &
  local tcpdump_pid=$!
  sleep 1

  printf "%s\n" "${SUDO_PASS}" | sudo -S -p "" bash -c \
    "source /home/agi/app/env.sh >/dev/null 2>&1; \
     export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH}:/home/agi/app/lib/trro/ffmpeg3:/home/agi/app/lib/trro\"; \
     timeout --kill-after=2s 6s /home/agi/app/bin/camera_rtc -e '${engine}' -n '${network}'" \
    >"${rtc_log}" 2>&1
  local rtc_status=$?

  sleep 1
  kill "${tcpdump_pid}" 2>/dev/null || true
  wait "${tcpdump_pid}" 2>/dev/null || true

  echo "rtc_status=${rtc_status}"
  ls -l "${pcap}" "${tcpdump_log}" "${rtc_log}" 2>/dev/null || true
  sed -n '1,20p' "${tcpdump_log}" 2>/dev/null || true
  printf "%s\n" "${SUDO_PASS}" | sudo -S -p "" tcpdump -nn -r "${pcap}" -c 3 2>&1 || true
  tail -30 "${rtc_log}" 2>/dev/null || true
}

stop_viewer
for combo in "${COMBOS[@]}"; do
  run_combo ${combo}
done
start_viewer
