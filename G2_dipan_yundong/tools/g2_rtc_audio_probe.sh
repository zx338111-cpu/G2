#!/usr/bin/env bash

# Short, no-motion probe for the robot RTC microphone path.
# It temporarily frees UDP 8088, starts the official camera_rtc process, and
# captures whether microphone PCM packets arrive on the configured RTC recorder
# port. It does not send chassis, arm, head, or end-effector commands.

SUDO_PASS="${SUDO_PASS:-1}"
PCAP_PATH="/tmp/rtc_8088_test.pcap"
TCPDUMP_LOG="/tmp/tcpdump_8088_test.log"
RTC_LOG="/tmp/camera_rtc_test.log"
VIEWER_PATTERN="[g]2_head_av_viewer.py"

viewer_pids="$(pgrep -f "${VIEWER_PATTERN}" || true)"
if [ -n "${viewer_pids}" ]; then
  echo "stopping viewer pids: ${viewer_pids}"
  kill ${viewer_pids} 2>/dev/null || true
  sleep 1
fi

printf "%s\n" "${SUDO_PASS}" | sudo -S -p "" rm -f "${PCAP_PATH}" "${TCPDUMP_LOG}" "${RTC_LOG}"

echo "starting tcpdump on xfi0.20 udp port 8088"
printf "%s\n" "${SUDO_PASS}" | sudo -S -p "" timeout --kill-after=2s 14s \
  tcpdump -i xfi0.20 -s 0 -U -w "${PCAP_PATH}" udp port 8088 \
  >"${TCPDUMP_LOG}" 2>&1 &
tcpdump_pid=$!
sleep 1

echo "starting official camera_rtc for 10s"
source /home/agi/app/env.sh >/dev/null 2>&1
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/home/agi/app/lib/trro/ffmpeg3:/home/agi/app/lib/trro"
printf "%s\n" "${SUDO_PASS}" | sudo -S -p "" env \
  LD_LIBRARY_PATH="${LD_LIBRARY_PATH}" \
  PATH="${PATH}" \
  PYTHONPATH="${PYTHONPATH:-}" \
  APP_CONF_PATH="${APP_CONF_PATH:-/home/agi/app/gdk/config/app_conf.json}" \
  timeout --kill-after=2s 10s /home/agi/app/bin/camera_rtc -e trro -n public >"${RTC_LOG}" 2>&1
rtc_status=$?
echo "camera_rtc exit status: ${rtc_status}"

sleep 1
kill "${tcpdump_pid}" 2>/dev/null || true
wait "${tcpdump_pid}" 2>/dev/null || true

echo "--- files"
ls -l "${PCAP_PATH}" "${TCPDUMP_LOG}" "${RTC_LOG}" 2>/dev/null || true

echo "--- tcpdump log"
sed -n '1,120p' "${TCPDUMP_LOG}" 2>/dev/null || true

echo "--- first packets"
printf "%s\n" "${SUDO_PASS}" | sudo -S -p "" tcpdump -nn -r "${PCAP_PATH}" -c 10 2>&1 || true

echo "--- camera_rtc log tail"
tail -120 "${RTC_LOG}" 2>/dev/null || true
