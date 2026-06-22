#!/usr/bin/env bash
set -u

HOST="${1:-agi@10.20.15.152}"
OUT_DIR="${2:-/tmp/g2_snapshot_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$OUT_DIR"

run_remote() {
  local name="$1"
  local cmd="$2"
  echo "[g2-snapshot] $name"
  ssh -o StrictHostKeyChecking=no "$HOST" "$cmd" >"$OUT_DIR/$name.txt" 2>&1 || true
}

run_remote "00_identity" 'date; hostname; uname -a; cat /home/agi/app/.version 2>/dev/null; head -80 /home/agi/app/.pkg_version 2>/dev/null'
run_remote "01_systemd" 'systemctl status genie_app.service --no-pager -l; systemctl is-active genie_app.service agibot_perfguard.service ethercat.service rhino_ptp4l_domain0.service rhino_mcu_daemon.service chrony.service edge-client.service'
run_remote "02_processes" 'ps -e -o pid,ppid,nlwp,stat,pcpu,pmem,comm,args --sort=-nlwp | head -100; pgrep -a -f "launcher|aorta|fastdds|gdk|hal|motion|quark|slam|dr|fault|camera|lidar|task|mcu"'
run_remote "03_network" 'ip -br addr; ip route; ss -lntup | grep -E "2379|2380|8849|11811" || true'
run_remote "04_startup_config" 'sed -n "1,90p" /home/agi/app/conf/sys/run.conf; grep --color=never -nE "LOCATOR|AORTA|fastdds|aorta|launcher|DEFAULT_LAUNCH_SCENE|COSINE_BUS" /home/agi/app/bin/run.sh; sed -n "1,240p" /home/agi/app/conf/manifest.d/base.json'
run_remote "05_logs_index" 'ls -l /data/logs/latest; find /data/logs/latest/ -maxdepth 2 -type f | sort | sed -n "1,240p"'
run_remote "06_recent_errors" 'grep -R "ERROR\|WARN\|Failed\|fault\|DataLoss\|type mismatch\|Relocalization Is Failed" /data/logs/latest -n 2>/dev/null | tail -300'
run_remote "07_hal_hardware" 'grep --color=never -nE "producer|tianji|juxie|omnipicker|create .*success|motor .*err|All motors|fault|idx21|idx61" /data/logs/latest/hal*.INFO* 2>/dev/null | tail -260'
run_remote "08_lowerlimb" 'grep --color=never -nE "ethercat|ecat|slave|FourWheel|power board|chassis|OPERATIONAL|fault|stop move" /data/logs/latest/hal_lowerlimb*.INFO* 2>/dev/null | tail -260'
run_remote "09_lidar_slam_pnc" 'grep --color=never -nE "Livox|10\.42|/lidar|/imu|global_loc|Relocalization|gicp|score|/dr/odom|/tf|task_state|chassis_joint_cmd" /data/logs/latest/lidar*.INFO* /data/logs/latest/*slam* /data/logs/latest/quark_navigation*.INFO* 2>/dev/null | tail -360'
run_remote "10_buses" 'sudo -n ethercat master; sudo -n ethercat slaves; ip -details link show can0; ip -details link show can1; ls -l /dev/EtherCAT* /dev/ptp* /dev/spidev2.0 2>/dev/null'
run_remote "11_parameters" 'find /data/parameters -maxdepth 3 -type f | sort; sed -n "1,220p" /data/parameters/hardware/arm_parameters.yaml 2>/dev/null; sed -n "1,180p" /data/parameters/system_tool_config.yaml 2>/dev/null'
run_remote "12_topics" 'grep -RhoE "topic:?/[A-Za-z0-9_./-]+" /data/logs/latest/ 2>/dev/null | sed "s/^topic://" | sort -u'

{
  echo "# G2 Quick Snapshot"
  echo
  echo "host: $HOST"
  echo "created: $(date)"
  echo "output: $OUT_DIR"
  echo
  echo "Files:"
  find "$OUT_DIR" -maxdepth 1 -type f | sort | sed "s#^#- #"
} >"$OUT_DIR/README.md"

echo "[g2-snapshot] wrote $OUT_DIR"
