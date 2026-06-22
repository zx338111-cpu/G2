#!/bin/bash
# No-motion boot recovery guard for recurring G2 arm EtherCAT drive latch faults.
#
# Scope:
#   - Reads EtherCAT arm drive error registers after genie_app/HAL has had time to start.
#   - Clears latched drive-side error registers with the same no-motion sequence used manually:
#       0x3002 := 0, then EtherCAT INIT -> OP on arm CoolDrive slaves.
#   - If the HAL cycle is still dirty, restarts genie_app.service once and repeats the check.
#
# Safety boundary:
#   - This script does not call arm, waist, gripper, chassis, PNC, SLAM, or navigation motion APIs.
#   - It only manipulates EtherCAT drive state during startup recovery.
#   - It should run only as root, from boot/startup service context, not during an active mission.

set -uo pipefail

LOG=${G2_ARM_ECAT_RECOVER_LOG:-/data/logs/g2_arm_ethercat_boot_recover.log}
LOCK=${G2_ARM_ECAT_RECOVER_LOCK:-/run/g2_arm_ethercat_boot_recover.lock}
SETTLE_SECONDS=${G2_ARM_ECAT_SETTLE_SECONDS:-55}
POST_CLEAR_SETTLE_SECONDS=${G2_ARM_ECAT_POST_CLEAR_SETTLE_SECONDS:-8}
POST_RESTART_SETTLE_SECONDS=${G2_ARM_ECAT_POST_RESTART_SETTLE_SECONDS:-65}
ALLOW_RESTART=${G2_ARM_ECAT_ALLOW_RESTART:-1}
CHECK_ONLY=0

# HAL mapping on this G2 stack:
#   left arm  motor 0-6  -> EtherCAT slave 2-8
#   right arm motor 7-13 -> EtherCAT slave 10-16
ARM_SLAVES=(2 3 4 5 6 7 8 10 11 12 13 14 15 16)

for arg in "$@"; do
  case "$arg" in
    --check-only) CHECK_ONLY=1 ;;
    --no-restart) ALLOW_RESTART=0 ;;
    --no-settle) SETTLE_SECONDS=0 ;;
    *)
      echo "unknown argument: $arg" >&2
      exit 2
      ;;
  esac
done

mkdir -p "$(dirname "$LOG")" /run
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "$(date '+%Y-%m-%d %H:%M:%S %Z') another recovery instance is active; exiting" >>"$LOG"
  exit 0
fi
exec >>"$LOG" 2>&1

log() {
  echo "$(date '+%Y-%m-%d %H:%M:%S %Z') $*"
}

run_cmd() {
  log "+ $*"
  "$@"
  local rc=$?
  log "rc=$rc"
  return "$rc"
}

wait_for_genie() {
  local end=$((SECONDS + 120))
  while [ "$SECONDS" -lt "$end" ]; do
    if systemctl is-active --quiet genie_app.service; then
      return 0
    fi
    sleep 2
  done
  return 1
}

ethercat_ready() {
  ethercat slaves >/dev/null 2>&1
}

wait_for_ethercat() {
  local end=$((SECONDS + 60))
  while [ "$SECONDS" -lt "$end" ]; do
    if ethercat_ready; then
      return 0
    fi
    sleep 2
  done
  return 1
}

read_sdo_hex() {
  local slave=$1
  local index=$2
  local out hex
  out=$(ethercat upload -p "$slave" -t uint16 "$index" 0x00 2>&1 || true)
  hex=$(printf '%s\n' "$out" | grep -Eo '0x[0-9a-fA-F]+' | tail -1 || true)
  if [ -z "$hex" ]; then
    printf 'READ_ERROR:%s' "$out"
  else
    printf '%s' "${hex,,}"
  fi
}

is_zero_hex() {
  [ "$1" = "0x0000" ] || [ "$1" = "0x0" ]
}

faulted_slaves=()

scan_arm_faults() {
  local slave err603f err3002 state_line found=0
  faulted_slaves=()
  log "scan_start"
  ethercat master || true
  ethercat slaves || true

  for slave in "${ARM_SLAVES[@]}"; do
    err603f=$(read_sdo_hex "$slave" 0x603f)
    err3002=$(read_sdo_hex "$slave" 0x3002)
    state_line=$(ethercat slaves 2>/dev/null | awk -v p="$slave" '$1 == p {print $0}' || true)
    log "slave=$slave state='${state_line}' err603f=$err603f err3002=$err3002"
    if ! is_zero_hex "$err603f" || ! is_zero_hex "$err3002"; then
      faulted_slaves+=("$slave")
      found=1
    fi
  done

  if [ "$found" -eq 0 ]; then
    log "scan_result healthy"
    return 0
  fi

  log "scan_result faulted_slaves=${faulted_slaves[*]}"
  return 1
}

clear_arm_latches() {
  local slave retry
  log "clear_start slaves=${ARM_SLAVES[*]}"

  for slave in "${ARM_SLAVES[@]}"; do
    run_cmd ethercat download -p "$slave" -t uint16 0x3002 0x00 0x0000 || true
  done

  for slave in "${ARM_SLAVES[@]}"; do
    run_cmd ethercat states -p "$slave" INIT || true
  done

  sleep 1

  for retry in 1 2 3; do
    log "op_retry=$retry"
    for slave in "${ARM_SLAVES[@]}"; do
      run_cmd ethercat states -p "$slave" OP || true
    done
    sleep 2
    if ethercat slaves | awk 'BEGIN{bad=0} $1 ~ /^[0-9]+$/ && $1 <= 17 && $3 != "OP" {bad=1} END{exit bad}'; then
      break
    fi
  done

  sleep "$POST_CLEAR_SETTLE_SECONDS"
  log "clear_done"
}

restart_genie_once() {
  if [ "$ALLOW_RESTART" != "1" ]; then
    log "restart_skipped allow_restart=$ALLOW_RESTART"
    return 1
  fi

  log "restart_genie_app_start"
  run_cmd systemctl restart genie_app.service || return 1
  wait_for_genie || return 1
  sleep "$POST_RESTART_SETTLE_SECONDS"
  log "restart_genie_app_done"
}

main() {
  log "boot_recover_start check_only=$CHECK_ONLY allow_restart=$ALLOW_RESTART settle=${SETTLE_SECONDS}s"

  if [ "$(id -u)" -ne 0 ]; then
    log "must run as root"
    exit 1
  fi

  wait_for_genie || {
    log "genie_app.service did not become active"
    exit 1
  }

  wait_for_ethercat || {
    log "ethercat command did not become ready"
    exit 1
  }

  if [ "$SETTLE_SECONDS" -gt 0 ]; then
    log "settle_sleep_start seconds=$SETTLE_SECONDS"
    sleep "$SETTLE_SECONDS"
    log "settle_sleep_done"
  fi

  if scan_arm_faults; then
    log "boot_recover_done healthy_without_action"
    exit 0
  fi

  if [ "$CHECK_ONLY" = "1" ]; then
    log "boot_recover_done check_only_fault_present"
    exit 1
  fi

  clear_arm_latches
  if scan_arm_faults; then
    log "boot_recover_done healthy_after_clear"
    exit 0
  fi

  restart_genie_once || {
    log "boot_recover_done restart_failed_or_disabled"
    exit 1
  }

  clear_arm_latches
  if scan_arm_faults; then
    log "boot_recover_done healthy_after_restart_and_clear"
    exit 0
  fi

  log "boot_recover_done fault_still_present manual_recovery_required"
  exit 1
}

main "$@"
