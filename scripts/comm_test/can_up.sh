#!/usr/bin/env bash
set -euo pipefail

readonly BITRATE=1000000
readonly TX_QUEUE_LENGTH=1000

if (( $# == 0 )); then
  channels=(can0 can1)
else
  channels=("$@")
fi

sudo modprobe gs_usb

for channel in "${channels[@]}"; do
  if ! ip link show "$channel" >/dev/null 2>&1; then
    printf 'CAN 인터페이스를 찾을 수 없습니다: %s\n' "$channel" >&2
    exit 1
  fi

  # CAN bitrate can only be configured while the interface is down.
  sudo ip link set "$channel" down 2>/dev/null || true
  sudo ip link set "$channel" type can bitrate "$BITRATE"
  sudo ip link set "$channel" txqueuelen "$TX_QUEUE_LENGTH"
  sudo ip link set "$channel" up
  printf '%s 활성화 완료 (bitrate=%d, txqueuelen=%d)\n' \
    "$channel" "$BITRATE" "$TX_QUEUE_LENGTH"
done
