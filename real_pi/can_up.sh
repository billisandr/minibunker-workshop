#!/usr/bin/env bash
# ============================================================================
#  can_up.sh — bring up the Bunker CAN interface (run on the Pi).
#
#  The adapter here is a gs_usb USB-CAN dongle, which does NOT support
#  `restart-ms` (it errors "Device doesn't support restart from Bus Off" and
#  leaves the link down). So we bounce it plainly. Use this each session, or
#  after a bus-off (gs_usb can't auto-recover — a manual down/up is the fix).
#
#  Usage:  bash can_up.sh            # can0 @ 500000
#          bash can_up.sh can0 500000
# ============================================================================
set -euo pipefail
IF="${1:-can0}"
BR="${2:-500000}"

sudo ip link set "$IF" down 2>/dev/null || true
sudo ip link set "$IF" up type can bitrate "$BR"
echo "[can_up] $IF up @ ${BR}:"
ip -br link show "$IF"
echo "[can_up] sanity: candump $IF should now show 0x211/0x221 from the Bunker."
