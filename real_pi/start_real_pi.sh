#!/usr/bin/env bash
# ============================================================================
#  start_real_pi.sh — one command to start the native MiniBunker station.
#
#  Does the safe ordered start: bounce CAN up (gs_usb) -> confirm the Bunker is
#  on the bus -> launch run.py --web. Run ON THE PI.
#
#  MANUAL prereqs (the script reminds you):
#    • Bunker powered ON + booted
#    • RC transmitter OFF  (RC overrides CAN -> mode thrash -> EXCEPTION lock)
#    • hardware e-stop in hand; fenced arena
#
#  Usage (after installing the alias below):
#    mb                 # bounce CAN, check it, launch --web panel
#    mb -f              # skip the CAN sanity check (force)
#    mb --save-video /tmp/run.mp4   # extra args pass through to run.py
#
#  Install the alias (run once on the Pi):
#    echo "alias mb='bash ~/minibunker-workshop/real_pi/start_real_pi.sh'" >> ~/.bashrc
#    source ~/.bashrc
# ============================================================================
set -euo pipefail

IF="${MB_CAN_IF:-can0}"
BR="${MB_CAN_BITRATE:-500000}"
VENV="${MB_VENV:-$HOME/mb-venv}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SKIP_CHECK=0
if [ "${1:-}" = "-f" ]; then SKIP_CHECK=1; shift; fi

echo "[mb] === MiniBunker native station ==="
echo "[mb] checklist:  Bunker ON?   RC transmitter OFF?   e-stop in hand?"

# Guard: never run two stations on one CAN bus (two motion owners).
if pgrep -f 'python.*run\.py' >/dev/null 2>&1; then
    echo "[mb] ERROR: a run.py is already running — stop it first (Ctrl-C):" >&2
    pgrep -af 'python.*run\.py' >&2 || true
    exit 1
fi

# 1. bounce the gs_usb CAN interface up (no restart-ms on gs_usb)
echo "[mb] bringing up $IF @ ${BR}…"
sudo ip link set "$IF" down 2>/dev/null || true
sudo ip link set "$IF" up type can bitrate "$BR"
ip -br link show "$IF"

# 2. confirm the Bunker is actually transmitting before we send anything
if [ "$SKIP_CHECK" = 0 ]; then
    echo "[mb] checking for CAN frames (2s)…"
    timeout 2 candump "$IF" > /tmp/mb_candump.txt 2>/dev/null || true
    if [ -s /tmp/mb_candump.txt ]; then
        echo "[mb] CAN OK — sample frames:"; head -3 /tmp/mb_candump.txt
    else
        echo "[mb] ERROR: no CAN frames on $IF in 2s." >&2
        echo "[mb]   -> Is the Bunker powered on + cabled? (RC off?)" >&2
        echo "[mb]   -> Fix it and retry, or run 'mb -f' to skip this check." >&2
        exit 1
    fi
fi

# 3. venv + launch the station (extra args pass through)
# shellcheck disable=SC1091
source "$VENV/bin/activate"
cd "$HERE"
echo "[mb] launching: run.py --web $* "
echo "[mb]   panel -> http://$(hostname).local:8080   (Ctrl-C to stop)"
exec python run.py --web "$@"
