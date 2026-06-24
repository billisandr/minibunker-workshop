#!/usr/bin/env bash
# ============================================================================
#  start_real.sh — bring up the MiniBunker SpaceMine station on the REAL robot
#  (Raspberry Pi 5 + Pi Camera + CAN -> bunker_base). Run ON THE PI.
#
#  Prereqs on the Pi host (see docs/HARDWARE_SETUP.md):
#    • CAN interface up:  sudo ip link set can0 up type can bitrate 500000
#    • Camera reachable by the container (see §9.3 — picamera2 on host, or a
#      V4L2 /dev/video* device passed through).
#    • Image built for arm64 (see docs/HARDWARE_SETUP.md):
#        docker build -f docker/Dockerfile \
#          --build-arg ROS_BASE_IMAGE=arm64v8/ros:noetic-ros-base \
#          -t minibunker-spacemine .
#
#  SAFETY: the rover boots DISARMED (zero Twist). It only moves after an
#  explicit ARM from the Streamlit UI or `mb_arm`. Keep the hardware e-stop
#  in hand and run inside the fenced arena only.
#
#  Usage: bash start_real.sh
# ============================================================================
set -euo pipefail

IMAGE="minibunker-spacemine"
CONTAINER="minibunker-real"
CAN_IF="${CAN_IF:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"

log() { echo "[start_real] $*"; }
die() { echo "[start_real] ERROR: $*" >&2; exit 1; }

docker info >/dev/null 2>&1 || die "Docker doesn't appear to be running."
docker image inspect "$IMAGE" >/dev/null 2>&1 \
    || die "Image '$IMAGE' not found — build it for arm64 first (see the header / docs/HARDWARE_SETUP.md)."

# --- 1. CAN interface must be up on the host --------------------------------
if ! ip link show "$CAN_IF" 2>/dev/null | grep -q "state UP"; then
    log "$CAN_IF is not UP — attempting to bring it up at ${CAN_BITRATE} bps (needs sudo)…"
    sudo ip link set "$CAN_IF" up type can bitrate "$CAN_BITRATE" \
        || die "Could not bring up $CAN_IF. Check the CAN adapter/HAT wiring (docs/HARDWARE_SETUP.md §CAN)."
fi
log "$CAN_IF is up."

# --- 2. Launch the real stack -----------------------------------------------
# --network host: CAN is a netdev, so the container shares the host's can0.
# --cap-add NET_ADMIN: lets bunker_base manage the CAN socket.
# device mounts: pass any V4L2 camera nodes through (picamera2 host-bridge is
# an alternative — see docs/HARDWARE_SETUP.md §Camera).
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
trap 'log "Stopping container…"; docker stop "$CONTAINER" >/dev/null 2>&1 || true' EXIT INT TERM

CAM_DEVICES=()
for d in /dev/video0 /dev/video1; do
    [ -e "$d" ] && CAM_DEVICES+=(--device "$d:$d")
done

log "Starting real-robot bringup (rover is DISARMED until you ARM it)…"
docker run -d --rm \
    --name "$CONTAINER" \
    --network host \
    --cap-add NET_ADMIN \
    "${CAM_DEVICES[@]}" \
    -v /run/udev:/run/udev:ro \
    "$IMAGE" bash -ic "mb_real" >/dev/null

log "Waiting for ROS…"
for _ in $(seq 1 60); do
    docker exec "$CONTAINER" bash -ic "rostopic list" >/dev/null 2>&1 && break
    sleep 1
done
log "Up. Streamlit panel:  bash catkin_ws/src/minibunker_ui/run_ui.sh"
log "ARM when ready:       docker exec -it $CONTAINER bash -ic mb_arm"
log "Following logs (Ctrl+C stops the station)…"
docker logs -f "$CONTAINER"
