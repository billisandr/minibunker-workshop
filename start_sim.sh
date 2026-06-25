#!/usr/bin/env bash
# ============================================================================
#  start_sim.sh — build + launch the MiniBunker Gazebo simulation
#  on the x86 DEV LAPTOP (no robot hardware).
#
#  Run from Git Bash ON THE WINDOWS HOST. Mirrors the proven ros_z1_teleop
#  pattern: detached container + foreground RViz to dodge the -it terminal
#  contention documented in project_ros_z1_zed_docker.
#
#  Usage:
#    bash start_sim.sh                       # build (if needed) + run sim
#    bash start_sim.sh --rebuild             # force a clean image rebuild
#    bash start_sim.sh "mb_sim backend:=cnn" # pass a launch override
#
#  DEV MOUNT: by default this bind-mounts the host catkin_ws/src over the copy
#  baked into the image, so edits to the world file, arena_setup.py, launch
#  files, minibunker.yaml and the Python nodes go LIVE on a plain relaunch — no
#  --rebuild needed (only C++ / custom-message changes still need a rebuild).
#  Set MB_NO_MOUNT=1 to disable and use the image's baked-in source.
# ============================================================================
set -euo pipefail

# Git Bash mangles POSIX-looking args into Windows paths before native .exe
# tools see them — disable that for every docker/powershell call below.
export MSYS_NO_PATHCONV=1

IMAGE="minibunker"
CONTAINER="minibunker-sim"
# Windows-style (not "/c/...") on purpose: MSYS_NO_PATHCONV=1 above disables
# Git Bash's automatic POSIX->Windows path translation, so a "/c/..." path
# passed into the embedded PowerShell -Command string below would reach
# Start-Process -FilePath literally and fail with "cannot find the file".
VCXSRV='C:\Program Files\VcXsrv\vcxsrv.exe'
LAUNCH="${1:-mb_sim}"
REBUILD=0
[ "${1:-}" = "--rebuild" ] && { REBUILD=1; LAUNCH="mb_sim"; }

log() { echo "[start_sim] $*"; }
die() { echo "[start_sim] ERROR: $*" >&2; exit 1; }

# --- 0. Docker up ------------------------------------------------------------
docker info >/dev/null 2>&1 || die "Docker Desktop doesn't appear to be running."

# --- 1. Build the image (first run, or --rebuild) ---------------------------
if [ "$REBUILD" = "1" ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    log "Building $IMAGE (this is slow the first time)…"
    docker build -f docker/Dockerfile -t "$IMAGE" .
fi

# --- 2. VcXsrv (X server) must run WITH -wgl for gzclient under software GL --
CMDLINE="$(powershell.exe -NoProfile -Command \
    "(Get-CimInstance Win32_Process -Filter \"Name='vcxsrv.exe'\").CommandLine" \
    2>/dev/null | tr -d '\r')"
if [ -z "$CMDLINE" ] || ! grep -q -- "-wgl" <<< "$CMDLINE"; then
    log "Starting VcXsrv with -wgl"
    powershell.exe -NoProfile -Command \
        "Get-Process vcxsrv -ErrorAction SilentlyContinue | Stop-Process -Force" >/dev/null 2>&1 || true
    powershell.exe -NoProfile -Command \
        "Start-Process -FilePath '$VCXSRV' -ArgumentList ':0 -multiwindow -clipboard -ac -wgl'" >/dev/null
    sleep 2
else
    log "VcXsrv already running with -wgl — OK"
fi

# --- 3. Launch the sim (detached) -------------------------------------------
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
trap 'log "Stopping container…"; docker stop "$CONTAINER" >/dev/null 2>&1 || true' EXIT INT TERM

# Dev source mount: overlay the host's catkin_ws/src onto the container so the
# launch-time files (world, arena_setup.py, launch, config, Python nodes) are
# read live — no rebuild, no Docker build-cache games. `pwd -W` gives the Windows
# path Docker Desktop wants; --mount source= avoids the drive-letter ':' clashing
# with -v's separator. Read-only so the container can't litter the host src.
MOUNT_ARGS=()
if [ "${MB_NO_MOUNT:-0}" != "1" ]; then
    SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/catkin_ws/src" && { pwd -W 2>/dev/null || pwd; })"
    MOUNT_ARGS=(--mount "type=bind,source=${SRC_DIR},target=/home/rosuser/catkin_ws/src,readonly")
    log "Dev mount (live edits): ${SRC_DIR} -> /home/rosuser/catkin_ws/src [ro]"
else
    log "MB_NO_MOUNT=1 -> using the source baked into the image (no live edits)"
fi

log "Starting Gazebo sim ($LAUNCH)…  rosbridge will be on ws://localhost:9090"
docker run -d --rm \
    --name "$CONTAINER" \
    -e DISPLAY=host.docker.internal:0.0 \
    -p 9090:9090 \
    ${MOUNT_ARGS[@]+"${MOUNT_ARGS[@]}"} \
    "$IMAGE" bash -ic "$LAUNCH" >/dev/null

# --- 4. Wait for ROS, then follow logs --------------------------------------
log "Waiting for ROS to come up…"
for _ in $(seq 1 90); do
    docker exec "$CONTAINER" bash -ic "rostopic list" >/dev/null 2>&1 && break
    sleep 1
done
log "ROS is up. To launch the Streamlit panel, in another terminal run:"
log "    bash catkin_ws/src/minibunker_ui/run_ui.sh"
log "To drive with the keyboard (Mission=none + ARM), in another terminal run:"
log "    bash teleop.sh"
log "Following sim logs (Ctrl+C stops the station)…"
docker logs -f "$CONTAINER"
