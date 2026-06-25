#!/usr/bin/env bash
# ============================================================================
#  teleop.sh — open a WASD keyboard teleop terminal for the running sim.
#
#  Drives the rover from the keyboard by running teleop_node.py INSIDE the
#  already-running sim container (so it shares the sim's ROS graph — this is the
#  in-simulator teleop, not the Streamlit UI pad). The node publishes
#  /minibunker/teleop_cmd; behaviour gates it (ARM + behavior/limits clamp) onto
#  /cmd_vel and only acts on it when mission/follow_item == none.
#
#  Keys:  W fwd  S back  A turn-left  D turn-right   X / space / K stop   Q quit
#
#  Usage (in a SECOND terminal, after `bash start_sim.sh`):
#     bash teleop.sh
#  Then, in the Streamlit UI: set Mission = none and press ARM, and drive here.
#
#  Override the container with MB_CONTAINER=... (defaults to the sim container).
# ============================================================================
set -euo pipefail
# Stop Git Bash rewriting the container-internal paths/args we pass to docker.
export MSYS_NO_PATHCONV=1

CONTAINER="${MB_CONTAINER:-minibunker-sim}"

docker inspect "$CONTAINER" >/dev/null 2>&1 || {
    echo "[teleop] container '$CONTAINER' is not running."
    echo "[teleop] start the sim first:   bash start_sim.sh"
    echo "[teleop] (or set MB_CONTAINER=<name> if you renamed it)"
    exit 1
}

# teleop_node reads raw keyboard input, so it needs a real TTY. `docker exec -it`
# allocates one; on Git Bash that requires winpty, elsewhere plain -it is fine.
# `bash -ic` sources the container's ROS env (matching start_sim.sh) so rosrun
# can find the node.
RUN="rosrun minibunker_behavior teleop_node.py"
echo "[teleop] WASD teleop -> $CONTAINER   (Mission must be 'none' + ARMED to move)"
if command -v winpty >/dev/null 2>&1; then
    exec winpty docker exec -it "$CONTAINER" bash -ic "$RUN"
else
    exec docker exec -it "$CONTAINER" bash -ic "$RUN"
fi
