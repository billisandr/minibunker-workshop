#!/usr/bin/env bash
# ============================================================================
#  run_ui.sh — launch the Streamlit control panel on the HOST (not the container).
#
#  The UI reaches the ROS graph only over rosbridge (ws://HOST:9090), which the
#  sim/real launches start and start_sim.sh/start_real.sh publish on :9090.
#  This script creates a small local venv so the UI's Python is independent of
#  ROS Noetic's Python 3.8.
#
#  Usage:  bash catkin_ws/src/minibunker_ui/run_ui.sh
#  Then open http://localhost:8501
# ============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/.venv-ui"

PY="$(command -v python3 || command -v python)"
[ -n "$PY" ] || { echo "python not found on PATH"; exit 1; }

if [ ! -d "$VENV" ]; then
    echo "[run_ui] creating venv at $VENV"
    "$PY" -m venv "$VENV"
fi
# shellcheck disable=SC1091
if [ -f "$VENV/bin/activate" ]; then source "$VENV/bin/activate"; else source "$VENV/Scripts/activate"; fi

pip install --quiet --upgrade pip
pip install --quiet -r "$HERE/requirements.txt"

echo "[run_ui] starting Streamlit on http://localhost:8501"
exec streamlit run "$HERE/app.py"
