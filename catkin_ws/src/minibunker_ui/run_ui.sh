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

# Prefer a real interpreter. On Windows `python3` is usually the Microsoft Store
# stub; the real one is `py`/`python`. On Linux/Mac (incl. the Pi) `py` is absent,
# so this falls through to python3/python. (`set -u` => initialise PY first.)
PY=""
for cand in py python3 python; do
    command -v "$cand" >/dev/null 2>&1 && { PY="$cand"; break; }
done
[ -n "$PY" ] || { echo "python not found on PATH"; exit 1; }

if [ ! -d "$VENV" ]; then
    echo "[run_ui] creating venv at $VENV"
    "$PY" -m venv "$VENV"
fi
# Call the venv's interpreter directly instead of `source activate`. The Windows
# `Scripts/activate` runs `uname` to detect MSYS/Cygwin; in a Git-Bash shell that
# doesn't have Git's usr/bin on PATH that fails with "uname: command not found".
# The venv python needs no activation, and this works the same on Win/Linux/Pi.
if [ -x "$VENV/bin/python" ]; then
    VPY="$VENV/bin/python"            # Linux / macOS / Pi
else
    VPY="$VENV/Scripts/python.exe"    # Windows
fi

"$VPY" -m pip install --quiet --upgrade pip
"$VPY" -m pip install --quiet -r "$HERE/requirements.txt"

echo "[run_ui] starting Streamlit on http://localhost:8501"
exec "$VPY" -m streamlit run "$HERE/app.py"
