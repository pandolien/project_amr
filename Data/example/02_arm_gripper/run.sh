#!/usr/bin/env bash
# Project run script: creates the venv (first time), installs requirements.txt, runs main.py.
#   ./run.sh                 3D window
#   ./run.sh --no-window     no window (5 seconds)
cd "$(dirname "$0")" || exit 1
export SIMUL_LIB="$PWD/Simul"
if [ ! -x venv/bin/python ]; then
    echo "[venv] creating venv ..."
    python3 -m venv venv || { echo "[venv] failed. Install python3-venv (Ubuntu: sudo apt install python3-venv) and try again."; exit 1; }
fi
[ -f requirements.txt ] && venv/bin/python -m pip install -q --disable-pip-version-check -r requirements.txt
exec venv/bin/python main.py "$@"
