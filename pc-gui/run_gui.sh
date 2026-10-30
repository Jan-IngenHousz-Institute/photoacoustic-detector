#!/bin/sh
# Start the photoacoustic detector GUI on Linux / macOS in a private environment
# (pc-gui/.venv, created on first use). Needs python3 >= 3.10 with the venv module
# (Debian/Ubuntu: sudo apt install python3-venv). On Linux the user must be allowed
# to open the serial port (usually: sudo usermod -aG dialout $USER, then log in again).
set -e
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
    echo "Creating the GUI's Python environment in $(pwd)/.venv ..."
    python3 -m venv .venv
fi
if ! .venv/bin/python -c "import PySide6, pyqtgraph, serial, numpy" 2>/dev/null; then
    echo "Installing the GUI requirements (one time) ..."
    .venv/bin/python -m pip install --upgrade pip --quiet
    .venv/bin/python -m pip install -r requirements.txt
fi
exec .venv/bin/python pa_gui.py "$@"
