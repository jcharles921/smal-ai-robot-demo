#!/usr/bin/env bash
# Linux / macOS / WSL2:  bash demo/run_demo.sh
cd "$(dirname "$0")"
python3 -m pip install --quiet --user -r requirements.txt 2>/dev/null \
  || python3 -m pip install --quiet --user --break-system-packages -r requirements.txt \
  || echo "(pip install failed - on Ubuntu try: sudo apt install python3-numpy python3-matplotlib python3-tk)"
python3 sim_demo.py "$@"
