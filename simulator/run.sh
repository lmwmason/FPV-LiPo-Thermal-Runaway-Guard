#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

mkdir -p build
cmake -S . -B build > /dev/null
cmake --build build > /dev/null

./build/fpv_battery_sim

PYTHON_BIN="$SCRIPT_DIR/../python/venv/bin/python3"
if [ ! -x "$PYTHON_BIN" ]; then
    PYTHON_BIN="python3"
fi

cd visualize
"$PYTHON_BIN" plot.py
