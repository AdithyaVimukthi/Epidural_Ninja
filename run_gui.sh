#!/usr/bin/env bash
# Start the OpenArm GUI with the project venv. See docs/08_gui.md.
# The venv is ../.venv next to src/ (an installed copy) or ../../.venv (the
# development layout, OpenArm_Project1/.venv), else the system python3.
cd "$(dirname "$(readlink -f "$0")")/src" || exit 1
for PY in ../.venv/bin/python ../../.venv/bin/python; do
    [ -x "$PY" ] && break
done
[ -x "$PY" ] || PY=python3
exec "$PY" gui_main.py "$@"
