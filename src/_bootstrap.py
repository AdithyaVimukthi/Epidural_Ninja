"""
Puts every src/ sub-directory on sys.path, so modules keep importing each
other by bare name (`from config import ...`, `from arm_can import ...`)
after being split into config/, comm/, motion/, sim/ and modes/.

main.py imports this directly. Scripts run from a sub-directory
(python3 comm/arm_can.py, python3 motion/kinematics.py, ...) first put
src/ on sys.path and then import it.
"""
import os
import sys

SRC_DIR = os.path.dirname(os.path.abspath(__file__))
SUBDIRS = ("config", "comm", "motion", "sim", "modes")

for _d in reversed(SUBDIRS):
    _p = os.path.join(SRC_DIR, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)
if SRC_DIR not in sys.path:
    sys.path.append(SRC_DIR)
