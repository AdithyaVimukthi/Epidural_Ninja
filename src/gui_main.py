"""
OpenArm GUI -- a desktop front end for everything main.py does: joint,
kinematic (Cartesian), line, imitate, record, replay and home, on sim, the real
arm or both, with a live 3D view, joint table, scope and console.

    python3 gui_main.py          # from OpenArm_v3_2_GUI/src/, in the project venv

The GUI runs the same mode code as the CLI (via main.dispatch), so every
button behaves exactly like the matching openarm> command. Full reference:
docs/08_gui.md.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _bootstrap  # noqa: F401,E402 -- puts every src/ sub-directory on sys.path

from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtGui import QColor, QIcon, QPixmap  # noqa: E402
from PySide6.QtWidgets import QApplication, QSplashScreen  # noqa: E402


ICON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "packaging", "icon",
                    "openarm-control.png")


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("OpenArm GUI")
    app.setOrganizationName("OpenArm")
    # Matches the launcher (openarm-control.desktop), so the dock shows this
    # window under its icon.
    app.setDesktopFileName("openarm-control")
    if os.path.exists(ICON):
        app.setWindowIcon(QIcon(ICON))
    pixmap = QPixmap(460, 150)
    pixmap.fill(QColor("#1e2027"))
    splash = QSplashScreen(pixmap)
    splash.showMessage("OpenArm GUI\n\nloading kinematics, CAN and sim modules…",
                       Qt.AlignmentFlag.AlignCenter, QColor("#d7dae0"))
    splash.show()
    app.processEvents()

    from gui.app import run   # the heavy imports (roboticstoolbox, pinocchio, ...) happen here
    return run(app, splash)


if __name__ == "__main__":
    sys.exit(main())
