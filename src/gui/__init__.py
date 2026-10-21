"""
OpenArm desktop GUI (PySide6). Start it with `python3 gui_main.py` from src/.

  backend.py   worker thread running the drive modes, log capture, telemetry
  actions.py   GUI-only jobs (enable, hold, go limp, sim start/stop/load scene)
  kin.py       FK for the live view + a background thread for IK previews
  plots.py     matplotlib canvases: 3D arm view, recording plot, scope
  widgets.py   small reusable widgets (joint editor, gains editor, ...)
  tabs.py      one tab per drive mode, plus scope and settings
  app.py       main window: toolbar, docks, console, live view

Full reference: docs/08_gui.md.
"""
