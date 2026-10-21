"""
Connection cache shared by every drive mode in main.py.

Real controllers are enabled on first use and stay enabled across commands
(so an interactive shell session doesn't re-enable motors for every move);
they're only disabled in close(), at the end of the one-shot command or when
the shell exits.
"""
from arm_can import ArmController
from sim_bridge import SimArmController


def resolve_sides(arm: str) -> list[str]:
    return ["left", "right"] if arm == "both" else [arm]


class Session:
    def __init__(self):
        self._real: dict[str, ArmController] = {}
        self._sim: dict[str, SimArmController] = {}
        # The needle load cell (comm/loadcell.py), shared by every command --
        # opened on first use, or handed over by the GUI with attach_loadcell().
        self.loadcell_dev = None

    def real(self, side: str) -> ArmController:
        if side not in self._real:
            print(f"[real] connecting to {side} arm...")
            ctrl = ArmController.for_side(side)
            ctrl.enable()
            self._real[side] = ctrl
        return self._real[side]

    def sim(self, side: str, host: str, port: int) -> SimArmController:
        if side not in self._sim:
            print(f"[sim] connecting to CoppeliaSim at {host}:{port} for {side} arm...")
            ctrl = SimArmController(side, host=host, port=port)
            ctrl.connect()
            self._sim[side] = ctrl
        return self._sim[side]

    def loadcell(self, port: str | None = None):
        """The real load cell, opened (config.yaml loadcell:) on first use."""
        lc = self.loadcell_dev
        if lc is None or lc.kind != "real" or not lc.is_open:
            import loadcell
            lc = loadcell.from_config("real", port=port)
            print(f"[loadcell] opening {lc.port if lc.port not in (None, '', 'auto') else loadcell.find_port()}...")
            lc.open()
            print(f"[loadcell] {lc.describe()}")
            self.loadcell_dev = lc
        return lc

    def attach_loadcell(self, lc) -> None:
        """Use this (already open) load cell from now on; closes the previous one."""
        old, self.loadcell_dev = self.loadcell_dev, lc
        if old is not None and old is not lc:
            old.close()

    def close_loadcell(self) -> None:
        lc, self.loadcell_dev = self.loadcell_dev, None
        if lc is not None:
            lc.close()

    def close(self) -> None:
        self.close_loadcell()
        for side, ctrl in self._real.items():
            print(f"[real] disabling {side} arm...")
            ctrl.disable()
        self._real.clear()
        self._sim.clear()
