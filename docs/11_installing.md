# Installing OpenArm Control

OpenArm Control appears in the Ubuntu app menu with the robot-arm icon. There
are three ways to install it: in place on this (development) computer, with a
single-file `.run` installer for one user on another Ubuntu computer, or with a
`.deb` package for every user on it. All put two commands on the PATH:

| Command | What it starts |
|---|---|
| `openarm-gui` | the GUI |
| `openarm` | the `openarm>` shell, or a one-shot command such as `openarm plunger --point 1 ...` |

Everything lives in `packaging/`:

| File | What it is |
|---|---|
| `install.sh` | The installer, in both modes (`--help`) |
| `uninstall.sh` | Removes it again |
| `build_installer.sh` | Builds the single-file installer for other computers |
| `build_deb.sh` | Builds the .deb package |
| `deb/` | The .deb's launchers (`openarm-gui`, `openarm`) and install/removal scripts (`postinst`, `postrm`) |
| `requirements.txt` | Python libraries, pinned to the tested versions |
| `requirements-loose.txt` | Fallback, for a different Python version |
| `icon/` | The app icon (`make_icon.py` makes it from `source.png`) |
| `VERSION` | The version number |

## This computer (in place)

```bash
cd "/home/adi99/Desktop/OpenArm_Project1/OpenArm_v4_GUI&ori"
bash packaging/install.sh --in-place
```

This adds only the menu entry, the icon and the commands. They run **this
project folder** with its existing venv (`OpenArm_Project1/.venv`), so code
changes, `config.yaml` and the recordings are the ones you already work with.

Started from the menu, the GUI's output also goes to
`~/.local/state/openarm-control/gui.log`.

## Other Ubuntu computers

**1. Build the installer** here, once per version (bump `packaging/VERSION`):

```bash
bash packaging/build_installer.sh            # -> packaging/dist/OpenArm-Control-4.1.0-installer.run (~180 MB)
bash packaging/build_installer.sh --offline  # also bundles the Python libraries, for computers
                                             # without internet (same Ubuntu / Python version)
```

**2. Install it on the other computer** by copying the `.run` file across
(USB stick, network), then running:

```bash
bash OpenArm-Control-4.1.0-installer.run
```

It installs for the current user into `~/OpenArm-Control`, which holds the
app, its own Python environment and `recordings/`. It asks before doing
anything that needs `sudo`:

| Step | Why |
|---|---|
| `libxcb-cursor0` | Qt needs it on X11 |
| `python3-venv` | Optional. Without it, pip is fetched directly. |
| Joining the `dialout` group | The load cell's USB serial port. Log out and back in once afterwards. |
| The real-arm packages (**only if you say the computer drives the arm**) | The OpenArm PPA, `python3-openarm-can` and `can-utils` |

Options are `--prefix DIR`, `--yes` (do the sudo steps without asking),
`--no-system` and `--real-arm`. The first install downloads about 400 MB of
Python libraries (unless the installer is the offline one).

**Upgrading:** run a newer installer over the old one. Recordings and
`user_config.yaml` are kept. An edited `config.yaml` is saved as
`config.yaml.bak-<date>` before the new one replaces it.

**Uninstalling:** `bash ~/OpenArm-Control/packaging/uninstall.sh`. It asks
before deleting the folder, and separately before deleting the recordings.

**Not part of the installer:**
- **CoppeliaSim** (simulation): install it separately and load
  `~/OpenArm-Control/sim_files/dual_openarm_org.ttt`.
- **Bringing up the CAN interfaces** before using the real arm: see
  [01_hardware_setup.md](01_hardware_setup.md).

## Other Ubuntu computers, for every user (.deb)

**1. Build the package** here (bump `packaging/VERSION` first):

```bash
bash packaging/build_deb.sh            # -> packaging/dist/openarm-control_4.1.0_all.deb (~145 MB)
bash packaging/build_deb.sh --offline  # ..._amd64.deb, also bundles the Python libraries
                                       # (same Python version as this computer)
```

**2. Install it on the other computer:**

```bash
sudo apt install ./openarm-control_4.1.0_all.deb
sudo usermod -aG dialout $USER         # the load cell; log out and in once
```

apt installs the system libraries it depends on. The app goes to
`/opt/openarm-control` (read-only), and its `postinst` makes the Python
environment `/opt/openarm-control/venv`, downloading about 400 MB from PyPI.
If that fails (no internet), fix it and run `sudo dpkg --configure -a`;
`sudo dpkg-reconfigure openarm-control` rebuilds a broken environment.

Each user's settings (`user_config.yaml`), `poses.json` and `recordings/`
go to `~/OpenArm`: the launchers set `OPENARM_DATA_DIR`, which `config.py`
and the GUI use instead of the app folder. Set it yourself to use another
folder. With the `config.yaml` read-only, hand-edited settings go in
`~/OpenArm/user_config.yaml`.

**Upgrading:** install the newer .deb the same way; `~/OpenArm` is kept.
**Uninstalling:** `sudo apt remove openarm-control`; `~/OpenArm` is kept.

Install either the .deb or the .run on a computer, not both: the .run's
`~/.local/bin/openarm-gui` would hide the .deb's `/usr/bin/openarm-gui`.

A computer without `python3-openarm-can` runs everything except the real arm.
Connecting to the arm then explains what to install.
