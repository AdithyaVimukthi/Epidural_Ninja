#!/usr/bin/env bash
# OpenArm Control installer, for one user on Ubuntu (22.04 tested).
#
#   bash install.sh                 full install (what the .run installer runs):
#                                   copy the app to ~/OpenArm-Control, give it its own
#                                   Python environment, add the menu icon and commands
#   bash install.sh --in-place      this project folder as it is (its existing venv):
#                                   only the menu icon and commands -- for the
#                                   development computer
#
# Options:
#   --prefix DIR      where the full install goes (default ~/OpenArm-Control)
#   --yes             do the system steps that need sudo without asking
#   --no-system       skip them (and just report what's missing)
#   --real-arm        also set up the real-arm packages (OpenArm PPA, python3-openarm-can,
#                     can-utils) -- otherwise asked
#   --from DIR        the unpacked installer (the .run file passes this)
#
# Installs, per user (nothing outside $HOME but the optional apt packages):
#   ~/.local/bin/openarm-gui        start the GUI          (log: ~/.local/state/openarm-control/gui.log
#   ~/.local/bin/openarm            the openarm> shell      when started from the menu)
#   ~/.local/share/applications/openarm-control.desktop + icons -> "OpenArm Control" in the menu
#   <app>/packaging/uninstall.sh    removes all of it again
set -euo pipefail

APP_ID="openarm-control"
APP_NAME="OpenArm Control"
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
SRC_ROOT="$(dirname "$HERE")"           # the project folder (or the unpacked installer)
PREFIX="$HOME/OpenArm-Control"
MODE="full"
ASK_SYSTEM="ask"
REAL_ARM="ask"
VERSION="$(cat "$HERE/VERSION" 2>/dev/null || echo "?")"

while [ $# -gt 0 ]; do
    case "$1" in
        --prefix) PREFIX="$(readlink -m "$2")"; shift 2 ;;
        --in-place) MODE="in-place"; shift ;;
        --yes) ASK_SYSTEM="yes"; shift ;;
        --no-system) ASK_SYSTEM="no"; shift ;;
        --real-arm) REAL_ARM="yes"; shift ;;
        --from) SRC_ROOT="$(readlink -f "$2")"; shift 2 ;;
        -h|--help) sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1 (--help lists them)"; exit 2 ;;
    esac
done

say()  { printf '\033[1m%s\033[0m\n' "$*"; }
ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }
die()  { printf '\033[31mError:\033[0m %s\n' "$*" >&2; exit 1; }
# yes/no question; --yes / --no-system / no terminal decide without asking
confirm() {
    case "$ASK_SYSTEM" in yes) return 0 ;; no) return 1 ;; esac
    [ -t 0 ] || return 1
    read -r -p "  $1 [y/N] " a; [[ "$a" =~ ^[Yy] ]]
}

unset PYTHONPATH            # e.g. ROS 2's -- the app has its own environment
say "$APP_NAME $VERSION -- ${MODE} install"

# ── where things go ─────────────────────────────────────────────────────────
if [ "$MODE" = "in-place" ]; then
    APP="$SRC_ROOT"
    VENV=""
    for v in "$APP/.venv" "$(dirname "$APP")/.venv"; do
        [ -x "$v/bin/python" ] && VENV="$v" && break
    done
    [ -n "$VENV" ] || die "no venv next to $APP (looked for .venv in it and in its parent folder)"
else
    APP="$PREFIX"
    VENV="$APP/.venv"
    case "$APP" in *" "*) die "the install folder can't contain spaces: $APP" ;; esac
fi
BIN="$HOME/.local/bin"
DATA="${XDG_DATA_HOME:-$HOME/.local/share}"
ok "app:    $APP"
ok "python: $VENV"

# ── system packages (sudo) ──────────────────────────────────────────────────
need_pkgs=()
python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))' || die "needs Python 3.10 or newer (this is $(python3 --version))"
if [ "$MODE" = "full" ] && ! python3 -c 'import ensurepip' 2>/dev/null; then
    need_pkgs+=("python3-venv")
fi
dpkg -s libxcb-cursor0 >/dev/null 2>&1 || need_pkgs+=("libxcb-cursor0")   # Qt 6 on X11
arm_pkgs=()
if ! python3 -c 'import sys; sys.path.append("/usr/lib/python3/dist-packages"); import openarm_can' 2>/dev/null; then
    if [ "$REAL_ARM" = "ask" ]; then
        if [ "$ASK_SYSTEM" = "yes" ]; then REAL_ARM="no"; fi       # --yes alone doesn't add a PPA
        if [ "$ASK_SYSTEM" = "ask" ] && [ -t 0 ]; then
            read -r -p "  Will this computer drive the real arm (CAN)? Adds the OpenArm PPA [y/N] " a
            [[ "$a" =~ ^[Yy] ]] && REAL_ARM="yes" || REAL_ARM="no"
        fi
    fi
    [ "$REAL_ARM" = "yes" ] && arm_pkgs=("python3-openarm-can" "can-utils")
fi
if [ ${#need_pkgs[@]} -gt 0 ] || [ ${#arm_pkgs[@]} -gt 0 ]; then
    say "System packages: ${need_pkgs[*]} ${arm_pkgs[*]}"
    if confirm "Install them with sudo apt?"; then
        if [ ${#arm_pkgs[@]} -gt 0 ]; then
            sudo apt-get install -y software-properties-common
            sudo add-apt-repository -y ppa:openarm/main
        fi
        sudo apt-get update
        sudo apt-get install -y "${need_pkgs[@]}" "${arm_pkgs[@]}"
        ok "installed"
    else
        for p in "${need_pkgs[@]}"; do
            warn "not installed: $p"
        done
        [ ${#arm_pkgs[@]} -gt 0 ] && warn "real arm: sudo add-apt-repository ppa:openarm/main && sudo apt install python3-openarm-can can-utils"
    fi
fi
if ! id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    say "The load cell's USB serial port needs the dialout group"
    if confirm "Add $USER to dialout (sudo)? Takes effect after logging out and in"; then
        sudo usermod -aG dialout "$USER" && ok "added -- log out and back in once"
    else
        warn "skipped: sudo usermod -aG dialout $USER  (until then the load cell is 'permission denied')"
    fi
fi

# ── the app files (full install) ────────────────────────────────────────────
if [ "$MODE" = "full" ]; then
    say "Copying the app to $APP"
    mkdir -p "$APP"
    stamp="$(date +%Y%m%d_%H%M%S)"
    cfg="$APP/src/config/config.yaml"
    if [ -f "$cfg" ] && ! cmp -s "$cfg" "$SRC_ROOT/src/config/config.yaml"; then
        cp "$cfg" "$cfg.bak-$stamp"
        warn "your edited config.yaml kept as config.yaml.bak-$stamp (the new one replaces it)"
    fi
    # Everything from the installer; never touches recordings/ or user_config.yaml.
    (cd "$SRC_ROOT" && tar cf - --exclude=./packaging/wheels --exclude=./recordings \
        --exclude=./src/config/user_config.yaml .) | (cd "$APP" && tar xf -)
    mkdir -p "$APP/recordings"
    # The URDFs name their meshes by absolute path: point them here.
    python3 - "$APP" <<'PY'
import pathlib, re, sys
app = pathlib.Path(sys.argv[1])
for urdf in (app / "sim_files" / "udrf").glob("*.urdf"):
    text = urdf.read_text()
    new = re.sub(r'file://[^"]*?/sim_files/udrf/', lambda m: f"file://{app}/sim_files/udrf/", text)
    if new != text:
        urdf.write_text(new)
PY
    ok "files copied"

    say "Python environment (downloads ~400 MB the first time)"
    if ! { [ -x "$VENV/bin/python" ] && "$VENV/bin/python" -m pip --version >/dev/null 2>&1; }; then
        if python3 -c 'import ensurepip' 2>/dev/null; then
            python3 -m venv "$VENV"
        else
            # No python3-venv: make the environment without pip, then fetch pip itself.
            warn "python3-venv isn't installed -- making the environment without it (pip from bootstrap.pypa.io)"
            python3 -m venv --without-pip "$VENV"
            getpip="$(mktemp)"
            python3 -c 'import sys, urllib.request; urllib.request.urlretrieve("https://bootstrap.pypa.io/get-pip.py", sys.argv[1])' "$getpip" \
                || die "couldn't download pip -- install python3-venv (sudo apt install python3-venv) or check the internet"
            "$VENV/bin/python" "$getpip" --quiet
            rm -f "$getpip"
        fi
    fi
    "$VENV/bin/python" -m pip install --quiet --upgrade pip
    pip_src=()
    if [ -d "$SRC_ROOT/packaging/wheels" ]; then
        pip_src=(--no-index --find-links "$SRC_ROOT/packaging/wheels")
        ok "offline installer: using its bundled wheels"
    fi
    if ! "$VENV/bin/python" -m pip install "${pip_src[@]}" -r "$APP/packaging/requirements.txt"; then
        warn "the pinned versions didn't install -- trying compatible ones"
        "$VENV/bin/python" -m pip install "${pip_src[@]}" -r "$APP/packaging/requirements-loose.txt"
    fi
    ok "libraries installed"
    "$VENV/bin/python" - "$APP/src" <<'PY' || die "the app's libraries don't import -- see above"
import sys; sys.path.insert(0, sys.argv[1]); import _bootstrap  # noqa
import numpy, scipy, matplotlib, PySide6, roboticstoolbox, spatialmath, pinocchio, yaml, serial, zmq  # noqa
import config, kinematics  # noqa
kinematics.ArmKinematics("right")
print("  \033[32m✓\033[0m libraries and arm model load")
PY
fi

# ── commands, menu entry, icon ─────────────────────────────────────────────
say "Menu entry and commands"
mkdir -p "$BIN" "$DATA/applications"
cat > "$BIN/openarm-gui" <<EOF
#!/usr/bin/env bash
# $APP_NAME -- the GUI. Made by $APP/packaging/install.sh
unset PYTHONPATH
cd "$APP/src" || exit 1
if [ -t 1 ]; then
    exec "$VENV/bin/python" gui_main.py "\$@"
fi
LOG="\${XDG_STATE_HOME:-\$HOME/.local/state}/$APP_ID/gui.log"
mkdir -p "\$(dirname "\$LOG")"
echo "=== \$(date -Is) ===" >> "\$LOG"
exec "$VENV/bin/python" gui_main.py "\$@" >> "\$LOG" 2>&1
EOF
cat > "$BIN/openarm" <<EOF
#!/usr/bin/env bash
# $APP_NAME -- the openarm> shell / one-shot commands (main.py). Made by $APP/packaging/install.sh
unset PYTHONPATH
cd "$APP/src" || exit 1
exec "$VENV/bin/python" main.py "\$@"
EOF
chmod +x "$BIN/openarm-gui" "$BIN/openarm"
for d in "$APP/packaging/icon/hicolor"/*/; do
    size="$(basename "$d")"
    mkdir -p "$DATA/icons/hicolor/$size/apps"
    cp "$d/apps/$APP_ID.png" "$DATA/icons/hicolor/$size/apps/$APP_ID.png"
done
cat > "$DATA/applications/$APP_ID.desktop" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=$APP_NAME
GenericName=Robot arm control
Comment=OpenArm: joint, Cartesian and line moves, plunger with load-cell feedback, record and replay
Exec=$BIN/openarm-gui
Icon=$APP_ID
Terminal=false
Categories=Science;Engineering;
Keywords=robot;arm;openarm;needle;loadcell;
StartupNotify=true
StartupWMClass=OpenArm GUI
EOF
command -v update-desktop-database >/dev/null && update-desktop-database -q "$DATA/applications" || true
command -v gtk-update-icon-cache >/dev/null && gtk-update-icon-cache -q -t -f "$DATA/icons/hicolor" 2>/dev/null || true
command -v desktop-file-validate >/dev/null && desktop-file-validate "$DATA/applications/$APP_ID.desktop" || true

cat > "$APP/packaging/installed.txt" <<EOF
mode=$MODE
app=$APP
venv=$VENV
version=$VERSION
date=$(date -Is)
$BIN/openarm-gui
$BIN/openarm
$DATA/applications/$APP_ID.desktop
EOF
ok "\"$APP_NAME\" is in the app menu; commands: openarm-gui, openarm"
case ":$PATH:" in *":$BIN:"*) ;; *) warn "$BIN isn't on your PATH yet -- open a new terminal (or log in again)" ;; esac

say "Done."
echo "  Start it from the app menu (search \"OpenArm\"), or run: openarm-gui"
echo "  Uninstall: bash \"$APP/packaging/uninstall.sh\""
if [ "$MODE" = "full" ]; then
    echo "  Recordings go to $APP/recordings; settings in $APP/src/config/config.yaml"
    echo "  Simulation: install CoppeliaSim separately and load $APP/sim_files/dual_openarm_org.ttt"
    echo "  Real arm: bring the CAN interfaces up first -- $APP/docs/01_hardware_setup.md"
fi
