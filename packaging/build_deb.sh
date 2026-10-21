#!/usr/bin/env bash
# Build the OpenArm Control .deb: packaging/dist/openarm-control_<version>_all.deb.
# On the other computer:  sudo apt install ./openarm-control_<version>_all.deb
#
#   bash packaging/build_deb.sh             online: installing it downloads the Python
#                                           libraries (~400 MB) from PyPI (postinst)
#   bash packaging/build_deb.sh --offline   bundles them too: ..._amd64.deb, much bigger,
#                                           for computers without internet -- same
#                                           Python version as this one
#
# What goes in, under /opt/openarm-control (read-only; the launchers send settings and
# recordings to ~/OpenArm -- config.py OPENARM_DATA_DIR): src/, sim_files/, docs/, the
# load-cell firmware and recorder, the icon and requirements. Plus /usr/bin/openarm-gui,
# /usr/bin/openarm, the menu entry and icons. The maintainer scripts are in packaging/deb/.
set -euo pipefail
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
ROOT="$(dirname "$HERE")"
VERSION="$(cat "$HERE/VERSION")"
PKG="openarm-control"
APP_ID="openarm-control"
DEST="/opt/$PKG"
MAINTAINER="${DEB_MAINTAINER:-OpenArm Control <adithyavimukthi23@gmail.com>}"
OFFLINE=0
[ "${1:-}" = "--offline" ] && OFFLINE=1
DIST="$HERE/dist"
WORK="$(mktemp -d "${XDG_CACHE_HOME:-$HOME/.cache}/openarm-deb.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
STAGE="$WORK/$PKG"
ARCH="all"
DEPENDS="python3 (>= 3.10), python3-venv, libxcb-cursor0, libgl1, libegl1, libxkbcommon-x11-0, libfontconfig1, libdbus-1-3"
if [ $OFFLINE = 1 ]; then
    # The wheels are built for this machine's CPU and Python.
    ARCH="$(dpkg --print-architecture)"
    pyver="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
    pynext="$(python3 -c 'import sys; print("%d.%d" % (sys.version_info[0], sys.version_info[1] + 1))')"
    DEPENDS="python3 (>= $pyver), python3 (<< $pynext), ${DEPENDS#python3 (>= 3.10), }"
fi
NAME="${PKG}_${VERSION}_${ARCH}.deb"

umask 022
mkdir -p "$STAGE$DEST" "$STAGE/DEBIAN" "$STAGE/usr/bin" "$STAGE/usr/share/applications" "$DIST"
echo "staging..."
(cd "$ROOT" && tar cf - \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='.pytest_cache' \
    --exclude='src/config/user_config.yaml' --exclude='src/config/*.tmp' \
    --exclude='*.step' --exclude='*.zip' --exclude='new_setup&files/arm_loadcell/recordings' \
    src sim_files docs "new_setup&files" \
    packaging/VERSION packaging/requirements.txt packaging/requirements-loose.txt packaging/icon) \
    | (cd "$STAGE$DEST" && tar xf - --no-same-owner)
# The URDFs name their meshes by absolute path: point them at /opt.
python3 - "$STAGE$DEST" "$DEST" <<'PY'
import pathlib, re, sys
stage, dest = pathlib.Path(sys.argv[1]), sys.argv[2]
for urdf in (stage / "sim_files" / "udrf").glob("*.urdf"):
    text = urdf.read_text()
    new = re.sub(r'file://[^"]*?/sim_files/udrf/', f"file://{dest}/sim_files/udrf/", text)
    if new != text:
        urdf.write_text(new)
PY
if [ $OFFLINE = 1 ]; then
    echo "downloading the Python libraries for $(python3 --version)..."
    env -u PYTHONPATH python3 -m pip download --quiet -d "$STAGE$DEST/packaging/wheels" \
        -r "$HERE/requirements.txt" pip
fi

install -m 755 "$HERE/deb/openarm-gui" "$HERE/deb/openarm" "$STAGE/usr/bin/"
for d in "$HERE/icon/hicolor"/*/; do
    size="$(basename "$d")"
    install -D -m 644 "$d/apps/$APP_ID.png" "$STAGE/usr/share/icons/hicolor/$size/apps/$APP_ID.png"
done
cat > "$STAGE/usr/share/applications/$APP_ID.desktop" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=OpenArm Control
GenericName=Robot arm control
Comment=OpenArm: joint, Cartesian and line moves, plunger with load-cell feedback, record and replay
Exec=/usr/bin/openarm-gui
Icon=$APP_ID
Terminal=false
Categories=Science;Engineering;
Keywords=robot;arm;openarm;needle;loadcell;
StartupNotify=true
StartupWMClass=OpenArm GUI
EOF
install -D -m 644 /dev/stdin "$STAGE/usr/share/doc/$PKG/README" <<EOF
OpenArm Control $VERSION -- installed in $DEST.
Start it from the app menu ("OpenArm Control") or with: openarm-gui
The openarm> shell / one-shot commands: openarm --help
Settings (user_config.yaml) and recordings: ~/OpenArm (set OPENARM_DATA_DIR to change).
Documentation: $DEST/docs/
EOF

install -m 755 "$HERE/deb/postinst" "$HERE/deb/postrm" "$STAGE/DEBIAN/"
find "$STAGE" -type d -exec chmod 755 {} +
cat > "$STAGE/DEBIAN/control" <<EOF
Package: $PKG
Version: $VERSION
Section: science
Priority: optional
Architecture: $ARCH
Maintainer: $MAINTAINER
Installed-Size: $(du -sk --exclude=DEBIAN "$STAGE" | cut -f1)
Depends: $DEPENDS
Recommends: can-utils
Suggests: python3-openarm-can
Description: OpenArm robot arm control -- GUI and command line
 Drives the OpenArm arms in CoppeliaSim, on the real arm over CAN-FD, or both:
 joint, Cartesian and straight-line moves, the plunger experiment with
 load-cell force feedback, and recording / replaying motions.
 .
 The Python libraries go into /opt/openarm-control/venv when the package is
 configured (downloaded from PyPI unless this is the offline build). The real
 arm also needs python3-openarm-can from ppa:openarm/main.
EOF

echo "packing (xz, takes a few minutes)..."
dpkg-deb --root-owner-group -Zxz --build "$STAGE" "$DIST/$NAME" >/dev/null
(cd "$DIST" && sha256sum "$NAME" > "$NAME.sha256")
echo "built $DIST/$NAME ($(du -h "$DIST/$NAME" | cut -f1))"
