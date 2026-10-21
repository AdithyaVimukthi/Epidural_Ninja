#!/usr/bin/env bash
# Build the OpenArm Control installer for other Ubuntu computers: one
# self-extracting file, packaging/dist/OpenArm-Control-<version>-installer.run.
# On the other computer:  bash OpenArm-Control-<version>-installer.run   (--help: options)
#
#   bash packaging/build_installer.sh             online: the installer downloads the
#                                                 Python libraries (~400 MB) from PyPI
#   bash packaging/build_installer.sh --offline   bundles them too (much bigger; for
#                                                 computers without internet -- same
#                                                 Ubuntu / Python version as this one)
#
# What goes in: src/, sim_files/ (URDFs, meshes, CoppeliaSim scenes), docs/, the
# load-cell firmware and recorder, packaging/ (installer, icon, requirements),
# run_gui.sh. Left out: recordings/, user_config.yaml, caches, the CAD (.step).
set -euo pipefail
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
ROOT="$(dirname "$HERE")"
VERSION="$(cat "$HERE/VERSION")"
OFFLINE=0
[ "${1:-}" = "--offline" ] && OFFLINE=1
DIST="$HERE/dist"
# Staged outside the project, so a pattern that misses can never make the
# copy include itself.
WORK="$(mktemp -d "${XDG_CACHE_HOME:-$HOME/.cache}/openarm-build.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
STAGE="$WORK/stage"
SUFFIX=""
[ $OFFLINE = 1 ] && SUFFIX="-offline"
NAME="OpenArm-Control-$VERSION-installer$SUFFIX.run"

mkdir -p "$STAGE" "$DIST"
echo "staging..."
# Member names are relative ("src/...", "packaging/..."): patterns without "./".
(cd "$ROOT" && tar cf - \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='.pytest_cache' \
    --exclude='src/config/user_config.yaml' --exclude='src/config/*.tmp' \
    --exclude='packaging/dist' --exclude='packaging/wheels' --exclude='packaging/installed.txt' \
    --exclude='*.step' --exclude='*.zip' --exclude='new_setup&files/arm_loadcell/recordings' \
    src sim_files docs "new_setup&files" packaging run_gui.sh) | (cd "$STAGE" && tar xf -)
du -sh "$STAGE" | sed 's/^/  staged /' 
if [ $OFFLINE = 1 ]; then
    echo "downloading the Python libraries for $(python3 --version)..."
    env -u PYTHONPATH python3 -m pip download --quiet -d "$STAGE/packaging/wheels" \
        -r "$HERE/requirements.txt" pip
fi

echo "packing..."
PAYLOAD="$WORK/payload.tar.gz"
(cd "$STAGE" && tar czf "$PAYLOAD" .)
cat > "$DIST/$NAME" <<EOF
#!/usr/bin/env bash
# OpenArm Control $VERSION installer (self-extracting). Run:  bash $NAME [--help]
set -e
case "\${1:-}" in -h|--help)
    echo "OpenArm Control $VERSION installer -- installs for the current user into ~/OpenArm-Control"
    echo "options are passed to install.sh:"
    echo "  --prefix DIR   install somewhere else"
    echo "  --yes          do the sudo steps (apt packages, dialout group) without asking"
    echo "  --no-system    skip them"
    echo "  --real-arm     also set up the real-arm packages (OpenArm PPA, python3-openarm-can, can-utils)"
    exit 0 ;;
esac
CACHE="\${XDG_CACHE_HOME:-\$HOME/.cache}"
mkdir -p "\$CACHE"
TMP="\$(mktemp -d "\$CACHE/openarm-installer.XXXXXX")"
trap 'rm -rf "\$TMP"' EXIT
echo "Unpacking OpenArm Control $VERSION..."
LINE=\$(awk '/^__OPENARM_PAYLOAD__\$/ {print NR + 1; exit 0}' "\$0")
tail -n +"\$LINE" "\$0" | tar xzf - -C "\$TMP"
bash "\$TMP/packaging/install.sh" --from "\$TMP" "\$@"
exit \$?
__OPENARM_PAYLOAD__
EOF
cat "$PAYLOAD" >> "$DIST/$NAME"
chmod +x "$DIST/$NAME"
(cd "$DIST" && sha256sum "$NAME" > "$NAME.sha256")
echo "built $DIST/$NAME ($(du -h "$DIST/$NAME" | cut -f1))"
