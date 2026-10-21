#!/usr/bin/env bash
# Remove OpenArm Control's menu entry, icons and commands; for a full install,
# then offer to delete the app folder too (recordings included -- asked
# separately). An in-place install leaves the project folder alone.
#
#   bash uninstall.sh [--yes]      --yes: also delete the app folder, recordings kept
set -euo pipefail
APP_ID="openarm-control"
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REC="$HERE/installed.txt"
YES=0
[ "${1:-}" = "--yes" ] && YES=1
[ -f "$REC" ] || { echo "nothing to uninstall (no $REC)"; exit 0; }
mode="$(sed -n 's/^mode=//p' "$REC")"
app="$(sed -n 's/^app=//p' "$REC")"
DATA="${XDG_DATA_HOME:-$HOME/.local/share}"

grep '^/' "$REC" | while read -r f; do rm -f "$f" && echo "removed $f"; done
find "$DATA/icons/hicolor" -path "*/apps/$APP_ID.png" -delete 2>/dev/null || true
command -v update-desktop-database >/dev/null && update-desktop-database -q "$DATA/applications" || true
command -v gtk-update-icon-cache >/dev/null && gtk-update-icon-cache -q -t -f "$DATA/icons/hicolor" 2>/dev/null || true
rm -f "$REC"
echo "menu entry, icons and commands removed"

if [ "$mode" = "full" ] && [ -d "$app" ]; then
    del=0
    if [ $YES = 1 ]; then del=1
    elif [ -t 0 ]; then read -r -p "Delete the app folder $app as well? [y/N] " a; [[ "$a" =~ ^[Yy] ]] && del=1
    fi
    if [ $del = 1 ]; then
        keep=0
        if [ -d "$app/recordings" ] && [ -n "$(ls -A "$app/recordings" 2>/dev/null)" ]; then
            keep=1
            if [ $YES = 0 ] && [ -t 0 ]; then
                read -r -p "It has recordings. Delete them too? [y/N] " a; [[ "$a" =~ ^[Yy] ]] && keep=0
            fi
        fi
        if [ $keep = 1 ]; then
            find "$app" -mindepth 1 -maxdepth 1 ! -name recordings -exec rm -rf {} +
            echo "deleted $app except its recordings/"
        else
            rm -rf "$app"
            echo "deleted $app"
        fi
    fi
fi
