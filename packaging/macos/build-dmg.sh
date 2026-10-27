#!/usr/bin/env bash
# Builds target/package/Driftless.app and target/package/Driftless-<version>-macos-arm64.dmg.
#
#   packaging/macos/build-dmg.sh [--ohne-bauen]
#
# --ohne-bauen: take the release programs already in target/release (no cargo build).
# The bundle gets an ad-hoc signature only (no Apple account): Gatekeeper asks once, right
# click → „Öffnen“ starts it. Check the result with packaging/macos/check-dmg.sh.
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"

build=1
for arg in "$@"; do
    case "$arg" in
        --ohne-bauen) build=0 ;;
        *) echo "Unbekannte Option: $arg (erlaubt: --ohne-bauen)" >&2; exit 2 ;;
    esac
done

if [ "$(uname -m)" != "arm64" ]; then
    echo "Das .dmg wird nur auf einem Mac mit Apple-Chip (arm64) gebaut, nicht auf $(uname -m)." >&2
    exit 1
fi

# The version of [workspace.package] in Cargo.toml.
version="$(awk '
    /^\[/ { in_pkg = ($0 == "[workspace.package]") }
    in_pkg && /^version[[:space:]]*=/ { gsub(/.*=[[:space:]]*"|".*/, ""); print; exit }
' Cargo.toml)"
if ! [[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+ ]]; then
    echo "Keine Version in [workspace.package] von Cargo.toml gefunden." >&2
    exit 1
fi

if [ "$build" = 1 ]; then
    cargo build --release --locked -p driftless-app -p driftless-cli
fi
for bin in driftless-app driftless; do
    if [ ! -x "target/release/$bin" ]; then
        echo "target/release/$bin fehlt (ohne --ohne-bauen bauen)." >&2
        exit 1
    fi
done

package="target/package"
app="$package/Driftless.app"
dmg="$package/Driftless-$version-macos-arm64.dmg"
staging="$package/dmg"
rm -rf "$app" "$staging" "$dmg"
mkdir -p "$app/Contents/MacOS" "$app/Contents/Resources"

sed "s/@VERSION@/$version/g" packaging/macos/Info.plist.in > "$app/Contents/Info.plist"
plutil -lint "$app/Contents/Info.plist" >/dev/null
cp target/release/driftless-app target/release/driftless "$app/Contents/MacOS/"
cp assets/icon/driftless.icns "$app/Contents/Resources/driftless.icns"

# Ad-hoc signature: the command line first, then the bundle with its main program.
codesign --force --sign - "$app/Contents/MacOS/driftless"
codesign --force --deep --sign - "$app"

# The disk image: the bundle and a link to /Applications to drag it onto.
mkdir -p "$staging"
cp -R "$app" "$staging/"
ln -s /Applications "$staging/Applications"
hdiutil create -volname "Driftless $version" -srcfolder "$staging" -fs HFS+ -format UDZO \
    -ov "$dmg" >/dev/null
rm -rf "$staging"

echo "$dmg ($(du -h "$dmg" | cut -f1))"
