#!/usr/bin/env bash
# Builds target/package/Driftless.app and target/package/Driftless-<version>-macos-arm64.dmg,
# and the licence notices target/package/THIRD-PARTY-NOTICES.html (in the bundle too), for which
# cargo-about is needed (cargo install cargo-about --locked --features cli).
#
#   packaging/macos/build-dmg.sh [--ohne-bauen]
#
# --ohne-bauen: take the release programs already in target/release (no cargo build).
# The programs are built for the oldest macOS that Info.plist names (LSMinimumSystemVersion, as
# MACOSX_DEPLOYMENT_TARGET); check-dmg.sh checks both, also for programs taken with --ohne-bauen.
# The bundle gets an ad-hoc signature only (no Apple account): Gatekeeper blocks the first start;
# then System Settings → „Datenschutz & Sicherheit“ → „Dennoch öffnen“, or
# `xattr -dr com.apple.quarantine /Applications/Driftless.app`. Check the result with
# packaging/macos/check-dmg.sh.
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

# The oldest macOS: LSMinimumSystemVersion of Info.plist, so that macOS opens the app on every
# system the programs are built for.
floor="$(/usr/libexec/PlistBuddy -c 'Print :LSMinimumSystemVersion' packaging/macos/Info.plist.in)"
export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-$floor}"
if [ "$MACOSX_DEPLOYMENT_TARGET" != "$floor" ]; then
    echo "MACOSX_DEPLOYMENT_TARGET=$MACOSX_DEPLOYMENT_TARGET passt nicht zu LSMinimumSystemVersion $floor in packaging/macos/Info.plist.in." >&2
    exit 1
fi

# Before the build, so that a missing cargo-about does not wait for it.
if ! command -v cargo-about >/dev/null; then
    echo "cargo-about fehlt (für die Lizenzen Dritter): cargo install cargo-about --locked --features cli" >&2
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
# The licence notices of the third-party parts (packaging/about.toml): in the bundle, and for the
# release next to the image.
notices="$package/THIRD-PARTY-NOTICES.html"
rm -rf "$app" "$staging" "$dmg" "$notices"
mkdir -p "$app/Contents/MacOS" "$app/Contents/Resources"

cargo about generate --workspace --locked --fail -c packaging/about.toml -o "$notices" \
    packaging/about.hbs
if [ ! -s "$notices" ]; then
    echo "$notices fehlt oder ist leer." >&2
    exit 1
fi

sed "s/@VERSION@/$version/g" packaging/macos/Info.plist.in > "$app/Contents/Info.plist"
plutil -lint "$app/Contents/Info.plist" >/dev/null
cp target/release/driftless-app target/release/driftless "$app/Contents/MacOS/"
cp assets/icon/driftless.icns "$app/Contents/Resources/driftless.icns"
cp "$notices" "$app/Contents/Resources/THIRD-PARTY-NOTICES.html"

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
