#!/usr/bin/env bash
# Checks a disk image of build-dmg.sh: mounts it read-only, then the files of the bundle,
# Info.plist, the signature, `--version` of both programs and the size. Exit ≠ 0 on any fault.
#
#   packaging/macos/check-dmg.sh target/package/Driftless-<version>-macos-arm64.dmg
set -euo pipefail

if [ $# -ne 1 ] || [ ! -f "$1" ]; then
    echo "Aufruf: $0 <Driftless-…-macos-arm64.dmg>" >&2
    exit 2
fi
dmg="$1"
root="$(cd "$(dirname "$0")/../.." && pwd)"

fail() {
    echo "FEHLER: $*" >&2
    exit 1
}
ok() {
    echo "ok: $*"
}

version="$(awk '
    /^\[/ { in_pkg = ($0 == "[workspace.package]") }
    in_pkg && /^version[[:space:]]*=/ { gsub(/.*=[[:space:]]*"|".*/, ""); print; exit }
' "$root/Cargo.toml")"
[ -n "$version" ] || fail "keine Version in Cargo.toml"

mnt="$(mktemp -d "${TMPDIR:-/tmp}/driftless-dmg.XXXXXX")"
attached=0
cleanup() {
    if [ "$attached" = 1 ]; then
        hdiutil detach -quiet "$mnt" || hdiutil detach -quiet -force "$mnt" || true
    fi
    rmdir "$mnt" 2>/dev/null || true
}
trap cleanup EXIT

hdiutil attach -nobrowse -readonly -noautoopen -mountpoint "$mnt" "$dmg" >/dev/null
attached=1
ok "eingehängt unter $mnt"

app="$mnt/Driftless.app"
[ -d "$app" ] || fail "Driftless.app fehlt im .dmg"
[ -L "$mnt/Applications" ] || fail "Verweis auf /Applications fehlt"
ok "Driftless.app und Verweis auf /Applications"

# Exactly these files in the bundle (the signature may hold several files).
files="$(cd "$app" && find . -type f -o -type l | sed 's|^\./||' | LC_ALL=C sort)"
expected="$(printf '%s\n' Contents/Info.plist Contents/MacOS/driftless Contents/MacOS/driftless-app \
    Contents/Resources/driftless.icns | LC_ALL=C sort)"
rest="$(printf '%s\n' "$files" | grep -v '^Contents/_CodeSignature/' || true)"
[ "$rest" = "$expected" ] || fail "unerwartete Dateien im Bündel:
$files"
printf '%s\n' "$files" | grep -q '^Contents/_CodeSignature/CodeResources$' \
    || fail "Contents/_CodeSignature/CodeResources fehlt"
ok "Dateiliste des Bündels"

plutil -lint "$app/Contents/Info.plist" >/dev/null || fail "Info.plist ungültig"
plist() { /usr/libexec/PlistBuddy -c "Print :$1" "$app/Contents/Info.plist"; }
[ "$(plist CFBundleExecutable)" = "driftless-app" ] || fail "CFBundleExecutable"
[ "$(plist CFBundleIdentifier)" = "io.github.roccat-xox.driftless" ] || fail "CFBundleIdentifier"
[ "$(plist CFBundleIconFile)" = "driftless" ] || fail "CFBundleIconFile"
[ "$(plist CFBundleShortVersionString)" = "$version" ] || fail "Version in Info.plist ≠ $version"
[ "$(plist LSMinimumSystemVersion)" = "15.0" ] || fail "LSMinimumSystemVersion"
[ "$(plist NSHighResolutionCapable)" = "true" ] || fail "NSHighResolutionCapable"
ok "Info.plist (Version $version)"

codesign --verify --deep --strict "$app" || fail "Signatur ungültig"
ok "Signatur (ad-hoc) gültig"

for bin in driftless driftless-app; do
    out="$("$app/Contents/MacOS/$bin" --version)" || fail "$bin --version endet mit Fehler"
    case "$out" in
        *"$version"*) ok "$bin --version: $out" ;;
        *) fail "$bin --version nennt $version nicht: $out" ;;
    esac
done

limit=$((500 * 1024 * 1024))
bytes="$(stat -f %z "$dmg")"
[ "$bytes" -lt "$limit" ] || fail ".dmg ist $bytes Byte groß (Grenze 500 MB)"
ok "Größe $(du -h "$dmg" | cut -f1) ($bytes Byte, Grenze 500 MB)"
