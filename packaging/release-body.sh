#!/usr/bin/env bash
# The text of the release draft (release.yml): the install steps of packaging/release-notes.md
# with @VERSION@ and @REPO@ filled in, then the section of this version in CHANGELOG.md – the
# lines after „## [<version>]“ up to the next „## [“. Stops with an error (a GitHub annotation)
# if that section is missing or empty, or if a placeholder is left.
#
#   packaging/release-body.sh <version> <repository address> <target file>
#   packaging/release-body.sh 0.1.0 https://github.com/ROCCAT-XOX/driftless /tmp/release.md
#
# Run from the root of the repository.
set -euo pipefail

version="${1:?Version fehlt, z. B. 0.1.0}"
repo="${2:?Adresse des Repositorys fehlt, z. B. https://github.com/ROCCAT-XOX/driftless}"
out="${3:?Zieldatei fehlt}"

# The section: from the line after the heading to the next version heading, without the empty
# lines at its start (command substitution drops those at its end).
section="$(awk -v head="## [$version]" '
    index($0, head) == 1 { found = 1; next }
    found && /^## \[/ { exit }
    found && !started && /^[[:space:]]*$/ { next }
    found { started = 1; print }
' CHANGELOG.md)"
if [ -z "$section" ]; then
  echo "::error::CHANGELOG.md hat keinen Abschnitt „## [$version]“ (oder er ist leer): vor dem Tag eintragen."
  exit 1
fi

{
  sed -e "s|@VERSION@|$version|g" -e "s|@REPO@|$repo|g" packaging/release-notes.md
  printf '\n## Version %s\n\n%s\n' "$version" "$section"
} > "$out"

if grep -n '@[A-Z][A-Z_]*@' "$out"; then
  echo "::error::packaging/release-notes.md: Platzhalter nicht ersetzt."
  exit 1
fi
