#!/bin/bash
# Final check of "weniger Rauschen" on VM 101: all runs of spec section 11, then the tables.
#   run_rauschen.sh <label> [--abgekoppelt]
# Trajectory and map once per scan (map 0.5 cm) in <label>/basis/, the model S1-S3 from
# `driftless merge` in basis/modell, then one folder per variant with one folder per scan:
#   vorher_{2,1}cm, nachher_{2,1}cm  --ohne alles / default, all five sets
#   ohne_{anlegen,belichtung,entrauschen,verdeckung}_1cm  S1
#   cpu_2cm  S1, --gpu aus;  probe_2cm  S1, program of 99284b9: its glb must equal vorher_2cm
# Images side by side (vorher left, nachher right, also as relief) in vorher_*/<scan>/bilder/.
# Tables in <label>/tabelle.md. A finished step leaves <step>.ok and is skipped on a restart.
# --abgekoppelt  starts itself detached from the terminal, log in <label>/lauf.log
# Env: DQ_SCANS      the four scan folders S1 S2 S3 office (default: as in run_vorlagen.sh)
#      DQ_ROOT       result root (default ~/td/quality)
#      DQ_PYTHON     python with numpy, scipy, laspy (default python3; on the VM ~/td/quality-venv/bin/python)
#      DQ_PROBE_BIN  driftless binary of 99284b9 (without it the probe is skipped with a line)
set -uo pipefail

usage() { sed -n 2,15p "$0"; exit 1; }
[ $# -ge 1 ] || usage
label=$1; shift
detach=0
while [ $# -gt 0 ]; do
  case $1 in
    --abgekoppelt) detach=1; shift ;;
    *) usage ;;
  esac
done

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd "$here/../.." && pwd)
python=${DQ_PYTHON:-python3}
root=${DQ_ROOT:-$HOME/td/quality}
mkdir -p "$root" && root=$(cd "$root" && pwd)   # the links between the run folders need it absolute
default_scans="$HOME/td/jmk6-2026-10-22/20261022133554 $HOME/td/jmk6-2026-10-22/20261022132520 $HOME/td/jmk6-2026-10-22/20261022133028 $HOME/td/jmk6-2026-10-20/20261020173822"
read -r -a scans <<< "${DQ_SCANS:-$default_scans}"
[ ${#scans[@]} -eq 4 ] || { echo "DQ_SCANS braucht vier Ordner: S1 S2 S3 Büro" >&2; exit 1; }
out=$root/$label
base=$out/basis
map_voxel=0.005

mkdir -p "$base"
if [ "$detach" = 1 ]; then
  setsid nohup "$here/run_rauschen.sh" "$label" >> "$out/lauf.log" 2>&1 < /dev/null &
  echo "abgekoppelt, PID $!, Protokoll: $out/lauf.log"
  exit 0
fi
exec 9> "$out/.lauf.lock"
flock -n 9 || { echo "Für $label läuft schon ein Lauf" >&2; exit 1; }

export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-6} OMP_NUM_THREADS=6 OPENBLAS_NUM_THREADS=6 MKL_NUM_THREADS=6
export CARGO_BUILD_JOBS=${CARGO_BUILD_JOBS:-6}

# Always the program and the image test of this checkout, not an older build in PATH.
(cd "$repo" && nice -n 10 cargo build --release -p driftless-cli \
  && nice -n 10 cargo test --release -p driftless-steps --test quality_real --no-run) \
  || { echo "Bauen fehlgeschlagen" >&2; exit 1; }
export PATH=$repo/target/release:$PATH
command -v gpu-job > /dev/null || { echo "gpu-job fehlt (GPU-Schritte laufen über die Warteschlange)" >&2; exit 1; }

failed=0

# step <dir> <name> <gpu 0|1> <command…>: log with /usr/bin/time -v to <dir>/<name>.log
step() {
  local dir=$1 name=$2 gpu=$3; shift 3
  if [ -f "$dir/$name.ok" ]; then
    echo "== $(basename "$(dirname "$dir")")/$(basename "$dir") $name schon fertig"
    return 0
  fi
  echo "== $(date +%T) $(basename "$(dirname "$dir")")/$(basename "$dir") $name  load $(cut -d' ' -f1-3 /proc/loadavg)  free $(free -g | awk '/Mem/{print $7}')G"
  if [ "$gpu" = 1 ]; then
    gpu-job --name "dq-$name" -- nice -n 10 /usr/bin/time -v "$@" > "$dir/$name.log" 2>&1
  else
    nice -n 10 /usr/bin/time -v "$@" > "$dir/$name.log" 2>&1
  fi
  local rc=$?
  echo "   rc=$rc $(grep -E 'Elapsed' "$dir/$name.log" | sed 's/.*: //')  maxrss $(grep 'Maximum resident' "$dir/$name.log" | awk '{printf "%.2f GB", $NF/1048576}')"
  [ $rc -eq 0 ] && touch "$dir/$name.ok"
  return $rc
}

# surface_of <variant> <scan folder|modell> <voxel m> <gpu an|aus> [--ohne WAS]: the surface in
# <variant>/<scan>/oberflaeche and its figures; the cloud figures come from the base.
surface_of() {
  local variant=$1 scan=$2 voxel=$3 gpu=$4; shift 4
  local name src d on_gpu=1
  name=$(basename "$scan")
  src=$base/$name
  d=$out/$variant/$name
  mkdir -p "$d"
  [ -f "$base/canon.json" ] && ln -sfn "$base/canon.json" "$out/$variant/canon.json"
  ln -sfn "$src/color.las" "$d/color.las"
  ln -sfn "$src/traj.tum" "$d/traj.tum"
  local input=("$scan" --slam "$src")
  [ "$scan" = modell ] && input=(--model "$src")
  [ "$gpu" = an ] || on_gpu=0
  step "$d" surface "$on_gpu" driftless surface "${input[@]}" --voxel "$voxel" --gpu "$gpu" -o "$d/oberflaeche" "$@" \
    && step "$d" analyze 0 "$python" "$here/q_analyze.py" surface "$d" "$src/metrics.json"
}

# images_of <run dir> [<run dir B>]: images and hole figures of a run, with B side by side.
images_of() {
  local a=$1 b=${2:-}
  [ -f "$a/bilder.ok" ] && return 0
  if [ -n "$b" ]; then
    QA_B_SURF=$b/oberflaeche QA_B_CLOUD=$b/color.las QA_RELIEF=1 "$here/q_images.sh" "$a" || return 1
    # The holes of run B, as the table reads them from its own folder.
    sed -n 's/^LOECHER_B /LOECHER /p' "$a/images.log" > "$b/images.log"
  else
    "$here/q_images.sh" "$a" || return 1
  fi
  touch "$a/bilder.ok"
}

# ---- once per scan
ready=()
for scan in "${scans[@]}"; do
  [ -d "$scan" ] || { echo "Scan fehlt: $scan" >&2; failed=$((failed + 1)); continue; }
  d=$base/$(basename "$scan")
  mkdir -p "$d"
  if step "$d" slam 0 driftless slam "$scan" -o "$d" --ueberschreiben \
    && step "$d" map 1 driftless map "$scan" --slam "$d" --voxel "$map_voxel" --gpu an --alle-dateien \
    && step "$d" analyze 0 "$python" "$here/q_analyze.py" variant "$d"; then
    ready+=("$scan")
  else
    failed=$((failed + 1))
  fi
done
[ ${#ready[@]} -gt 0 ] || { echo "kein Scan fertig geworden" >&2; exit 1; }

is_ready() {
  local s
  for s in "${ready[@]}"; do [ "$s" = "$1" ] && return 0; done
  return 1
}

s1=${scans[0]} s2=${scans[1]} s3=${scans[2]}
n1=$(basename "$s1") n2=$(basename "$s2") n3=$(basename "$s3")
sets=("${ready[@]}")
if is_ready "$s1" && is_ready "$s2" && is_ready "$s3"; then
  # Spread of the room dimensions (Z5) and the transforms of the fixed views.
  step "$base" cross 0 "$python" "$here/q_analyze.py" cross "$base" "$n1" "$n2" "$n3" || failed=$((failed + 1))
  m=$base/modell
  mkdir -p "$m"
  if step "$m" merge 0 driftless merge "$s1" "$s2" "$s3" -o "$m" \
    --slam "$base/$n1" --slam "$base/$n2" --slam "$base/$n3" \
    --map "$base/$n1" --map "$base/$n2" --map "$base/$n3"; then
    # For the images: the model cloud, and the trajectory of S1 (the model frame is the frame of S1).
    ln -sfn "$m/modell.las" "$m/color.las"
    ln -sfn "$base/$n1/traj.tum" "$m/traj.tum"
    sets+=(modell)
  else
    failed=$((failed + 1))
  fi
fi

# ---- vorher and nachher, all five sets, the coarse size first (early result)
for size in "2 0.02" "1 0.01"; do
  read -r cm voxel <<< "$size"
  for scan in "${sets[@]}"; do
    name=$(basename "$scan")
    { surface_of "vorher_${cm}cm" "$scan" "$voxel" an --ohne alles \
      && surface_of "nachher_${cm}cm" "$scan" "$voxel" an \
      && images_of "$out/vorher_${cm}cm/$name" "$out/nachher_${cm}cm/$name"; } \
      || failed=$((failed + 1))
  done
done

if is_ready "$s1"; then
  # ---- every measure off on its own, S1 1 cm
  for what in anlegen belichtung entrauschen verdeckung; do
    { surface_of "ohne_${what}_1cm" "$s1" 0.01 an --ohne "$what" \
      && images_of "$out/ohne_${what}_1cm/$n1"; } || failed=$((failed + 1))
  done

  # ---- on the CPU, S1 2 cm
  { surface_of cpu_2cm "$s1" 0.02 aus && images_of "$out/cpu_2cm/$n1"; } || failed=$((failed + 1))

  # ---- the program of 99284b9 against --ohne alles, S1 2 cm: same glb, byte for byte
  if [ -z "${DQ_PROBE_BIN:-}" ]; then
    probe="PROBE übersprungen: DQ_PROBE_BIN nicht gesetzt"
  else
    d=$out/probe_2cm/$n1
    mkdir -p "$d"
    if ! step "$d" surface 1 "$DQ_PROBE_BIN" surface "$s1" --slam "$base/$n1" --voxel 0.02 --gpu an -o "$d/oberflaeche"; then
      probe="PROBE fehlgeschlagen: siehe $d/surface.log"
      failed=$((failed + 1))
    elif cmp "$d/oberflaeche/oberflaeche.glb" "$out/vorher_2cm/$n1/oberflaeche/oberflaeche.glb"; then
      probe="PROBE gleich: oberflaeche.glb von 99284b9 und --ohne alles Byte für Byte"
    else
      probe="PROBE VERSCHIEDEN: oberflaeche.glb von 99284b9 und --ohne alles"
      failed=$((failed + 1))
    fi
  fi
  echo "$probe" | tee "$out/probe.txt"
fi

# ---- the tables, one per variant
: > "$out/tabelle.md"
for v in vorher_2cm nachher_2cm vorher_1cm nachher_1cm \
  ohne_anlegen_1cm ohne_belichtung_1cm ohne_entrauschen_1cm ohne_verdeckung_1cm cpu_2cm; do
  [ -d "$out/$v" ] || continue
  "$python" "$here/q_table.py" "$out/$v" | tee -a "$out/tabelle.md"
  echo | tee -a "$out/tabelle.md"
done
echo "Tabelle: $out/tabelle.md"
df -h "$HOME" | tail -1
echo "FERTIG $(date +%T), Schritte mit Fehler: $failed"
[ "$failed" -eq 0 ]
