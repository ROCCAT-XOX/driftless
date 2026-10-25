#!/bin/bash
# Template scans end to end on VM 101: slam, map, surface, then metrics and the table.
#   run_vorlagen.sh <label> [--map-voxel 0.005] [--surface-voxel 0.01]
# Results in ${DQ_ROOT:-~/td/quality}/<label>/<scan>/ and the table in .../<label>/tabelle.md.
# Env: DQ_SCANS  space-separated scan folders (default: the four template scans on the VM)
#      DQ_ROOT   result root (default ~/td/quality)
#      DQ_PYTHON python with numpy, scipy, laspy (default python3; on the VM ~/td/quality-venv/bin/python)
# Start with `driftless-vm run tools/qualitaet/run_vorlagen.sh <label>` (half load, nice 10, one run at a time).
set -uo pipefail

usage() { sed -n 2,9p "$0"; exit 1; }
[ $# -ge 1 ] || usage
label=$1; shift
map_voxel=0.005
surface_voxel=0.01
while [ $# -gt 0 ]; do
  case $1 in
    --map-voxel) map_voxel=${2:?}; shift 2 ;;
    --surface-voxel) surface_voxel=${2:?}; shift 2 ;;
    *) usage ;;
  esac
done

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
python=${DQ_PYTHON:-python3}
root=${DQ_ROOT:-$HOME/td/quality}
default_scans="$HOME/td/jmk6-2026-10-22/20261022133554 $HOME/td/jmk6-2026-10-22/20261022132520 $HOME/td/jmk6-2026-10-22/20261022133028 $HOME/td/jmk6-2026-10-20/20261020173822"
read -r -a scans <<< "${DQ_SCANS:-$default_scans}"
out=$root/$label

command -v driftless > /dev/null || { echo "driftless nicht im PATH (cargo build --release -p driftless-cli)" >&2; exit 1; }
command -v gpu-job > /dev/null || { echo "gpu-job fehlt (GPU-Schritte laufen über die Warteschlange)" >&2; exit 1; }
export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-6} OMP_NUM_THREADS=6 OPENBLAS_NUM_THREADS=6 MKL_NUM_THREADS=6

# step <dir> <name> <gpu 0|1> <command…>: log with /usr/bin/time -v to <dir>/<name>.log
step() {
  local dir=$1 name=$2 gpu=$3; shift 3
  echo "== $(date +%T) $(basename "$dir") $name  load $(cut -d' ' -f1-3 /proc/loadavg)  free $(free -g | awk '/Mem/{print $7}')G"
  if [ "$gpu" = 1 ]; then
    gpu-job --name "dq-$name" -- nice -n 10 /usr/bin/time -v "$@" > "$dir/$name.log" 2>&1
  else
    nice -n 10 /usr/bin/time -v "$@" > "$dir/$name.log" 2>&1
  fi
  local rc=$?
  echo "   rc=$rc $(grep -E 'Elapsed' "$dir/$name.log" | sed 's/.*: //')  maxrss $(grep 'Maximum resident' "$dir/$name.log" | awk '{printf "%.2f GB", $NF/1048576}')"
  return $rc
}

mkdir -p "$out"
names=()
for scan in "${scans[@]}"; do
  [ -d "$scan" ] || { echo "Scan fehlt: $scan" >&2; continue; }
  name=$(basename "$scan")
  d=$out/$name
  mkdir -p "$d"
  step "$d" slam 0 driftless slam "$scan" -o "$d" || continue
  step "$d" map 1 driftless map "$scan" --slam "$d" --voxel "$map_voxel" --gpu an || continue
  step "$d" surface 1 driftless surface "$scan" --slam "$d" --voxel "$surface_voxel" --gpu an -o "$d/oberflaeche" || continue
  step "$d" analyze 0 "$python" "$here/q_analyze.py" variant "$d" || continue
  names+=("$name")
done

[ ${#names[@]} -gt 0 ] || { echo "kein Scan fertig geworden" >&2; exit 1; }
"$python" "$here/q_table.py" "$out" "${names[@]}" | tee "$out/tabelle.md"
echo "Tabelle: $out/tabelle.md"
df -h "$HOME" | tail -1
