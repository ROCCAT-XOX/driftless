#!/bin/bash
# Surface images and hole figures of one run:  q_images.sh <run_dir>
#   <run_dir> = <label>/<scan> with traj.tum, color.las and oberflaeche/ (after slam, map, surface).
# Writes <run_dir>/bilder/*.png (stay on the VM) and <run_dir>/images.log with the lines
#   LOECHER <view> <percent>   (share of cloud pixels without surface; views oben, rundblick_000..315
#                               and, if <label>/canon.json exists, the fixed views of the black room)
# Env: DQ_PYTHON  python with numpy (for q_views.py; default python3)
set -uo pipefail
[ $# -eq 1 ] || { sed -n 2,3p "$0"; exit 1; }
d=$(cd "$1" && pwd)
scan=$(basename "$d")
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd "$here/../.." && pwd)
python=${DQ_PYTHON:-python3}

newest_test() { ls -t "$repo"/target/release/deps/quality_real-* 2> /dev/null | grep -v '\.d$' | head -1; }
bin=$(newest_test)
if [ -z "$bin" ]; then
  (cd "$repo" && cargo test --release -p driftless-steps --test quality_real --no-run) || exit 1
  bin=$(newest_test)
fi

views=""
case $scan in
  2026102213*) [ -f "$d/../canon.json" ] && views=$("$python" "$here/q_views.py" "$scan" "$d/../canon.json") ;;
esac

QA_A_SURF=$d/oberflaeche QA_A_CLOUD=$d/color.las QA_TRAJ=$d/traj.tum \
QA_OUT=$d/bilder QA_PREFIX="${scan}_" QA_VIEWS="$views" \
RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-6} OMP_NUM_THREADS=6 \
  nice -n 10 /usr/bin/time -v "$bin" --ignored --nocapture --test-threads 1 > "$d/images.log" 2>&1
rc=$?
echo "   images rc=$rc"
grep -E "^LOECHER|Elapsed|Maximum res" "$d/images.log"
exit $rc
