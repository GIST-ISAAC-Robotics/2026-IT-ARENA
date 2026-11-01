# 작업 트리를 건드리지 않고 임시 복사본에 수정 전 파일을 복원해 같은 시험을 돌린다.
set +u
SRC='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
W="$HOME/arena_scratch/fc1_before"
B=artifacts/validation/2026-11-01/follow_continuity_v1
rm -rf "$W"; mkdir -p "$W"
cd "$SRC"
for d in src scripts tests config; do cp -r "$d" "$W/$d"; done
for f in *; do [ -e "$W/$f" ] || ln -s "$SRC/$f" "$W/$f"; done
cd "$W"
python3 - "$SRC/$B/before.zip" <<'PY'
import sys,zipfile
from pathlib import Path
with zipfile.ZipFile(sys.argv[1]) as z:
    for n in z.namelist():
        Path(n).write_bytes(z.read(n))
Path('config/tests/follow_continuity_r1_sha256.json').unlink()
Path('tests/test_low_motion_reserve.py').unlink()
p=Path('tests/test_source_provenance_r0_plus_b3.py')
d=p.read_bytes();old=b", 'config/tests/follow_continuity_r1_sha256.json')";assert d.count(old)==1
p.write_bytes(d.replace(old,b')'))
PY
source /opt/ros/jazzy/setup.bash
source "$SRC/install/local_setup.bash"
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$W/src/arena_autonomy:$W/src/arena_vehicle_interface:$W/scripts:$W/tests${PYTHONPATH:+:$PYTHONPATH}"
grep -E "^FAILED " "$SRC/$B/pytest_full.log" | sed -E 's/^FAILED ([^ ]+).*/\1/' > /tmp/fc1_failed.txt
python3 -B -m pytest -q -p no:cacheprovider $(cat /tmp/fc1_failed.txt) > "$SRC/$B/pytest_failed_on_before.log" 2>&1
printf 'ACTUAL_EXIT_CODE=%s\n' "$?" >> "$SRC/$B/pytest_failed_on_before.log"
tail -22 "$SRC/$B/pytest_failed_on_before.log" | cut -c1-130
