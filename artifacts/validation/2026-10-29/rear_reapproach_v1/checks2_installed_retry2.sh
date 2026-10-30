#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/rear_reapproach_v1
test ! -e "$base/checks2_installed_retry2.log"
set +e
env -u PYTHONPATH bash <<'INNER' > "$base/checks2_installed_retry2.log" 2>&1
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 -B - <<'PY'
import arena_autonomy.local_follow as lf
import arena_autonomy.rear_lane_keep_trial as rl
import arena_autonomy.local_pursuit as lp
import hashlib, pathlib, json
src=pathlib.Path('src/arena_autonomy/arena_autonomy')
mods={'local_follow':lf,'rear_lane_keep_trial':rl,'local_pursuit':lp}
same={n:hashlib.sha256(pathlib.Path(m.__file__).read_bytes()).hexdigest()==hashlib.sha256((src/(n+'.py')).read_bytes()).hexdigest() for n,m in mods.items()}
files={n:str(pathlib.Path(m.__file__)) for n,m in mods.items()}
resolved={n:str(pathlib.Path(m.__file__).resolve()) for n,m in mods.items()}
print(json.dumps({'installed_matches_source':same,'default_off':lf.FollowConfig().rear_lane_keep_trial is False,'files':files,'resolved_files':resolved,'all_in_install':all('/install/' in f for f in files.values()),'any_module_file_symlink':any(pathlib.Path(m.__file__).is_symlink() for m in mods.values())}))
PY
INNER
result=$?
printf '%s\n' "$result" > "$base/checks2_installed_retry2.exit"
exit "$result"