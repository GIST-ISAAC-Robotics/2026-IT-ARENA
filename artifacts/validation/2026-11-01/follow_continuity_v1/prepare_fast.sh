set +u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
B=artifacts/validation/2026-11-01/follow_continuity_v1
python3 -B - <<'PY'
import sys
sys.path.insert(0,'scripts')
from pathlib import Path
from validate_g1_traffic import load_config,load_centerline,check_straight,actor_start_s
cfg=load_config(Path('artifacts/validation/2026-11-01/follow_continuity_v1/fast_stop.json'))
line,scene=load_centerline()
case=cfg['cases'][0]
print(check_straight(cfg,line,case,actor_start_s(cfg,scene,0,case)))
PY
python3 -B "$B/run_fast_stop.py" closing --prepare-only && python3 -B "$B/run_fast_stop.py" low_motion --prepare-only && echo PREPARED
