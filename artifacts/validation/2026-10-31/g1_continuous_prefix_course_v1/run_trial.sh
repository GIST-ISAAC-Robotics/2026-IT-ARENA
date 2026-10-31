#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1
test ! -e "$base/run1"
test ! -e "$base/run1_trial.log"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby|record_chase.py' | grep -v grep > "$base/run1_preexisting_processes.txt" || true
if [ -s "$base/run1_preexisting_processes.txt" ]; then echo 'preexisting simulation processes; not starting' > "$base/run1_ABORTED.txt"; exit 92; fi
python3 -B - <<'PYCODEC' > "$base/run1_codec.log" 2>&1
from pathlib import Path
import cv2,numpy as np
p=Path('artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1/codec_probe.mp4')
assert not p.exists()
w=cv2.VideoWriter(str(p),cv2.VideoWriter_fourcc(*'avc1'),20.,(960,540))
assert w.isOpened(),'H264 codec unavailable'
for _ in range(3):w.write(np.zeros((540,960,3),dtype=np.uint8))
w.release();c=cv2.VideoCapture(str(p));count=0
while True:
    ok,frame=c.read()
    if not ok:break
    count+=1
c.release();assert count==3
print('H264 encode/decode3 frames passed')
PYCODEC
python3 -B "$base/run.py" --output "$base/run1" --prepare-only > "$base/run1_prepare.log" 2>&1
python3 -B "$base/preflight.py" run1 > "$base/run1_preflight.log" 2>&1
set +e
python3 -B "$base/run.py" --output "$base/run1" > "$base/run1_trial.log" 2>&1
trial_exit=$?
if [ ! -f "$base/run1/report.json" ] || [ ! -f "$base/run1/follow_decisions.jsonl" ]; then
    printf 'setup_or_capture_failure=%s\n' "$trial_exit" > "$base/run1_exit.txt"
    ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby|record_chase.py' | grep -v grep > "$base/run1_residual_processes.txt" || true
    exit 1
fi
python3 -B scripts/replay_follow_trace.py "$base/run1/follow_decisions.jsonl" --output "$base/run1/follow_trace_replay.json" > "$base/run1_replay.log" 2>&1
replay_exit=$?
python3 -B "$base/evaluate.py" "$base/run1" --output "$base/run1_evaluation.json" > "$base/run1_evaluation.log" 2>&1
evaluation_exit=$?
python3 -B artifacts/validation/2026-10-28/bump_traffic_v1/analyze.py "$base/run1" --output "$base/run1_bump_analysis.json" > "$base/run1_bump_analysis.log" 2>&1
analysis_exit=$?
python3 -B "$base/progress_eval.py" "$base/run1" --output "$base/run1_progress.json" > "$base/run1_progress.log" 2>&1
progress_exit=$?
python3 -B scripts/retime_autonomy_video.py "$base/run1/video" > "$base/run1_video_timing.log" 2>&1
video_timing_exit=$?
python3 -B scripts/inspect_autonomy_video.py "$base/run1/video/lidar_third_person_simtime_1x.mp4" --output "$base/run1/video/inspection" > "$base/run1_video_inspection.log" 2>&1
video_inspection_exit=$?
printf 'timing=%s\ndecode=%s\n' "$video_timing_exit" "$video_inspection_exit" > "$base/run1_video_exit.txt"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby|record_chase.py' | grep -v grep > "$base/run1_residual_processes.txt" || true
printf 'original_g1_trial=%s\nreplay=%s\nhandoff_evaluation=%s\nbump_analysis=%s\nprogress=%s\n' "$trial_exit" "$replay_exit" "$evaluation_exit" "$analysis_exit" "$progress_exit" > "$base/run1_exit.txt"
if [ "$video_timing_exit" != 0 ] || [ "$video_inspection_exit" != 0 ]; then exit 1; fi
if [ "$progress_exit" != 0 ]; then exit "$progress_exit"; fi
exit "$evaluation_exit"
