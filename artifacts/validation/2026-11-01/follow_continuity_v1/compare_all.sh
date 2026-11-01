set +u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-01/follow_continuity_v1'
L=../rolling_lead_v1
python3 -B compare_runs.py $L/run1 $L/curve_low_motion curve_compare.json --base-surface $L/run1_surface.json --new-surface curve_surface.json
python3 -B compare_runs.py ../race_follow_closing_v1/stop $L/stop_low_motion stop_compare.json --base-surface ../race_follow_closing_v1/stop_surface.json --new-surface stop_surface.json
python3 -B hold_slip.py hold_slip_candidate.json $L/curve_low_motion $L/stop_low_motion
python3 -B timeline.py $L/curve_low_motion ~/arena_scratch/fc1/curve.csv
python3 -B timeline.py $L/stop_low_motion ~/arena_scratch/fc1/stop_lm.csv
echo == curve; python3 -B episodes.py curve 17
echo == stop; python3 -B episodes.py stop_lm 15
