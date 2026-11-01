set +u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-01/follow_continuity_v1'
python3 -B compare_runs.py fast_stop_closing fast_stop_low_motion fast_stop_compare.json --base-surface fast_stop_closing_surface.json --new-surface fast_stop_low_motion_surface.json
python3 -B timeline.py fast_stop_closing ~/arena_scratch/fc1/fs_closing.csv
python3 -B timeline.py fast_stop_low_motion ~/arena_scratch/fc1/fs_low.csv
echo == closing; python3 -B state_table.py fs_closing 2.5 9.85 | tail -14
echo == low; python3 -B state_table.py fs_low 2.5 9.85 | tail -16
