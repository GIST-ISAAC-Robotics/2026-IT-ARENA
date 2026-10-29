"""완료된 측방 범위 후보와 직전 두 실패를 원래 판정으로 대조한다."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
out = BASE/'course_comparison.png'
if out.exists():
    raise FileExistsError(out)
fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, layout='constrained')
runs = [
    (BASE.parent/'traffic_speed_contract_v1/run1_course', 'Speed contract: retained obstacle', '#888888'),
    (BASE.parent/'traffic_positive_clearance_v1/run1_course', 'Old location check: side scope rejected', '#cc8000'),
    (BASE/'run1_course', 'Consistent same-path side scope', '#008050'),
]
for run, label, color in runs:
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    green, stop = report['green_observed_sim_s'], report['stop_request_sim_s']
    with (run/'trajectory.jsonl').open(encoding='utf-8') as stream:
        rows = [json.loads(line) for line in stream]
    rows = [row for row in rows if green <= row['t'] <= stop]
    time = [r['t']-green for r in rows]
    axes[0].plot(time, [r['progress_m']-rows[0]['progress_m'] for r in rows], color=color,
                 label=f"{label}: {'PASS' if report['passed'] else 'FAIL'} (original verdict)")
    axes[1].plot(time, [r['ground_speed_mps'] for r in rows], color=color, label=label, lw=.8)
for ax in axes:
    ax.grid(alpha=.25)
    ax.legend(loc='best', fontsize=8)
axes[0].set_title('Same scripted traffic; simulation truth used only for evaluation')
axes[0].set_ylabel('Course progress (m)')
axes[1].set_ylabel('Ground speed (m/s)')
axes[1].set_xlabel('Simulation time after green (s)')
fig.savefig(out, dpi=160)
print(out)
