"""완료된 기존/새 물리 실행의 진행·속도 그림. 각 실행의 원래 판정을 유지한다."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE=Path(__file__).resolve().parent
out=BASE/'course_comparison.png'
if out.exists():raise FileExistsError(out)
fig,axes=plt.subplots(2,1,figsize=(11,6.8),sharex=True,layout='constrained')
for run,label,color in ((BASE.parents[3]/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course','Previous run','#777777'),
                        (BASE/'run1_course','Explicit traffic recovery','#006c9e')):
    report=json.loads((run/'report.json').read_text())
    green=report['green_observed_sim_s'];stop=report['stop_request_sim_s']
    rows=[json.loads(line) for line in (run/'trajectory.jsonl').open()]
    rows=[row for row in rows if green<=row['t']<=stop]
    time=[r['t']-green for r in rows]
    progress=[r['progress_m']-rows[0]['progress_m'] for r in rows]
    axes[0].plot(time,progress,label=f"{label} (original verdict: {'PASS' if report['passed'] else 'FAIL'})",color=color)
    axes[1].plot(time,[r['ground_speed_mps'] for r in rows],label=label,color=color,lw=.9)
    if run==BASE/'run1_course':
        diag=json.loads((BASE/'run1_diagnostics.json').read_text())
        marks=[r['t_s']-green for r in diag['recovery_events']]
        axes[1].scatter(marks,[-.07]*len(marks),s=18,color='#bc4e00',label='Reobserved latch release (controller)')
for ax in axes:
    ax.grid(alpha=.25);ax.legend(loc='best',fontsize=8)
axes[0].set_ylabel('Course progress (m)')
axes[1].set_ylabel('Ground speed (m/s)')
axes[1].set_xlabel('Simulation time after green (s)')
axes[0].set_title('Same scripted traffic scenario; simulation ground truth used for evaluation only')
fig.savefig(out,dpi=160)
print(out)
