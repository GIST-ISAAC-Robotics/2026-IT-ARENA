"""완료된 실제 두 실행의 녹색 이후 진행/차체 속도/계획 명령을 비교한다."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE=Path(__file__).resolve().parent
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
for folder,label,color in [(BASE.parent/'fast_lead_course_v1','Before: 1 mm rear limit','#d96a35'),
                           (BASE,'Candidate: bounded rear sweep','#247ba0')]:
    run=folder/'run1';report=json.loads((run/'report.json').read_bytes())
    green=report['case_window']['green_sim_s']
    rows=[json.loads(s) for s in (run/'trajectory.jsonl').read_text().splitlines()]
    rows=[r for r in rows if 0<=r['t']-green<=78]
    times=[r['t']-green for r in rows]
    axes[0].plot(times,[r['progress_m'] for r in rows],label=label,color=color)
    axes[1].plot(times,[r['ground_speed_mps'] for r in rows],color=color,lw=1.)
    axes[2].plot(times,[r['safe_speed_mps'] for r in rows],color=color,lw=1.)
axes[0].axhline(46.6329,color='gray',ls='--',lw=1.,label='One lap: 46.633 m')
axes[0].legend(loc='upper left');axes[0].set_ylabel('Course progress [m]')
axes[1].set_ylabel('Body speed [m/s]');axes[2].set_ylabel('Guarded command [m/s]')
axes[2].set_xlabel('Time since green [s]')
for ax in axes:
    ax.grid(alpha=.25);ax.set_xlim(0,78)
    ax.axvspan(21.428,31.363,color='#d96a35',alpha=.08)
fig.suptitle('Same 78 s traffic script, one run per setting\nEvaluation-only truth; scan phase and trajectory differ')
fig.savefig(BASE/'comparison.png',dpi=160)
plt.close(fig)
