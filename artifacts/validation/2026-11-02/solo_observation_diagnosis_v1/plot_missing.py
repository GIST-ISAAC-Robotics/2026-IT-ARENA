"""저장 무반환의 상대 각도와 당시 속도 상한. 평가용 그림."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
data=json.loads((HERE/'rays.json').read_text(encoding='utf-8'))
fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True,layout='constrained')
for category,color,label in [('beyond_range','#2171b5','Geometry: beyond 12 m'),
                             ('inside_range_hit','#d95f02','Geometry: within 12 m (approximation unresolved)')]:
    rays=[r for r in data['rays'] if r['unknown'] and r['class_name']==category and r['progress_m']>42]
    axes[0].scatter([r['progress_m'] for r in rays],[r['angle_deg'] for r in rays],s=30,c=color,label=label)
axes[0].axhspan(0,30,color='#efedf5',alpha=.7)
axes[0].axhline(30,color='#756bb1',linestyle='--',linewidth=1)
axes[0].text(42.04,24,'Current front classification: +/-30 degrees',fontsize=9)
axes[0].set(ylabel='Ray angle relative to vehicle (deg)',ylim=(0,100),title='The same long-straight direction moves from side to front during a turn')
axes[0].legend(fontsize=8,loc='upper right')
calls=[r for r in data['calls'] if r['progress_m']>42]
axes[1].plot([r['progress_m'] for r in calls],[r['speed'] for r in calls],color='#333333',label='Recorded planned speed')
limited=[r for r in calls if r['cap'] is not None]
axes[1].scatter([r['progress_m'] for r in limited],[r['cap'] for r in limited],s=8,color='#cb181d',label='Partial-observation speed cap')
axes[1].set(xlabel='Progress from start (m)',ylabel='Speed (m/s)',xlim=(42,44),ylim=(-.05,2.1))
axes[1].legend(fontsize=9)
for ax in axes:ax.grid(alpha=.2)
fig.savefig(HERE/'missing_direction_and_speed.png',dpi=160)
plt.close(fig)
