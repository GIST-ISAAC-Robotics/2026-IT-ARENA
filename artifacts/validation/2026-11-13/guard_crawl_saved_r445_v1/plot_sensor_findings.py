"""기록된 센서 전용 기하를 그림으로 표시한다. 객체 분류나 제어 계산 없음."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

B = Path(__file__).resolve().parent / 'sensor_only_v1'
g = json.loads((B / 'seq998.json').read_text(encoding='utf8'))
p = np.asarray(g['returns_xy']); path = np.asarray(g['baseline_path'])
fig, axes = plt.subplots(1, 2, figsize=(11, 6), gridspec_kw={'width_ratios':[1.4, 1]})
for ax in axes:
    ax.scatter(p[:,0],p[:,1],s=8,color='#8090a2',label='Finite returns')
    ax.plot(path[:,0],path[:,1],color='#22814d',lw=1.5,label='Recorded path')
    for side, color in [('left','#ce3b44'),('right','#2869c7')]:
        points=np.asarray([t['xy'] for t in g['threat_points'][side]])
        ax.scatter(points[:,0],points[:,1],s=36,color=color,label=side+' selected threat returns',zorder=5)
    ax.set_xlabel('Vehicle x / forward (m)');ax.set_ylabel('Vehicle y / left (m)')
    ax.grid(alpha=.2);ax.set_aspect('equal')
axes[0].add_patch(Rectangle((-.1035,-.075),.207,.15,facecolor='#d3d8df',edgecolor='#555',label='Body envelope'))
axes[0].set_xlim(-.4,1.1);axes[0].set_ylim(-.55,1.3)
axes[0].set_title('seq 998: sensor-derived geometry, reference 44.42 s')
axes[0].legend(loc='upper right',fontsize=7)
axes[0].annotate('Left: 3 close returns\nAdjacent rays reach\n2.95 / 3.57 m',
    xy=(.074,.434),xytext=(-.33,.98),fontsize=8,arrowprops={'arrowstyle':'->','color':'#ce3b44'})
axes[1].set_xlim(.02,.12);axes[1].set_ylim(-.47,-.26)
axes[1].set_title('Right foreground and adjacent returns')
axes[1].annotate('4 selected rays\ny ~ -0.296 m',xy=(.064,-.296),xytext=(.027,-.275),fontsize=8,
    arrowprops={'arrowstyle':'->','color':'#2869c7'})
axes[1].annotate('~0.140 m depth separation',xy=(.08,-.435),xytext=(.025,-.37),fontsize=8,
    arrowprops={'arrowstyle':'->','color':'#555'})
fig.suptitle('No actor truth, object labels, or occupancy removal used',fontsize=11)
fig.tight_layout();fig.savefig(B/'seq998_sensor_geometry.png',dpi=180);plt.close(fig)
