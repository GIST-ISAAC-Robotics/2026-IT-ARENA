"""Draw the run samples in course order. Speeds are Gazebo chassis measurements."""
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.ticker import MultipleLocator

here = Path(__file__).resolve().parent
font = FontProperties(fname='/mnt/c/Windows/Fonts/malgun.ttf')
fig, (ax, clock) = plt.subplots(2, 1, figsize=(9, 5.7), sharex=True,
                                gridspec_kw={'height_ratios': [2, 1]})
for folder, color, name in [('solo_1mps', '#70839a', '1.0 m/s'),
                            ('solo_1p5', '#1b7854', '1.5 m/s'),
                            ('solo_1p5_race_path', '#7356a0', '1.5 m/s + 경기 경로'),
                            ('solo_2p0_corrected', '#bc4c24', '2.0 m/s + 경기 경로')]:
    with (here / folder / 'trajectory.jsonl').open() as f:
        rows = [r for r in map(json.loads, f) if r['light'] == 'green']
    x = [r['progress_m'] for r in rows]
    v = [math.hypot(r['dynamics']['truth_longitudinal_mps'], r['dynamics']['truth_lateral_mps']) for r in rows]
    t = [r['time'] - rows[0]['time'] for r in rows]
    ax.plot(x, v, lw=1.35, color=color, label=name)
    clock.plot(x, t, lw=1.65, color=color)
    clock.scatter(x[-1], t[-1], s=18, color=color, zorder=3)
for a in (ax, clock):
    a.set_xlim(left=0)
    a.spines[['right', 'top']].set_visible(False)
    a.xaxis.set_major_locator(MultipleLocator(5))
    a.grid(axis='both', color='#e4e9ec', linewidth=.7)
    a.axvline(46.633, color='#60676e', linestyle=':', linewidth=1)
ax.set_ylim(bottom=0)
ax.set_ylabel('차체 속도 (m/s)', fontproperties=font)
ax.legend(prop=font, frameon=False, loc='upper left', ncol=4, bbox_to_anchor=(0,1.19))
clock.set_ylabel('출발 후 시간 (초)', fontproperties=font)
clock.set_xlabel('공식 코스 진행 거리 (m)   |   점선: 한 바퀴 46.63 m', fontproperties=font)
fig.suptitle('설정 속도를 높이면 실제로 어느 구간에서 빨라졌나', fontproperties=font, fontsize=16)
fig.subplots_adjust(hspace=.18, left=.09, right=.98, top=.83, bottom=.13)
fig.savefig(here/'solo_course_comparison.png', dpi=145, bbox_inches='tight', facecolor='white')
