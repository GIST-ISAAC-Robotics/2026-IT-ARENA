"""녹색 이후 실제 차체 속도와 거리별 도착 시간; 자세는 평가에만 사용."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


def series(run):
    with (run/'trajectory.jsonl').open() as stream:
        green=[r for r in map(json.loads,stream) if r['light']=='green']
    x=np.array([r['progress_m'] for r in green])
    t=np.array([r['time']-green[0]['time'] for r in green])
    v=np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'], r['dynamics']['truth_lateral_mps']) for r in green])
    return x,t,v


def first_pass(x,t,goals):
    keep=np.r_[True,np.diff(np.maximum.accumulate(x))>1e-8]
    return np.interp(goals,x[keep],t[keep],left=np.nan,right=np.nan)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--ten',type=Path)
    args=parser.parse_args()
    here=Path(__file__).resolve().parent; ref=here.parent/'race_continuity_v1'
    rows=[(ref/'solo_1p5_race_path','1.5 기준','#465764'),(ref/'solo_2p0_corrected','2.0 변경 전','#cb7754'),
          (here/'solo_2p0','2.0 코너 접근 변경','#07876e')]
    if args.ten:rows.append((args.ten,'10 km/h 상한','#7852a0'))
    font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/malgun.ttf')
    plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False})
    fig,(a,b)=plt.subplots(2,1,figsize=(9.3,5.5),sharex=True,gridspec_kw=dict(height_ratios=(2,1)))
    goals=np.arange(.25,46.634,.25); base=first_pass(*series(rows[0][0])[:2],goals)
    for run,name,color in rows:
        x,t,v=series(run)
        a.plot(x,v,color=color,label=name,alpha=.9,lw=1.5)
        b.plot(goals,base-first_pass(x,t,goals),color=color,lw=1.5)
    for ax in (a,b):
        ax.grid(alpha=.18);ax.spines[['top','right']].set_visible(False)
        ax.axvline(28.88,ls=':',color='#7a7a7a');ax.axvline(46.633,ls=':',color='#7a7a7a')
    a.set(ylabel='차체 속도 (m/s)',ylim=(0,None));a.legend(ncol=len(rows),loc='lower center',bbox_to_anchor=(.5,1.01),frameon=False)
    b.set(ylabel='1.5 기준보다\n앞선 시간 (초)',xlabel='공식 코스 진행 거리 (m)',xlim=(0,48.8))
    b.axhline(0,color='#444444',lw=.6)
    fig.tight_layout()
    out=here/('lap_comparison_10kmh.png' if args.ten else 'lap_comparison.png')
    if out.exists():raise FileExistsError(out)
    fig.savefig(out,dpi=145)
