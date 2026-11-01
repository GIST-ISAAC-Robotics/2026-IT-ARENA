"""실제 차체 속도·간격과 반환 근거를 같은 상대 시각에 표시한다."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE=Path(__file__).resolve().parent


def rows(folder):
    r=json.loads((folder/'report.json').read_bytes());g=r['case_window']['green_sim_s']
    with (folder/'trajectory.jsonl').open() as f:
        raw=[p for p in map(json.loads,f) if 0<=p['t']-g<=17]
    a=np.asarray([[p['t']-g,p['ground_speed_mps'],p['safe_speed_mps'],p['sampled_gap_m']] for p in raw])
    if not np.isfinite(a).all() or np.max(np.diff(a[:,0]))>.25:raise ValueError('표본 누락')
    return a


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('out',type=Path);ap.add_argument('runs',nargs='+',type=Path)
    args=ap.parse_args()
    if args.out.exists():raise FileExistsError(args.out)
    font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/malgun.ttf')
    plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False})
    fig,(v,gap)=plt.subplots(2,1,figsize=(9,5.5),sharex=True,layout='constrained')
    for i,folder in enumerate(args.runs):
        a=rows(folder);color=('#667080','#087f79')[i%2];label='기준' if i==0 else '최근 전진 근거'
        v.plot(a[:,0],a[:,1],label=label+' 차체',lw=1.5,color=color)
        if len(args.runs)==1:
            v.plot(a[:,0],a[:,2],label='보호층 통과 명령',color='#9c6d35',lw=.9,alpha=.8)
        gap.plot(a[:,0],a[:,3],label=label,lw=1.5,color=color)
    v.step([0,6,13,17],[.85,.55,.825,.825],where='post',color='#bb4b58',ls='--',label='앞차 각본')
    gap.axhline(1.2,color='#bb4b58',ls='--',lw=.9,label='최종 간격 기준 1.20 m')
    v.set(ylabel='속도 (m/s)',title='앞차가 감속·재가속할 때 자차의 실제 반응')
    gap.set(ylabel='차체 간격 (m)',xlabel='녹색 적용 후 시간 (초)',xlim=(0,17))
    for ax in (v,gap):
        ax.axvspan(6,13,color='#d4e2e4',alpha=.3);ax.grid(alpha=.2)
        ax.spines[['top','right']].set_visible(False);ax.legend(frameon=False,ncol=3,fontsize=9)
    fig.savefig(args.out,dpi=140);plt.close(fig)
