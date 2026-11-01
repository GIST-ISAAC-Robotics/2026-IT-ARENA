"""교통 각본의 차체 속도·간격·진행과 실제 모드 전환을 한 시간축에 표시."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

BASE=Path(__file__).resolve().parent


if __name__=='__main__':
    report=json.loads((BASE/'run1/report.json').read_text())
    g=report['case_window']['green_sim_s']
    with (BASE/'run1/trajectory.jsonl').open() as f:
        rows=[r for r in map(json.loads,f) if r['elapsed_since_green_s'] is not None
              and 0<=r['elapsed_since_green_s']<=155]
    data=np.asarray([[r['elapsed_since_green_s'],r['ground_speed_mps'],r['drive_speed_mps'],
                      r['sampled_gap_m'],r['progress_m']] for r in rows])
    t,v,command,gap,progress=data.T
    events=[];last=None
    with (BASE/'run1/follow_decisions.jsonl').open() as f:
        for r in map(json.loads,f):
            if r['kind']!='call':continue
            d=r['data'];stamp=d['input']['reference_s']-g;status=d['output'][2]
            if 0<=stamp<=155 and status.get('handoff_state')!=last:
                last=status['handoff_state'];events.append((stamp,last))
    events.append((155,None))
    font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/malgun.ttf')
    plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False})
    fig,axes=plt.subplots(3,2,figsize=(11,7),layout='constrained',sharex='col')
    for j,span in enumerate(((0,25),(0,155))):
        axes[0,j].plot(t,command,color='#9fabb5',lw=.8,label='발행 명령')
        axes[0,j].plot(t,v,color='#057f85',lw=1.2,label='차체 속력')
        axes[0,j].plot([0,4,4,9,9,155],[.35,.35,0,0,.35,.35],color='#c17a3c',
                       ls='--',lw=1,label='앞차 각본')
        axes[1,j].plot(t,gap,color='#4d6173',lw=1.2)
        axes[1,j].axhline(.15,color='#a46658',ls=':',lw=1,label='이전 15 cm 판정')
        axes[2,j].plot(t,progress,color='#057f85',lw=1.2)
        for (start,state),(end,_) in zip(events,events[1:]):
            if state=='BYPASS':
                for ax in axes[:,j]:ax.axvspan(start,end,alpha=.15,color='#d39755')
        for ax in axes[:,j]:
            ax.set_xlim(*span);ax.set_ylim(bottom=0);ax.grid(alpha=.18)
            ax.spines[['top','right']].set_visible(False)
        axes[0,j].set_title('초기 정지차 우회·복귀' if j==0 else '155초 전체 각본')
        axes[2,j].set_xlabel('실제 녹색 이후 시뮬레이션 시간 (초)')
    for ax,label in zip(axes[:,0],('속도 (m/s)','앞차와의 최단 간격 (m)','공식 코스 진행 (m)')):ax.set_ylabel(label)
    axes[0,0].legend(loc='upper right',fontsize=8);axes[1,0].legend(fontsize=8)
    out=BASE/'course_progress.png'
    if out.exists():raise FileExistsError(out)
    fig.savefig(out,dpi=150);plt.close(fig)
