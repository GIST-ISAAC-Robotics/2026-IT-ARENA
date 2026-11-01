"""두 각본의 실제 차체 속도와 표본 간격을 비교한다. 상대 정답은 평가에만 쓰인다."""
import json
from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt

BASE=Path(__file__).resolve().parent
OLD=BASE.parent/'race_follow_pair_v1'

def series(folder):
    report=json.loads((folder/'report.json').read_text(encoding='utf-8'))
    end=report['case_window']['duration_s']
    with (folder/'trajectory.jsonl').open(encoding='utf-8') as f:
        rows=[r for r in map(json.loads,f) if r['elapsed_since_green_s'] is not None
              and 0<=r['elapsed_since_green_s']<=end]
    return np.array([[r['elapsed_since_green_s'],r['ground_speed_mps'],r['sampled_gap_m']] for r in rows]).T

def main():
    out=BASE/'follow_comparison.png'
    if out.exists():raise FileExistsError(out)
    fig,axes=plt.subplots(2,2,sharex='col',figsize=(10.5,5.7),layout='constrained')
    for col,(old,new,title) in enumerate([('slow_moving','slow','Lead continues at 0.35 m/s'),
                                          ('moving','stop','Lead stops, then moves again')]):
        for folder,label,color in [(OLD/old,'Match pace','#6b7280'),(BASE/new,'Match pace + close gap','#007f86')]:
            t,v,gap=series(folder)
            axes[0,col].plot(t,v,label=label,color=color,lw=1.7)
            axes[1,col].plot(t,gap,label=label,color=color,lw=1.7)
        end=12 if col==0 else 15
        x=[0,end] if col==0 else [0,4,4,9,9,end]
        lead=[.35,.35] if col==0 else [.35,.35,0.,0.,.35,.35]
        axes[0,col].plot(x,lead,color='#be6520',lw=1.3,ls='--',label='Scripted lead')
        if col==1:
            for ax in axes[:,col]:ax.axvspan(4,9,color='#feead6',alpha=.65,zorder=0)
        axes[0,col].set_title(title);axes[1,col].set_xlabel('Simulation time after green (s)')
        for ax in axes[:,col]:
            ax.set_xlim(0,end);ax.set_ylim(bottom=0);ax.grid(alpha=.20)
    axes[0,0].set_ylabel('Body speed (m/s)');axes[1,0].set_ylabel('Nearest body gap (m)')
    axes[0,0].legend(fontsize=8)
    fig.savefig(out,dpi=150);plt.close(fig)

if __name__=='__main__':main()
