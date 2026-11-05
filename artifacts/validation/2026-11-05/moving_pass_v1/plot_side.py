"""두 주행의 평가 전용 자세로 앞섬·차로 복귀·진행을 시간축에 비교한다."""
from pathlib import Path
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from review import lines,read,at

HERE=Path(__file__).resolve().parent
def trajectory(folder):
    d=HERE/folder/'run1'
    window=read(d/'report.json')['case_window'];green,end=window['green_sim_s'],window['end_sim_s']
    ego=list(lines(d/'ego_poses.jsonl')); actor=list(lines(d/'actor_poses.jsonl'))
    et=[p[0] for p in ego];act=[p[0] for p in actor]
    x,y=at(actor,act,green);u,v=at(actor,act,end);angle=math.atan2(v-y,u-x)
    c,s=math.cos(angle),math.sin(angle)
    out=[]
    for i in range(round(20*(end-green))+1):
        t=green+i*.05;x,y=at(ego,et,t);u,v=at(actor,act,t)
        out.append((t-green,(x-u)*c+(y-v)*s,-(x-u)*s+(y-v)*c-.12))
    return list(zip(*out))

if __name__=='__main__':
    fig,axes=plt.subplots(2,1,figsize=(8,5.2),sharex=True,constrained_layout=True)
    for label,folder,color in [('Follow only','closed_loop_side_follow_1','#737a85'),
            ('Pass r11','closed_loop_side_pass_1','#bb4b1a'),('Pass r12','closed_loop_side_pass_2','#007788')]:
        times,long,lat=trajectory(folder)
        for ax,ys in zip(axes,[long,lat]):ax.plot(times,ys,label=label,color=color,lw=2)
    for ax in axes:
        ax.axhline(0,color='#444',lw=.7)
        ax.grid(axis='y',alpha=.18);ax.spines[['right','top']].set_visible(False)
    axes[0].axhspan(-.1,.1,color='#b9bdc3',alpha=.2);axes[1].axhspan(-.06,.06,color='#b9bdc3',alpha=.23)
    axes[0].set_ylabel('Ahead of opponent (m)')
    axes[1].set_ylabel('Left of start straight (m)');axes[1].set_xlabel('Seconds after green')
    axes[1].axvline(12.216,color='#007788',ls=':',lw=1)
    axes[1].text(12.28,.30,'r12 into\nfirst turn',fontsize=8,color='#006477')
    axes[0].set_title('Moving pass on the official straight  |  opponent at 0.30 m/s, 12 cm right',loc='left',fontsize=12)
    axes[0].legend(loc='upper left',fontsize=8);axes[1].set_xlim(0,14)
    path=HERE/'side_compare.png'
    fig.savefig(path,dpi=160)
    plt.close(fig)
