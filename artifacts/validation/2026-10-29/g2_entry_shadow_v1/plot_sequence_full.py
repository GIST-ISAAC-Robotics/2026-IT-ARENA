"""저장된 궤적 전체와 실제 평가 차체를 표시한다. 실행/센서/제어 변경 없음."""
import argparse,json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle

def body(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in ((-.1,-.075),(.1,-.075),(.1,.075),(-.1,.075))]

def main():
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);args=p.parse_args()
    output=args.folder/'entry_paths_full.png'
    if output.exists():raise ValueError('preserve existing figure')
    fig,axes=plt.subplots(4,1,figsize=(11,8),layout='constrained')
    for ax,path in zip(axes,sorted(args.folder.glob('y*.json'))):
        d=json.loads(path.read_text());r=d['records'];poses=[q['evaluation']['center_pose'] for q in r]
        x,y=zip(*[(q[0],q[1]) for q in poses]);end=max(1.6,max(x)+.2)
        ax.axhspan(-.425,-.225,color='#82af78',alpha=.35);ax.axhspan(.225,.425,color='#82af78',alpha=.35)
        for wall in (-.425,.425):ax.axhline(wall,color='#555555')
        ax.add_patch(Rectangle((1.1,d['target_y']-.075),.2,.15,fill=False,edgecolor='#a91f35',linewidth=2))
        if d['target_restarts']:
            target_x=1.2+.25*max(0,r[-1]['t']-1.5)
            ax.add_patch(Rectangle((target_x-.1,d['target_y']-.075),.2,.15,fill=False,edgecolor='#a91f35',linestyle='--'))
            end=max(end,target_x+.2)
        ax.plot(x,y,color='#236a9a',linewidth=1.5)
        for index in sorted(set([0,len(r)//3,2*len(r)//3,len(r)-1])):
            ax.add_patch(Polygon(body(*poses[index]),fill=False,edgecolor='#236a9a',alpha=.6))
        phases={}
        for row in r:
            if row['entry_phase'] in ('rejoin','centered'):phases.setdefault(row['entry_phase'],row)
        for label,row in phases.items():
            cx,cy,_=row['evaluation']['center_pose'];ax.scatter([cx],[cy],s=15)
            ax.annotate(label,(cx,cy),xytext=(3,14),textcoords='offset points',fontsize=8)
        ax.set(xlim=(-.15,end),ylim=(-.48,.48),ylabel='y [m]')
        ax.set_title(f"target y={d['target_y']:+.2f} m / restart={d['target_restarts']} / x={x[-1]:.3f} m / {d['sensor_profile']['profile']}",fontsize=9)
        ax.set_aspect('equal');ax.grid(alpha=.2)
    axes[-1].set_xlabel('Evaluation world x [m] (truth used only for plotting)')
    fig.suptitle('Sensor control and separate guard / ideal bicycle / not Gazebo or hardware',fontsize=11)
    fig.savefig(output,dpi=160);plt.close(fig);print(output)
if __name__=='__main__':main()
