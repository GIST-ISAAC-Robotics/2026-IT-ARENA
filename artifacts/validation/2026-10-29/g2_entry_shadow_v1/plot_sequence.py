"""저장된 센서 진입 궤적만 그린다. 새 시뮬레이션을 실행하지 않는다."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    args=parser.parse_args()
    fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    for ax,path in zip(axes.flat,sorted(args.folder.glob('y*.json'))):
        result=json.loads(path.read_text(encoding='utf-8'))
        records=result['records']
        x=[r['evaluation']['center_pose'][0] for r in records]
        y=[r['evaluation']['center_pose'][1] for r in records]
        ax.axhspan(-.425,-.225,color='#82af78',alpha=.35)
        ax.axhspan(.225,.425,color='#82af78',alpha=.35)
        for wall in (-.425,.425):ax.axhline(wall,color='#555555')
        ax.add_patch(Rectangle((1.1,result['target_y']-.075),.2,.15,
                              fill=False,edgecolor='#a91f35',linewidth=2,label='target at start'))
        ax.plot(x,y,color='#236a9a',linewidth=2)
        ax.scatter([x[0],x[-1]],[y[0],y[-1]],c=['#2f8b49','#a91f35'],zorder=4)
        ax.set_title(f"y={result['target_y']:+.2f} m, restart={result['target_restarts']}\n"
                     f"progress={result['final_forward_progress_m']:.3f} m, side switches={result['side_selection_changes']}")
        ax.set_aspect('equal');ax.set_xlim(-.15,1.55);ax.set_ylim(-.48,.48)
        ax.set_xlabel('Evaluation world x [m]');ax.set_ylabel('Evaluation world y [m]');ax.grid(alpha=.2)
    fig.suptitle('Sensor-based short entry / ideal bicycle — not a completed bypass')
    output=args.folder/'entry_paths.png'
    if output.exists():raise ValueError('existing plot must be preserved')
    fig.savefig(output,dpi=150);plt.close(fig)
    print(output)


if __name__=='__main__':main()
