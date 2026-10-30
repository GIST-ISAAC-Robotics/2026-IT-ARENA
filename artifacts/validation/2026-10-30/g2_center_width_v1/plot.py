"""저장 관측점의 필요조건 진단을 표시한다. 실제 차량 자세 그림은 아니다."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

BASE=Path(__file__).resolve().parent
report=json.loads((BASE/'analysis1.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(12,5),sharex=True,sharey=True,layout='constrained')
for ax,at in zip(axes,(3.2,3.28)):
    row=next(x for x in report['results'] if abs(x['reference_s']-at)<1e-8)
    for (x,y),p in zip(row['points_wall_frame'],row['padding_m']):
        if -.2<x<1.1 and -.3<y<.6:
            ax.add_patch(Rectangle((x-p,y-p),2*p,2*p,color='#ef806e',alpha=.10,lw=0))
            ax.plot(x,y,'.',color='#292f36',ms=3)
    for lo,hi in row['fixed_parallel_band_0_to_09m']:
        ax.axhspan(lo,hi,color='#008f73',alpha=.5,label='Point-only center band')
    for y,color in ((.24,'#295cba'),(.27,'#a25f00')):
        ax.axhline(y,color=color,ls='--',lw=1.4,label=f'Registered offset {y:.2f} m')
    ax.add_patch(Rectangle((.30-.10,.26-.075),.20,.15,fill=False,ec='#7d219a',lw=2,
        label='Illustrative parallel car at offset 0.26 m'))
    ax.set_title(f'Stored observation at {at:.2f} s\n'+('Nonempty point-only band' if row['fixed_parallel_band_0_to_09m'] else 'No common parallel band'))
    ax.set(xlim=(-.05,.95),ylim=(-.18,.52),xlabel='Forward along fitted wall [m]',ylabel='Offset from assumed wall reference [m]')
    ax.set_aspect('equal');ax.grid(alpha=.18)
handles,labels=axes[0].get_legend_handles_labels()
fig.legend(handles,labels,loc='lower center',bbox_to_anchor=(.5,-.12),ncol=2,fontsize=9)
fig.suptitle('Stored returns and age-dependent envelopes: geometry diagnosis only\nNot the actual ego pose; no unseen-free-space or transition authorization',fontsize=11)
fig.savefig(BASE/'stored_point_bands.png',dpi=170,bbox_inches='tight')
