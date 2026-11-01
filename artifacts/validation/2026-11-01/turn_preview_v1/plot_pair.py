"""동일 시설의 첫 랩만: 거리별 실제 차체 속도·첫 통과 시각."""
from pathlib import Path
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'corner_approach_v1'))
from plot_speed import series


def crossings(x,t,goals):
    # 처음 도착한 실제 인접 표본 사이에서만 보간. 중간에 후퇴한 시간을 단축하지 않는다.
    out=[]
    i=0
    for d in goals:
        while i+1<len(x) and not (x[i]<d<=x[i+1]):i+=1
        if i+1==len(x):raise ValueError('등록 거리 도달 기록 없음')
        out.append(t[i]+(d-x[i])/(x[i+1]-x[i])*(t[i+1]-t[i]))
    return np.asarray(out)


if __name__=='__main__':
    comp=json.loads((HERE/'run4_lap_compare.json').read_bytes());length=comp['prior']['length_m']
    goals=np.linspace(.05,length,240);results=[]
    font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/malgun.ttf')
    plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False})
    fig,(a,b)=plt.subplots(2,1,figsize=(9,5.8),sharex=True,layout='constrained',gridspec_kw={'height_ratios':[2,1]})
    for folder,name,color in [(HERE.parent/'official_prelap_v1/run2','기준 · 47.14초','#6c7987'),
                              (HERE/'run4','코너 보완 · 44.28초','#067d78')]:
        x,t,v=series(folder);first=crossings(x,t,goals);results.append(first)
        valid=t<=first[-1]
        a.plot(x[valid],v[valid],label=name,color=color,lw=1.05,alpha=.9)
    a.set(ylabel='차체 속도 (m/s)',title='같은 2 m/s 상한에서 코너 감속을 줄인 한 랩');a.legend(frameon=False,ncol=2)
    a.axhline(2,ls=':',color='#b9742c',lw=.8)
    lead=results[0]-results[1];b.plot(goals,lead,color='#076d79');b.fill_between(goals,0,lead,color='#69adba',alpha=.3)
    b.set(xlabel='공식 중심선 진행 거리 (m)',ylabel='기준보다 앞선 시간 (초)',xlim=(0,length))
    for ax in (a,b):
        ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
    b.annotate(f'{lead[-1]:.2f}초',xy=(goals[-1],lead[-1]),xytext=(-35,-13),textcoords='offset points')
    out=HERE/'lap_comparison.png'
    if out.exists():raise FileExistsError(out)
    fig.savefig(out,dpi=135);plt.close(fig)
