from compare_step import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from matplotlib import font_manager
fp=Path('C:/Windows/Fonts/malgun.ttf')
font_manager.fontManager.addfont(fp)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=fp).get_name(),'axes.unicode_minus':False})
fig,axs=plt.subplots(1,2,figsize=(14,5.25),sharey=True,dpi=160)
fig.patch.set_facecolor('#f2f5f9')
fig.subplots_adjust(left=.078,right=.986,top=.83,bottom=.18,wspace=.045)
for ax,v in zip(axs,('old','new')):
    ax.imshow(plt.imread(OUT/f'c1_{v}_right.png'),extent=(75.4-36*850/560,75.4+36*850/560,90,162), interpolation='bilinear')
    ax.axhspan(131.6,137.6,color='#40efdd',alpha=.13)
    ax.axhline(134.6,ls='--',color='#5cf9e5',lw=1.65)
    ax.axhline(104.8,ls=':',color='#ffcf73',lw=1.35)
    ax.set(xlim=(26,120),ylim=(96,157),xlabel='차량 X (mm)  ·  → 앞')
    ax.set_title('11/13 이전 확정본' if v=='old' else '11/14 새 최종본',loc='left',fontweight='bold',fontsize=12)
    ax.xaxis.set_major_locator(MultipleLocator(20));ax.yaxis.set_major_locator(MultipleLocator(10))
    if v=='new':
        ax.text(118.5,135.4,'스캔 공칭선 134.6 mm',fontsize=9,ha='right',va='bottom',
                color='#b6fff6',bbox=dict(boxstyle='round,pad=.28',fc='#172535',ec='none'))
        ax.annotate('하단 받침의 최고점 = 장착면 104.8 mm',(95,104.8),(117,98.6),color='#ffdc97',fontsize=8,
                    ha='right',arrowprops=dict(arrowstyle='->',color='#ffdc97'))
axs[0].set_ylabel('지면 기준 높이 Z (mm)')
fig.text(.078,.94,'C1 스캔 높이와 장착부 | 차량 우측에서 본 두 STEP',color='#142238',fontsize=17,fontweight='bold')
fig.text(.078,.063,'청록 띠: 131.6~137.6 mm 단면 검사 범위. 제조사 공차나 실차 스캔 높이의 보증 범위가 아닙니다.',fontsize=10,color='#34455c')
fig.savefig(OUT/'c1_scan_height_comparison.png')

dat=get(OUT,'section_comparison.json')
fig,ax=plt.subplots(figsize=(9,5),dpi=160)
fig.patch.set_facecolor('#f2f5f9')
ax.set_facecolor('#fbfdff')
for k,color,style,alpha in [('old','#ee895c','-',.9),('new','#007d91','--',1.)]:
    for i,e in enumerate(dat[k]['134.6']):
        for j,l in enumerate(e['lines']):
            pts=np.array(l)
            ax.plot(*pts.T,color=color,ls=style,alpha=alpha,lw=2,
                    label={'old':'11/13 이전 단면','new':'11/14 새 단면'}[k] if i==j==0 else None)
ax.scatter(75.4,0,marker='+',c='#263b52',s=130,zorder=4,label='C1 회전 중심')
ax.annotate('전방', (105.,0), (82.,0),va='center',color='#263b52',arrowprops=dict(arrowstyle='->',color='#263b52'))
ax.set(xlim=(-113,120),ylim=(-76,77),aspect='equal',xlabel='차량 X (mm)',ylabel='차량 Y (mm) · 위가 좌측',title='Z = 134.6 mm의 차체 단면 — 배선·마커 제외')
ax.grid(alpha=.15);ax.legend(loc='upper left',framealpha=.97)
fig.tight_layout()
fig.savefig(OUT/'c1_scan_section_comparison.png')

