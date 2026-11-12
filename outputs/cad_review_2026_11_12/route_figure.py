from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,FancyArrowPatch
OUT=Path(__file__).parent
s=json.loads((OUT/'sections.json').read_text(encoding='utf-8'))
plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
fig,(ax,bx)=plt.subplots(1,2,figsize=(16,10),gridspec_kw={'width_ratios':[1,1.25]})
ax.add_patch(Rectangle((-75,-96.2),150,199.7,fill=False,ec='#d35d4d',ls='--',lw=2))
for y,x,w,l,label,c in [(-70.5,-95.7,140,100,'J01','#afdec7'),(-51.5,12.1,103,90.75,'Jetson','#bfcdd9')]:
 ax.add_patch(Rectangle((y,x),w,l,fc=c,alpha=.75,ec='gray'));ax.text(y+w/2,x+l/2,label,ha='center',fontsize=16)
for d in s['151.5']:
 for line in d['lines']:
  p=np.array(line);ax.plot(p[:,1],p[:,0],c='#50555c',lw=.8)
for d in s['134.6']:
 for line in d['lines']:
  p=np.array(line);ax.plot(p[:,1],p[:,0],c='#c07822',lw=2)
path=np.array([[-85.8,34],[-72,34],[8.6,34],[8.6,8],[12.1,8]])
ax.plot(path[:,1],path[:,0],c='#087fac',lw=3,ls='--')
ax.scatter([34],[8.6],s=80,c='#087fac');ax.annotate('여기서 상판 위로\n유효 틈 약 6.8~7 mm',xy=(34,8.6),xytext=(85,3),arrowprops={'arrowstyle':'->'},fontsize=11)
ax.annotate('기둥의 앞/안쪽 면 →\n상판 아래를 따라 이동',xy=(34,-45),xytext=(82,-65),arrowprops={'arrowstyle':'->'},fontsize=11)
ax.annotate('J01 수평 XT30\n외측 여유 3.5 mm',xy=(-71.5,-75.3),xytext=(-105,-115),arrowprops={'arrowstyle':'->','color':'#c64439'},fontsize=11,color='#a43e36')
ax.set(xlim=(-120,143),ylim=(-135,118),xlabel='좌우 Y [mm] · +가 좌측',ylabel='전후 X [mm] · +가 전방')
ax.set_aspect('equal');ax.set_title('상판 평면 — 파선은 아래쪽 배선 경로 후보',fontsize=14);ax.grid(alpha=.15)

def block(x,z,w,h,label,c):
 bx.add_patch(Rectangle((x,z),w,h,facecolor=c,alpha=.7,edgecolor='#555',linewidth=.8));bx.text(x+w/2,z+h/2,label,ha='center',va='center',fontsize=10)
block(-82.5,67.7,52,34,'배터리','#dba79f')
block(-22.5,97.5,52,21.8,'MD20A','#8ecbbb')
block(47.6,104.8,55.6,41.3,'C1','#bcc5cc')
block(12.1,153.2,90.75,34.8,'Jetson','#bfcdd9')
block(-95.7,153.2,100,30.5,'J01','#afdec7')
block(-37.62,193.29,100,30.5,'M01','#afdec7')
bx.plot([-96.1,103.14],[150.21,150.21],c='#555',lw=3)
bx.plot([-91,-91],[7.2,150.21],c='#c07822',lw=6)
bx.axhline(134.6,c='#bc6a18',ls='--',lw=1);bx.text(-121,136,'라이다 광축 약 134.6 mm',fontsize=10,color='#a35a14')
bx.plot([-85.8,-85.8,8.6,8.6,12.1],[112,145,145,169,169],c='#087fac',lw=3)
bx.annotate('천장 아래, 중심 높이 약 145 mm\nØ6 가정: 142~148 mm 점유\n광축 위에서 가로로 이동',xy=(-45,145),xytext=(-116,234),arrowprops={'arrowstyle':'->'},fontsize=11,color='#12617e')
bx.annotate('Jetson–J01 사이 틈으로 상승\n프레임 끝 X=5.3 → 포트쪽 X 약 12.1',xy=(8.6,151),xytext=(20,242),arrowprops={'arrowstyle':'->'},fontsize=10)
bx.set(xlim=(-125,125),ylim=(0,270),xlabel='전후 X [mm] · +가 전방',ylabel='지상 높이 Z [mm]')
bx.set_aspect('equal');bx.grid(alpha=.15);bx.set_title('측면 — 아래 구간·플러그 삽입·굴곡은 추가 확인',fontsize=14)
fig.suptitle('기둥을 따라 올리고, 천장 아래로 돌아 내부 틈에서 상승하는 안',fontsize=17)
fig.text(.5,.025,'제안 경로이며 배선 완료 CAD가 아닙니다. 부품은 단순 외접 상자. 케이블 지름·플러그·타이 머리·실제 굴곡 여유를 넣어 확인해야 합니다.',ha='center',fontsize=10)
fig.tight_layout(rect=[0,.09,1,.96]);fig.savefig(OUT/'wiring_route_candidate.png',dpi=170)
