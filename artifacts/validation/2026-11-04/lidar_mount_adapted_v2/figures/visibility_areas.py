"""차체에 고정한 영역에서의 가림 비율(10 mm 격자). 감지 확률 아님."""
from pathlib import Path
import math,json
here=Path(__file__).resolve().parent
def counts(sx,front,rear,ry,x0,x1,y0,y1,bilateral):
 posts=[(x,sg*y) for x,y in [(front,62),(rear,ry)] for sg in [-1,1]]; counts=[0,0,0]
 for x in range(x0+5,x1,10):
  for y0pos in range(y0+5,y1,10):
   for sign in ([-1,1] if bilateral else [1]):
    y=y0pos*sign;d=math.hypot(x-sx,y);theta=math.atan2(y,x-sx);occluded=False
    for px,py in posts:
     r=math.hypot(px-sx,py);a=math.atan2(py,px-sx);diff=abs(math.remainder(theta-a,2*math.pi))
     if d>=r and diff<=math.asin(3.5/r)+math.radians(.5):occluded=True;break
    marker=math.atan2(25,sx+100.5)+math.radians(.5)
    if x<=-100.5 and abs(math.remainder(theta-math.pi,2*math.pi))<=marker:occluded=True
    counts[0]+=1;counts[1]+=occluded;counts[2]+=d<50
 return dict(points=counts[0],fixture_hidden=counts[1],fixture_percent=round(100*counts[1]/counts[0],3),inside_min_range=counts[2])
regions={'forward_diagonal':(120,720,100,400,True),'side_strip':(-80,80,100,300,True),'front_bumper_strip':(120,1120,-75,75,False)}
out={n:{k:counts(*cfg,*reg) for k,reg in regions.items()} for n,cfg in {'C30':(30,80,-80,62),'C65':(65,80,-80,62),'JR50':(50,10,-80,52)}.items()}
(here/'visibility_areas.json').write_text(json.dumps({'regions_mm':regions,'sample_cell_mm':10,'layouts':out,'scope':'차체 기준 격자 중심. 이상적 2D 지주와 뒤 마커 및 구간 여유. 면적비는 사고·검출 확률이 아니다.'},ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(out)
