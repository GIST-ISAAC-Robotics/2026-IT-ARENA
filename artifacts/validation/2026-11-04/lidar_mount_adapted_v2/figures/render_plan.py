"""상면 한 장: C 센서 전진과 JR 지주 이동 차이. 평면 기하만 표시한다."""
from pathlib import Path
from functools import lru_cache
import json, math, os
from PIL import Image, ImageDraw, ImageFont

here=Path(__file__).resolve().parent
data={r['name']:r for r in json.loads((here.parent/'geometry.json').read_text(encoding='utf8'))['rows']}
W,H=1500,821
im=Image.new('RGB',(W,H),'#f4f5ef'); d=ImageDraw.Draw(im)
@lru_cache()
def ft(s,b=False):return ImageFont.truetype(str(Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts'/('malgunbd.ttf' if b else 'malgun.ttf')),s)
def label(x,y,v,s=24,c='#142f36',b=False):d.text((x,y),v,font=ft(s,b),fill=c,anchor='lt')
label(40,28,'앞 기둥은 그대로, 라이다만 옮기면 어떻게 달라질까?',39,b=True)
label(42,92,'동일한 200 × 150 mm 외곽 · 광축 높이 75 mm · Ø7 mm 지주 · 각 가림의 양 끝에 0.5° 여유',23,'#43585a')
for i,key in enumerate(['C30','C65','JR50']):
 x0=38+i*480; y0=148; wide=461
 d.rounded_rectangle((x0,y0,x0+wide,y0+542),radius=14,fill='white',outline='#cad2c9',width=2)
 label(x0+20,y0+21,{'C30':'C 기본  ·  축 +30 mm','C65':'C 전진  ·  축 +65 mm','JR50':'JR  ·  축 +50 mm'}[key],28,b=True)
 label(x0+22,y0+68,'네 모서리 지주 고정' if key!='JR50' else '앞 지주를 라이다보다 뒤로 이동',19,'#536262')
 r=data[key]; sx=r['sensor_x_mm']; cx=wide/2; cy=187; sc=1.12
 canvas=Image.new('RGB',(wide-4,345),'#f6f8f5'); cr=ImageDraw.Draw(canvas)
 def pt(x,y):return (cx-y*sc,cy-x*sc)
 def rect(xa,ya,xb,yb,col,stroke=None):
  a,b=pt(xa,ya),pt(xb,yb);cr.rectangle((min(a[0],b[0]),min(a[1],b[1]),max(a[0],b[0]),max(a[1],b[1])),fill=col,outline=stroke,width=2)
 rect(-100,-75,100,75,'#e2e8e3','#667c73')
 def quadrant(center,half,dist):
  ax,bx=map(math.radians,[center-half,center+half]);
  def v(a,t):return pt(sx+t*math.cos(a),t*math.sin(a))
  cr.polygon([v(ax,dist),v(ax,650),v(bx,650),v(bx,dist)],fill='#f0a470')
 def line(a,b,color,w=1):cr.line([pt(*a),pt(*b)],fill=color,width=w)
 front,rear,yr=(10,-80,52) if key=='JR50' else (80,-80,62)
 posts=[(x,y) for x,mag in [(front,62),(rear,yr)] for y in (-mag,mag)]
 for x,y in posts:
  dist=math.hypot(x-sx,y);quadrant(math.degrees(math.atan2(y,x-sx)),math.degrees(math.asin(3.5/dist))+.5,dist)
 mx=-100.5;quadrant(180,math.degrees(math.atan2(25,sx-mx))+.5,sx-mx);line((mx,-25),(mx,25),'#324548',4)
 for x,y in posts:
  xx,yy=pt(x,y);cr.ellipse((xx-4.5,yy-4.5,xx+4.5,yy+4.5),fill='#263e40')
 # 마커판은 정후방 벽/전선통로가 아니다. 판 왼쪽 끝으로 직접 연결한 레이블.
 xx,yy=pt(mx,25);cr.line([(xx-34,yy+16),(xx-23,yy+16),(xx,yy)],fill='#435c60',width=2)
 cr.text((8,yy+11),'뒤 마커판',font=ft(16,True),fill='#27444c',anchor='lt')
 rect(sx-27.8,-27.8,sx+27.8,27.8,'#a3cfbd','#267561')
 xx,yy=pt(sx,0);cr.ellipse((xx-4,yy-4,xx+4,yy+4),fill='#124c42')
 for sign in (-1,1):
  a=math.radians(sign*r['front_unoccluded_fan_deg']/2)
  line((sx,0),(sx+500*math.cos(a),500*math.sin(a)),'#188579',2)
 cr.text((9,8),'↑ 전방',font=ft(19,True),fill='#2c5e58')
 im.paste(canvas,(x0+2,y0+108))
 label(x0+21,y0+465,'정면 연속 시야',20,'#43585a');label(x0+202,y0+455,f"{r['front_unoccluded_fan_deg']:.1f}°",36,'#005d51',True)
 label(x0+21,y0+510,f"라이다 본체 앞끝 여유   {r['sensor_front_body_clearance_mm']:.1f} mm",18,'#43585a')
label(40,712,'검은 점: 지주 4개    후면 막대: 차량 식별 마커판 ID 49 (폭 50 mm)    주황: 가려진 방향    초록선: 정면 연속 시야의 양 끝',20,'#37504e')
label(40,751,'마커판 높이 65~115 mm에 광축 75 mm가 겹치는 시뮬 설정입니다. 배선·전선 통로는 그리지 않았습니다.',19,'#37504e')
label(40,785,'라이다 본체는 축 중심 55.6 mm 사각 근사. 실물 마운트·커넥터·범퍼·조향 간섭과 실제 감지 성능은 별도입니다.',17,'#58706a')
im.save(here/'mount_comparison.png')
