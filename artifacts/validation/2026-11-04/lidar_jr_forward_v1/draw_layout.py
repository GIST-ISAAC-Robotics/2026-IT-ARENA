"""회전축·원형 지주 좌표를 사용해 상면 비교 1장을 그린다. Pillow, 맑은 고딕."""
from pathlib import Path
import math
from PIL import Image, ImageDraw, ImageFont
from layout_budget import layout

out = Path(__file__).with_name('jr_forward_top.png')
im = Image.new('RGB', (1500, 830), '#f2f6f5')
d = ImageDraw.Draw(im)


def label(x,y,txt,sz,ink='#173e46',bold=False):
    d.text((x,y), txt, fill=ink, font=ImageFont.truetype('C:/Windows/Fonts/malgun'+('bd' if bold else '')+'.ttf',sz))


d.rectangle((0,0,1500,130),fill='#163e46')
label(38,15,'IT ARENA   /   임시 JR   ·   하드웨어팀 상면 비교',22,'#b4ddd4',True)
label(37,52,'센서만 앞당길 때와 앞 지주도 함께 옮길 때',39,'white',True)
for j,(s,f,name) in enumerate([(50,10,'기준 JR50'),(60,10,'JR60 · 지주 고정'),(60,20,'JR60 · 앞 지주도 +10')]):
    x=24+492*j;cx=x+225;cy=394;scale=1.52
    def xy(a,b):return cx+scale*a,cy-scale*b
    def rect(a,b,c,e,color,line):d.rectangle((*xy(a,e),*xy(c,b)),fill=color,outline=line,width=2)
    def path(a,b,c,e,line,width=1):d.line((*xy(a,b),*xy(c,e)),fill=line,width=width)
    def circ(a,b,rad,color):d.ellipse((*xy(a-rad,b+rad),*xy(a+rad,b-rad)),fill=color)
    d.rounded_rectangle((x,153,x+466,705),radius=14,fill='white',outline='#c8d9d7',width=2)
    label(x+24,170,name,29,bold=True)
    label(x+24,221,f'센서 +{s} mm   /   앞 지주 +{f} mm',20)
    rect(-100,-75,100,75,'#eff4f2','#668387')
    for y in range(-73,76,8):path(0,y,0,min(y+3,75),'#b4c5c4')
    for a,b in ((f,62),(f,-62),(-80,52),(-80,-52)):
        dx=a-s;dist=math.hypot(dx,b);bearing=math.atan2(b,dx);half=math.asin(3.5/dist)
        near=[];ends=[]
        for ang in [bearing-half,bearing+half]:
            ux,uy=math.cos(ang),math.sin(ang)
            length=min(((-131 if ux<0 else 131)-s)/ux,88/abs(uy))
            near.append(xy(s+dist*math.cos(half)*ux,dist*math.cos(half)*uy))
            ends.append(xy(s+length*ux,length*uy))
        d.polygon([near[0],*ends,near[1]],fill='#e1a881')
        path(s,0,a,b,'#b0c3bf');circ(a,b,3.5,'#233c46')
    rect(s-27.8,-27.8,s+27.8,27.8,'#d2eae0','#368b77');circ(s,0,3,'#278b72')
    v=layout(s,f); b=v['posts_only']
    label(x+24,537,f"정면 연속 시야     {b['front_clear_deg']:.1f}°",25,'#137059',True)
    label(x+24,580,f"라이다 본체 앞 여유      {v['nose_margin_mm']:.1f} mm",20)
    label(x+24,616,f"상판 앞쪽 돌출 길이      {v['deck_overhang_to_x90_mm']} mm",20)
    label(x+24,652,f"정후방 연속 시야           {b['rear_clear_deg']:.1f}°",20)
label(36,729,'오른쪽이 전방 →    차체 200 × 150 mm    /    점: Ø7 지주    /    살구색: 지주가 가리는 방향    /    각도는 스캔면의 실제 기하 기준',21)
label(36,775,'앞 여유는 라이다 외함에서 차체 앞끝까지, 돌출은 앞 지주에서 예시 상판 앞끝(+90 mm)까지입니다. 실제 전선관과 클립은 추가 가림을 만듭니다.',19,'#48666c')
im.save(out)
