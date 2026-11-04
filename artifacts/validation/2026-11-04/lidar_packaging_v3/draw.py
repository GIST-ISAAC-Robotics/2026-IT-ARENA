"""Top views from geometry.json / lower_fit.json. Pillow, Malgun Gothic (Windows)."""
import json,math
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
P=Path(__file__).resolve().parent
J=json.loads((P/'geometry.json').read_text(encoding='utf8'))
fit=json.loads((P/'lower_fit.json').read_text())['JR50_h102_stepped_steer26_diff1_posts']
F='C:/Windows/Fonts/malgun'; im=Image.new('RGB',(1776,1162),'#f3f5f6'); d=ImageDraw.Draw(im)
def font(n,b=False):return ImageFont.truetype(F+('bd' if b else '')+'.ttf',n)
def t(x,y,s,n=23,fill='#172e3e',b=False,anchor=None):
    d.text((x,y),s,font=font(n,b),fill=fill,anchor=anchor,stroke_width=0)
def panel(X,Y,title,kicker):
    d.rounded_rectangle((X,Y,X+848,Y+405),18,fill='white',outline='#d5dfe1',width=2)
    t(X+24,Y+20,title,31,b=True);t(X+25,Y+66,kicker,18,'#476675')
    return X+250,Y+242
scale=1.86
def pos(c,x,y):return(c[0]+scale*x,c[1]-scale*y)
def bb(c,x,y,a,b):return (*pos(c,x,b),*pos(c,a,y))
def box(c,x,y,a,b,fill,stroke,w=2): d.rectangle(bb(c,x,y,a,b),fill=fill,outline=stroke,width=w)
def disk(c,x,y,r,color,edge=None,w=1): d.ellipse(bb(c,x-r,y-r,x+r,y+r),fill=color,outline=edge,width=w)
def chassis(c,tires=True):
    box(c,-100,-75,100,75,'#f4f8f8','#64838e')
    if tires:
        for x in [-98.7,34.3]:
            for y in [-74,47.9]: box(c,x,y,x+64.4,y+26.1,'#dce4e5','#b6c4c6',1)
def tags(c,notes):
    x=c[0]+221;y=c[1]-112
    for a,b in notes:
        t(x,y,a,20,'#45636c');t(x,y+30,b,24,'#183c49',True); y+=81
def scan(c,model,s,front,ry):
    chassis(c); origin=pos(c,s,0)
    # Shade only inside car and a small surrounding frame by clipping the ray overlay.
    overlay=Image.new('RGBA',im.size,(0,0,0,0)); od=ImageDraw.Draw(overlay)
    for l,r in J['optics'][model]['radial']['intervals_deg']:
        a,b=[math.radians(i) for i in (l,r)]
        od.polygon([origin,pos(c,s+270*math.cos(a),270*math.sin(a)),pos(c,s+270*math.cos(b),270*math.sin(b))],fill=(216,113,71,130))
    bbox=[int(v) for v in bb(c,-119,-80,112,80)]
    crop=overlay.crop(bbox); im.paste(crop,bbox[:2],crop)
    box(c,s-27.8,-27.8,s+27.8,27.8,'#d4efea','#16998a',2)
    disk(c,s,0,22,'#e1f5f0','#16998a',3);t(*origin,'C1',22,'#127566',True,'mm')
    for x,y in [(front,62),(front,-62),(-80,ry),(-80,-ry)]: disk(c,x,y,3.5,'#273a42')
    for x,y,D in J['optics'][model]['rear_ducts_xyD_mm']:disk(c,x,y,D/2,'#ea8658','#9a482e',2)
def center(c,x,y,s,n=17): t(*pos(c,x,y),s,n,'#233e4a',True,'mm')
d.rectangle((0,0,1776,145),fill='#173b48')
t(38,23,'IT ARENA   /   하드웨어 검토     2026.11.04',19,'#a6d8cd',True)
t(36,53,'뒤를 열어 둔 라이다 배치와 전장품 적층',44,'white',True)
t(38,112,'모든 그림은 같은 축척의 상면도입니다.  오른쪽이 차량 전방  →     |     검사 외곽 200 × 150 mm, 좌표는 차체 중심 기준',20,'#c7dedf')
c=panel(30,164,'01   JR  /  앞 지주를 뒤로','스캔 높이에서 본 유효 단면  ·  회색 타이어는 아래층 투영')
scan(c,'JR50',50,10,52)
tags(c,[('라이다 / 앞 지주 x','+50 / +10 mm'),('정면 / 후면 연속 시야¹','240° / 40°'),('기둥 + 분리 배선 사각','18.6° (기둥만 16.6°)')])
c=panel(898,164,'02   C  /  네 모서리 지지','JR과 같은 라이다 +50 mm  ·  앞 지주는 +80 mm, 뒤 지주 폭도 다름')
scan(c,'C50',50,80,62)
tags(c,[('라이다 / 앞 지주 x','+50 / +80 mm'),('정면 / 후면 연속 시야¹','123° / 47°'),('센서만 +65 mm로 이동','정면 147° · 앞 여유 7.2 mm')])
c=panel(30,589,'03   JR 하부  /  가운데에 2단 적층','광축 102 mm 예  ·  배터리 밑면 12 mm, 모터 밑면 49 mm')
chassis(c)
box(c,-82,-16,-51,16,'#e9d8c6','#9a785b');center(c,-66.5,0,'차동',16)
for k,color,name in [('battery','#d8d5e9','배터리'),('motor','#b5d8e1','모터'),('servo','#f1d9ad','서보')]:
    x,y,z,a,b,h=fit[k];box(c,x,y,a,b,color,'#66828c')
    if k!='battery': center(c,(x+a)/2,(y+b)/2,name,18)
for x,y,a,b in [(22.2,-27.8,77.8,27.8)]:
    points=[pos(c,x,y),pos(c,x,b),pos(c,a,b),pos(c,a,y),pos(c,x,y)]
    for p,q in zip(points,points[1:]):
        length=math.dist(p,q)
        for i in range(0,int(length),13):
            d.line([(p[0]+(q[0]-p[0])*v/length,p[1]+(q[1]-p[1])*v/length) for v in [i,min(i+7,length)]],fill='#127566',width=3)
tags(c,[('중앙 파랑 / 아래 보라','가로 모터 / 눕힌 배터리'),('왼쪽 갈색 / 초록 점선','차동 예약 / 위층 C1 투영'),('남은 확인','기어 연결·실제 조향·열')])
c=panel(898,589,'04   상부  /  보드 평면 배치','PDB 2장은 각각 별도 층  ·  그 위 한 층의 전장품 예')
chassis(c,False);box(c,-95,-70,90,70,'#eff4f3','#91a8a6')
colors=['#c3dbd2','#c3daed','#e1d5ed','#e5ded3','#f3d6ac']
for row,color,label in zip(J['upper_rects'],colors,['Jetson','MD20A','ESP32','서보 어댑터','CAM']):
    x,y,a,b=row['xyXY_mm'];box(c,x,y,a,b,color,'#778b8d');center(c,(x+a)/2,(y+b)/2,label,14 if label in ['CAM','서보 어댑터'] else 18)
tags(c,[('상부 트레이 예','185 × 140 mm'),('PDB J01 / M01 각 층','141 × 100 × 30.5 mm'),('후방 마커·가로 보강','모두 광학창 위 또는 밖')])
diag=json.loads((P/'drive_diagnostics.json').read_text())
def lap(n):
    if diag[n]['completed_laps']>0:return f"{diag[n]['lap_s']:.2f}초"
    return '미완주'
names=[('JR50','jr50_1'),('C30','c30_1'),('C50','c50_2'),('C65','c65_1')]
scores='     /     '.join(f'{k}  {lap(n)}' for k,n in names if n in diag)
d.rounded_rectangle((30,1007,1746,1060),12,fill='#d9eae5')
t(48,1017,'단독 1랩 · 각 1회     '+scores,24,'#153e46',True)
t(38,1071,'¹ 정면·정후방 중심의 대칭 연속각. 주황 = 기둥 그림자·배선관. 그림은 외경 9~11 mm 배선관, 주행은 지주와 그림자가 겹치는 얇은 선·광축 75 mm입니다.',19,'#385763')
t(38,1107,'부품 형상은 구형 STEP 기준입니다. 제작도면·실차 조향·구조 강도·추월 검증이 아니며, 75 mm 주행 결과를 102 mm 실장 예에 그대로 적용하지 않습니다.',19,'#385763')
im.save(P/'top_view.png')
