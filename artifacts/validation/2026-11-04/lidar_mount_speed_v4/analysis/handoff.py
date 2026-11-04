"""Draw a stand-alone engineering figure from measured runs. Pillow / Malgun Gothic."""
from pathlib import Path
import json,math
from PIL import Image,ImageDraw,ImageFont
P=Path(__file__).resolve().parent.parent
old=json.loads((P/'v3_baseline.json').read_text())
result=json.loads((P/'comparison.json').read_text())['runs']
im=Image.new('RGB',(1600,1000),'#f3f6f6');d=ImageDraw.Draw(im)
def tx(x,y,s,size=23,fill='#183945',b=False,anchor='la'):
    font=ImageFont.truetype('C:/Windows/Fonts/malgun'+('bd' if b else '')+'.ttf',size)
    d.text((x,y),s,font=font,fill=fill,anchor=anchor)
d.rectangle((0,0,1600,150),fill='#173b48')
tx(44,22,'IT ARENA   /   C1 장착 비교    ·    수정 후 단독 주행',22,'#b4dfd3',True)
tx(42,56,'C도 증속했다. 완주 차이는 코너에서 났다.',42,'white',True)
tx(44,116,'차체 200×150 mm · 광축 75 mm · 마커 하단 115 mm · 차체 중앙 기준, 오른쪽이 전방  →    |    검은 점: Ø7 지주    살구색: 기둥 그림자',21,'#c8dadd')
names=[('C30','c30_1',30,80,62),('C50','c50_2',50,80,62),('C65','c65_1',65,80,62),('JR50','jr50_1',50,10,52)]
def fmt(v,k=2):
    if not v:return '—'
    return ' / '.join(f'{x:.{k}f}' for x in v)
for j,(key,baseline,s,fx,ry) in enumerate(names):
    xx=32+390*j; d.rounded_rectangle((xx,173,xx+365,825),18,fill='white',outline='#d4dfe2',width=2)
    rs=[result[k] for k in sorted(result) if k.lower().startswith(key.lower()+'_')]
    tx(xx+23,191,key,38,b=True);tx(xx+23,248,'센서 중심 +%d mm'%s,23)
    cx=xx+181;cy=393;scale=.98
    def xy(x,y):return(cx+scale*x,cy-scale*y)
    def rect(x,y,a,b,fill,line):d.rectangle((*xy(x,b),*xy(a,y)),fill=fill,outline=line,width=2)
    rect(-100,-75,100,75,'#edf4f3','#78959b')
    for px,yy in [(fx,62),(fx,-62),(-80,ry),(-80,-ry)]:
        distance=math.hypot(px-s,yy);ori=math.atan2(yy,px-s);h=math.asin(3.5/distance)
        ends=[];near=[]
        for ang in [ori-h,ori+h]:
            ux,uy=math.cos(ang),math.sin(ang)
            length=min(((145 if ux>0 else -145)-s)/ux,92/abs(uy))
            near.append(xy(s+distance*math.cos(h)*ux,distance*math.cos(h)*uy))
            ends.append(xy(s+length*ux,length*uy))
        for step in range(0,int(distance)-6,8):
            a,b=step,min(step+3,distance-6)
            d.line((xy(s+a*math.cos(ori),a*math.sin(ori)),xy(s+b*math.cos(ori),b*math.sin(ori))),fill='#a7bdbc')
        d.polygon([near[0],*ends,near[1]],fill='#d9895c')
        a,b=xy(px,yy); d.ellipse((a-4,b-4,a+4,b+4),fill='#233d46')
    rect(s-27.8,-27.8,s+27.8,27.8,'#d3ece5','#307e79')
    a,b=xy(s,0);d.ellipse((a-20,b-20,a+20,b+20),outline='#288e7b',width=3);tx(a,b,'C1',18,'#20695c',True,'mm')
    fan=2*math.degrees(math.atan2(62,fx-s)-math.asin(3.5/math.hypot(fx-s,62)))
    tx(xx+23,488,'정면 개방각 %.0f°  ·  앞 여유 %.1f mm'%(fan,100-s-27.8),18,'#48666c')
    tx(xx+23,519,'이전 코드  ·  1랩',20,'#58717a')
    tx(xx+23,541,fmt([round(old[baseline]['lap_s'],3)+1e-8])+'초',28,b=True)
    tx(xx+23,591,'수정 후  ·  1랩'+('  (각 실행)' if len(rs)>1 else ''),20,'#58717a')
    lap_text=' / '.join(f"{v['lap_s']:.2f}초" if v.get('lap_s') is not None else f"미완주 {v['progress_m']:.1f} m" for v in rs) or '결과 대기'
    tx(xx+23,621,lap_text,25 if len(lap_text)>16 else (28 if len(rs)>1 else 31),'#11735e' if all(v.get('lap_s') is not None for v in rs) else '#a55422',True)
    tx(xx+23,686,'차체 최고속도 (m/s)    '+fmt([v['peak_mps'] for v in rs]),18)
    tx(xx+23,726,'출발 후 실제 정지 (회)    '+fmt([v['measured_stop_after_first_motion']['count'] for v in rs],0),18)
    tx(xx+23,766,'출발 후 0 명령 (회)    '+fmt([v['zero_command_after_motion']['count'] for v in rs],0),18)
tx(43,858,'검사 조건',21,'#11735e',True)
tx(43,898,'실제 정지 = 첫 이동 후, 결승 또는 실패 판정 전 차체 속도 0.05 m/s 미만이 0.10초 이상 지속.  0 명령 = 같은 구간에 추종기가 연속으로 정지를 지시한 횟수.',19)
tx(43,943,'두 숫자가 있으면 순서대로 1·2차 실행값. 랩 시간은 초록 신호 기준. 구조 그림자를 빈 공간으로 채우지 않았으며 실차·추월 검증은 포함하지 않습니다.',19)
im.save(P/'mount_speed_comparison.png')
