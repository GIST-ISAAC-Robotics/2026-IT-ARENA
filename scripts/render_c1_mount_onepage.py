"""Render a single top-view comparison from the existing C1 study JSON (Pillow)."""
import json
import math
import os
from pathlib import Path
from functools import lru_cache
from PIL import Image, ImageDraw, ImageFont

OUT=Path(__file__).resolve().parents[1]/"artifacts/validation/2026-11-03/lidar_mount_study_v1"
FONT=Path(os.environ.get("WINDIR","C:/Windows"))/"Fonts"
S=2
INK="#132f36"; SUB="#425d61"; TEAL="#006e65"; LINE="#c8d5cf"; DARK="#304955"
PAPER="#fafaf5"; CARD="#ffffff"; BODY="#e5ede7"; SAND="#ecae83"; SENSOR="#acd8c9"

@lru_cache()
def font(size,bold):
    return ImageFont.truetype(str(FONT/("malgunbd.ttf" if bold else "malgun.ttf")),size*S)

def box(pts): return tuple(round(x*S) for x in pts)

def label(d,xy,value,size=20,color=INK,bold=False,anchor="lt"):
    d.text(box(xy),value,font=font(size,bold),fill=color,anchor=anchor,stroke_width=0)

def rect(d,coords,fill,outline=None,rad=0,w=1):
    d.rounded_rectangle(box(coords),radius=rad*S,fill=fill,outline=outline,width=w*S)

def topview(l,highlight):
    ww,hh=222,202
    img=Image.new("RGB",(ww*S,hh*S), "#f1f7f1" if highlight else "#f4f7f3")
    d=ImageDraw.Draw(img)
    scale=.69; cx,cy=ww/2,hh/2
    def pt(x,y):return box((cx-scale*y,cy-scale*x))
    def poly(vertices,fill):d.polygon([pt(x,y) for x,y in vertices], fill=fill)
    def beam(p,q):
        d.line([pt(*p),pt(*q)],fill="#aab9ae",width=S)
    def metric_rect(x0,y0,x1,y1,fill,stroke=None,rad=0):
        d.rounded_rectangle((*pt(x1,y1),*pt(x0,y0)),radius=rad*S,fill=fill,outline=stroke,width=S)
    def disk(x,y,r,color):
        d.ellipse((*pt(x+r,y+r),*pt(x-r,y-r)),fill=color)
    metric_rect(-100,-75,100,75,BODY,"#879b92",4)
    # Wedges start at the obstacle, not inside the chassis between it and the C1.
    sx=l["sx"]
    for x,y,diam in l["posts"]:
        r=math.hypot(x-sx,y); direction=math.atan2(y,x-sx)
        half=math.asin(diam/(2*r)); near=math.sqrt(r*r-(diam/2)**2)
        def v(a,t):return (sx+t*math.cos(a),t*math.sin(a))
        a,b=direction-half,direction+half
        poly([v(a,near),v(a,620),v(b,620),v(b,near)],SAND)
        beam((sx,0),(x,y))
    if l["plate"]:
        x,w=l["plate"]
        # Finite plate with a rearward shadow; dashboard geometry always has w>0.
        poly([(x,-w/2),(sx+8*(x-sx),-4*w),(sx+8*(x-sx),4*w),(x,w/2)],SAND)
        for y in (-w/2,w/2):beam((sx,0),(x,y))
        metric_rect(-100,-w/2,x,w/2,DARK)
    for x,y,diam in l["posts"]:disk(x,y,diam/2,DARK)
    metric_rect(sx-27.8,-27.8,sx+27.8,27.8,SENSOR,TEAL,4)
    disk(sx,0,4,TEAL)
    # Nose tick marks vehicle orientation even in a cropped/forward-shifted view.
    d.line([pt(102,15),pt(102,-15)],fill=TEAL,width=3*S)
    return img

ROWS=[
("01","네 모서리 기둥을 고정하고 라이다만 이동","지주 위치 동일  ·  차체 중심을 0 mm로 표시",[
 ("A","중앙 장착","앞뒤 그림자가 대칭","전방 대각에 사각이 남음"),
 ("B","30 mm 후퇴","앞 기둥 그림자가 좁아짐","대신 정면 쪽으로 이동"),
 ("C","30 mm 전진","앞 대각 가림이 줄어듦","대신 뒤 대각 가림 증가")]),
("02","같은 후방판에서 라이다만 전진","폭 100 mm 뒤판만 계산  ·  추가 지주는 다음 줄에서 비교",[
 ("J0","중앙 + 뒤판","정후방이 넓게 가려짐","전진 효과를 보는 기준"),
 ("J50","50 mm 전진","뒤판의 가림을 크게 줄임","앞쪽 몸체 여유 22 mm"),
 ("J60","60 mm 전진","J50보다 약 3.6° 개선","앞쪽 몸체 여유 12 mm")]),
("03","기둥과 후방 보강 위치까지 변경","전방 시야를 확보하면서 뒤쪽을 얼마나 남길지 비교",[
 ("D","앞 기둥만 후퇴","라이다는 중앙에 유지","가림이 대각에서 옆으로"),
 ("JS","뒤판 + 네 기둥","라이다를 50 mm 전진","뒤판·기둥이 함께 가림"),
 ("JR","열린 후방 + 네 기둥","라이다를 50 mm 전진","정면과 정후방 모두 열림")])]

def main():
    layouts={r["id"]:r for name in ("results.json","stack.json") for r in json.loads((OUT/name).read_text(encoding="utf-8"))["layouts"]}
    W,H=1580,1158
    img=Image.new("RGB",(W*S,H*S),PAPER);d=ImageDraw.Draw(img)
    rect(d,(0,0,W,10),TEAL)
    label(d,(46,30),"IT ARENA   /   기구팀 설계 검토",19,TEAL,True)
    label(d,(W-46,32),"2026.11.03   ·   평면도",18,SUB,anchor="rt")
    label(d,(44,62),"라이다 배치 9안, 어디가 가려지는가",48,INK,True)
    label(d,(47,128),"차체 200 × 150 mm   ·   ↑ 전방   ·   동일 축척   ·   지주 유효 지름 7 mm",22,SUB)
    for x,col,txt in [(960,SENSOR,"라이다"),(1095,DARK,"기둥·뒤판"),(1274,SAND,"가려지는 방향")]:
        rect(d,(x,131,x+22,152),col,TEAL if col==SENSOR else None,4)
        label(d,(x+31,130),txt,20,SUB,True)
    gap,cw,left=13,487,46
    for row,(n,title,desc,items) in enumerate(ROWS):
        sy=183+row*283
        label(d,(left,sy),n,24,TEAL,True)
        label(d,(left+48,sy),title,25,INK,True)
        label(d,(1531,sy+4),desc,17,SUB,anchor="rt")
        for j,(key,title,one,two) in enumerate(items):
            x=left+j*(cw+gap); y=sy+38; l=layouts[key];rec=key=="JR"
            rect(d,(x,y,x+cw,y+230),"#f2f9f4" if rec else CARD,TEAL if rec else LINE,10,3 if rec else 1)
            tile=topview(l,rec);img.paste(tile,box((x+10,y+14)))
            tx=x+248
            label(d,(tx,y+20),key+("  /  추천 목업" if rec else ""),17,TEAL,True)
            label(d,(tx,y+47),title,22,INK,True)
            label(d,(tx,y+88),f"{l['total_deg']:.1f}°",39,TEAL if rec else INK,True)
            label(d,(tx,y+138),"전체 사각",16,SUB)
            label(d,(tx,y+169),one,17,INK)
            label(d,(tx,y+197),two,17,INK)
    fy=1036
    rect(d,(46,fy,1534,fy+69),            "#e2eee7",None,9)
    label(d,(66,fy+23),"추천  JR",22,TEAL,True)
    label(d,(200,fy+23),"앞 50 mm에 라이다를 두고, 뒤 가로 보강은 광학창 밖으로. 뒤 지주는 바퀴 안쪽으로 옮깁니다.",22,INK,True)
    label(d,(48,1124),"전체 사각 = 가려지는 방향의 합집합. 기존 시뮬의 정후방 연속 60°와는 분포가 다릅니다. 실측 전 수평 기하 비교입니다.",19,SUB)
    name=OUT/"c1_mount_topview_onepage.png";img.resize((2370,1737),Image.Resampling.LANCZOS).save(name,optimize=True)
    print(name)


if __name__=="__main__":main()
