"""Rear reinforcement + forward C1 + stacked PDBs; no changes to driving configuration."""
import json
import math
from pathlib import Path
from argparse import ArgumentParser
from dataclasses import asdict
from study_c1_mount import OUT, Layout, four, intervals, union, overlap, mask


def wheel_mask(sx, z, steer):
    # Ideal rigid 65 mm tires with 12 mm axial width, axes 135 x 135 mm apart.
    # Evaluate a rectangular cut of each cylinder, rotating the front pair in yaw.
    if abs(z-32.5)>32.5:
        return []
    half=math.sqrt(32.5**2-(z-32.5)**2)
    spans=[]
    for x in (-67.5,67.5):
        yaw=steer if x>0 else 0
        for y in (-67.5,67.5):
            center=math.degrees(math.atan2(y,x-sx))
            bearings=[]
            for a in (-half,half):
                for b in (-6,6):
                    xx=x+a*math.cos(yaw)-b*math.sin(yaw)
                    yy=y+a*math.sin(yaw)+b*math.cos(yaw)
                    angle=math.degrees(math.atan2(yy,xx-sx))
                    bearings.append(center+(angle-center+180)%360-180)
            lo,hi=min(bearings),max(bearings)
            spans+=intervals((lo+hi)/2,(hi-lo)/2)
    return union(spans)


def main():
    parser=ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,default=OUT)
    out=parser.parse_args().out;out.mkdir(parents=True,exist_ok=True)
    layouts=[
        Layout("J0","중앙 · 뒤판 100",0,plate=(-60,100),note="후방 보강 앞면 x=−60"),
        Layout("J50","50 mm 전진",50,plate=(-60,100),note="같은 뒤판 · 센서–판 110 mm"),
        Layout("J60","60 mm 전진",60,plate=(-60,100),note="같은 뒤판 · 센서–판 120 mm"),
        Layout("JS","전진 + 뒤판 + 네 지주",50,four(10),(-60,100),note="뒤판과 Ø7 기둥을 함께 계산"),
        Layout("JP","큰 부품이 창까지 침범",50,plate=(0,140),note="폭 140, 깊이 100의 불투명 단면"),
        Layout("JR","전진 + 열린 뒤 보강",50,four(10)[:2]+((-80,-52,7),(-80,52,7)),note="보강·PDB는 창 위, 뒤 지주는 바퀴 안쪽"),
    ]
    result=[]
    for l in layouts:
        arcs=mask(l)
        result.append(dict(**asdict(l), arcs=arcs, total_deg=round(sum(b-a for a,b in arcs),4),
            front100=round(overlap(arcs,[(-100,100)]),4), body_front_margin_mm=round(100-l.sx-27.8,1),
            center_min_range_gap_mm=max(0,50-(100-l.sx))))
    # At z=45.2 mm, rear post x=-80 lies within the rear tire chord.
    rear_fit=[dict(post_abs_y_mm=y,post_d_mm=d,tire_inner_y_mm=61.5,
                   lateral_clearance_mm=61.5-y-d/2) for y in (52,62) for d in (7,10)]
    wheels=[]
    for z in (55,60,65,70,75,80,85,95):
        for steer in (-.45,0,.45):
            ar=wheel_mask(50,z,steer)
            combined=union(ar+mask(layouts[-1]))
            wheels.append(dict(plane_mm=z,steer_rad=steer,wheels_deg=round(sum(b-a for a,b in ar),4),
                jr_with_wheels_deg=round(sum(b-a for a,b in combined),4),
                mount_top_mm=round(z-29.8,1),c1_top_mm=round(z+11.5,1),
                shelf_underside_example_mm=round(z+20.2,1)))
    # Two board orientations; each board on a different z level, x origin unchanged.
    pdb=[]
    for name,length,width,back,front in [('wide',100,140,-95,35),('long',140,100,-95,55)]:
        bx=-90
        L=front-10
        pdb.append(dict(layout=name,board_x=[bx,bx+length],board_y=[-width/2,width/2],shelf_x=[back,front],
            shelf_width_mm=140,overhang_mm=L,body_side_reserve_mm=(150-width)/2,
            ideal_tip_mm=round((.25*3*9.81)*L**3/(3*69000*(60*2**3/12)),5)))
    (out/'stack.json').write_text(json.dumps(dict(layouts=result,wheel_slices=wheels,pdb=pdb,rear_post_fit=rear_fit),
        indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    for l in result: print(l['id'],l['total_deg'],l['front100'],l['body_front_margin_mm'],l['center_min_range_gap_mm'])
    print('24 tire slices; 2 board orientations')


if __name__=='__main__':
    main()
