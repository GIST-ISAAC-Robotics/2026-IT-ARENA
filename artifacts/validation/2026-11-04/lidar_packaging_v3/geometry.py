"""Produce the dimension and optical budget. mm; x forward, y left. No drive simulation."""
from pathlib import Path
import math as m, json, itertools
P=Path(__file__).resolve().parent
old=json.loads((P.parent/'lidar_mount_cad_reference'/'assembly.json').read_text(encoding='utf8'))
parts={r['label'].split('_')[0]:r['bounds'] for r in old['top_level']}
def rounded(x): return round(x,3)
def angle(s,o):
    x,y,d=o; a=m.degrees(m.atan2(y,x-s))%360
    h=m.degrees(m.asin(d/2/m.hypot(x-s,y)))
    return a,h
def arcs(s,objects,extra=[]):
    intervals=[]
    for a,h in [angle(s,o) for o in objects]+extra:
        for turn in [-360,0,360]:
            l=max(0,a-h+turn);r=min(360,a+h+turn)
            if r>l: intervals.append((l,r))
    merged=[]
    for l,r in sorted(intervals):
        if merged and l<=merged[-1][1]: merged[-1][1]=max(merged[-1][1],r)
        else: merged.append([l,r])
    def central(a):
        if any(l<=a<=r for l,r in merged): return 0
        l=max(r-turn for _,r in merged for turn in [0,360] if r-turn<=a)
        r=min(l+turn for l,_ in merged for turn in [0,360] if l+turn>=a)
        return 2*min(r-a,a-l)
    return dict(total=rounded(sum(r-l for l,r in merged)),front_symmetric=rounded(central(0)),rear_symmetric=rounded(central(180)),intervals_deg=[[rounded(l),rounded(r)] for l,r in merged])
def posts(front,rear_y): return [(front,62,7),(front,-62,7),(-80,rear_y,7),(-80,-rear_y,7)]
def tube(ds,phi=0.5,wall=1):
    inner=m.sqrt(sum(d*d for d in ds)/phi)
    return dict(insulated_od=ds,inner_mm=rounded(inner),outer_mm=rounded(inner+2*wall),insulated_cross_section_mm2=rounded(sum(m.pi*d*d/4 for d in ds)))
def harness(s,front,ry,diam,style):
    p=posts(front,ry)
    for (x,y,_),d in zip(p[2:4],diam):
        length=m.hypot(x-s,y); ux=(x-s)/length; uy=y/length
        if style=='tangent_out': ux,uy=(abs(uy),m.copysign(abs(ux),y))
        gap=(7+d)/2
        p.append((x+ux*gap,y+uy*gap,d))
    return p
power=tube([3.,3.,3.,3.]); data=tube([3.5,3,2.5])
ds=[power['outer_mm'],data['outer_mm']]
optics={}
for model,s,front,ry in [('JR50',50,10,52),('JR50_inboard',50,10,36)]+[(f'JR50_front{x}',50,x,52) for x in [20,25,30]]+[(f'C{x}',x,80,62) for x in (30,50,55,60,65)]:
    pp=posts(front,ry)
    optics[model]={style:arcs(s,harness(s,front,ry,ds,style)) for style in ['radial','tangent_out']}
    optics[model]['bare']=arcs(s,pp)
    # A plane of negligible fore-aft thickness; the specified x is its nearest face.
    optics[model]['rear_center_duct']={w:arcs(s,pp,[(180,m.degrees(m.atan(w/2/(s+80))))]) for w in [12,20,30,50]}
    optics[model]['front_to_bumper_mm']=rounded(100-s-27.8)
    optics[model]['unsupported_near_field_ahead_of_bumper_mm']=max(0,s-50)
    optics[model]['v2_half_degree_margin_front_deg']=rounded(optics[model]['bare']['front_symmetric']-1)
    optics[model]['rear_ducts_xyD_mm']=[[rounded(i) for i in o] for o in harness(s,front,ry,ds,'radial')[-2:]]
    optics[model]['power_wire_4mm_radial']=arcs(s,harness(s,front,ry,[tube([4.]*4)['outer_mm'],ds[1]],'radial'))
# Trial board orientations on the upper deck. No free margin is counted as a connector.
board_rows=[('Jetson','05',-94,-67,1,0),('MD20A','06',1,-28,2,1),
            ('ESP32','07',-85,41,0,1),('servo_adapter','08',4,-67,0,1),('camera','11',65,-12.5,1,0)]
boards={}
for label,k,x,y,xi,yi in board_rows:
    b=parts[k]['size']; boards[label]=[x,y,rounded(x+b[xi]),rounded(y+b[yi])]
assert all(-95<=x<a<=90 and -70<=y<b<=70 for x,y,a,b in boards.values())
for (x,y,a,b),(u,v,c,d) in itertools.combinations(boards.values(),2):
    assert a<=u or c<=x or b<=v or d<=y
zdata={}
for H in (75,85,90,95,115):
    mount=H-29.8
    zdata[str(H)]=dict(lidar_base=rounded(mount),max_under_top_3gap=rounded(mount-3),
        lidar_top=rounded(mount+41.3),max_floor_if_stacking_motor_below=rounded(mount-3-parts['02']['size'][2]),
        max_floor_if_stacking_upright_servo_below=rounded(mount-3-parts['10']['size'][2]),
        max_floor_if_stacking_flat_battery_below=rounded(mount-3-34))
out=dict(axes='x forward y left mm; angles from front; symmetric clear fan incl center ray; positive CCW',
         harness=dict(fill=0.5,wall_mm=1,power=power,data=data),optics=optics,heights_mm=zdata,
         upper_rects=[{'part':p,'xyXY_mm':b} for p,b in boards.items()],
         stability=[dict(cg_h_mm=h,ay_g=rounded(60.95/h),ay_g_if_10mm_offset=rounded(50.95/h)) for h in [90,110,130]])
(P/'geometry.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
if __name__=='__main__':
    for name,row in optics.items():
        print(name,*[(style, *list(row[style].values())[:3]) for style in ['bare','radial','tangent_out']])
    print(ds,boards)
