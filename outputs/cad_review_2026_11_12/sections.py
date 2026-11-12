from analyze_geometry import *
import sys
from OCP.GeomAbs import GeomAbs_Line
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle
plt.rcParams['font.family']='Malgun Gothic'
plt.rcParams['axes.unicode_minus']=False

def cuts(s,height):
    op=BRepAlgoAPI_Section(s,gp_Pln(gp_Pnt(0,height-32.2,0),gp_Dir(0,1,0)),False); op.Build(); lines=[]
    ex=TopExp_Explorer(op.Shape(),TopAbs_EDGE)
    while ex.More():
        a=BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        ts=np.linspace(a.FirstParameter(),a.LastParameter(),2 if a.GetType()==GeomAbs_Line else 65)
        lines.append([xyz(a.Value(float(t)).Coord())[:2] for t in ts]); ex.Next()
    return lines

def ray_hits(lines,origin):
    ang=np.arange(-180,180,.02); rays=np.c_[np.cos(np.deg2rad(ang)),np.sin(np.deg2rad(ang))]
    hit=np.zeros(len(ang),dtype=bool)
    for line in lines:
        for p,q in zip(line[:-1],line[1:]):
            p=np.array(p)-origin; d=np.array(q)-origin-p
            cross=rays[:,0]*d[1]-rays[:,1]*d[0]
            cross=np.where(abs(cross)<1e-12,np.nan,cross)
            t=(p[0]*d[1]-p[1]*d[0])/cross
            u=(p[0]*rays[:,1]-p[1]*rays[:,0])/cross
            hit |= (t>0)&(u>=-1e-8)&(u<=1+1e-8)
    bounds=[]; start=None
    for i,h in enumerate(np.r_[hit,False]):
        if h and start is None: start=i
        if not h and start is not None:
            bounds.append([round(float(ang[start]),2),round(float(ang[min(i,len(ang)-1)]+(.02 if i==len(ang) else 0)),2)]); start=None
    return {'blocked_deg':float(hit.sum()*.02),'intervals_deg':bounds}

result=json.loads((OUT/'sections.json').read_text(encoding='utf-8')) if '--reuse' in sys.argv else {}
for height in ([] if '--reuse' in sys.argv else [75,115,125,134.6,140,149,151.5,154,160,190.5]):
    data=[]
    for r in groups:
        b=r['bbox']
        if r['id']==1858 or not b[1]<height-32.2<b[4]: continue
        ls=cuts(readshape(r['id']),height)
        if ls:data.append({'id':r['id'],'name':r['name'],'lines':ls})
    result[str(height)]=data
    print('SECTION',height,[(d['id'],len(d['lines'])) for d in data],flush=True)
(OUT/'sections.json').write_text(json.dumps(result,ensure_ascii=False),encoding='utf-8')

origin=np.array([75.4,0])
fig,axs=plt.subplots(1,2,figsize=(15,8))
for ax,mode in zip(axs,['old','new']):
    ax.add_patch(Rectangle(((-100 if mode=='old' else -96.2),-75),(200 if mode=='old' else 199.7),150,fill=False,ls='--',edgecolor='gray'))
    if mode=='old':
        org=np.array([60,0]); lines=[]
        for x,y,d in [(20,62,7),(20,-62,7),(-80,52,7),(-80,-52,7),(-88.906,55.308,20),(-88.906,-55.308,20)]:
            a=np.linspace(0,2*np.pi,361); p=np.c_[x+d/2*np.cos(a),y+d/2*np.sin(a)];lines.append(p)
            ax.plot(p[:,0],p[:,1],c='#d27b24');ax.text(x,y+6,f'{x:g}, {y:g}',fontsize=9,ha='center')
        title='기존 JR60P20 · 광축 높이 75 mm\nØ7 기둥 4개 + Ø20 배선 예약 2개'
    else:
        org=origin;lines=[]
        for d in result['134.6']:
            for line in d['lines']:
                p=np.array(line);lines.append(p);ax.plot(p[:,0],p[:,1],color='#c36b20')
            ps=np.concatenate(d['lines']);ax.text(ps[:,0].mean(),ps[:,1].mean(),str(d['id']),fontsize=10)
        title='이번 STEP · 추정 광축 높이 134.6 mm\n실제 절단 형상 · 추가 배선 제외'
    hit=ray_hits(lines,org);print(mode,hit,flush=True)
    result[mode+'_occlusion']=hit
    ax.plot(*org,'o',c='#147a9b');ax.add_patch(Circle(org,27.8,fill=False,color='#147a9b'))
    for lo,hi in hit['intervals_deg']:
        aa=np.deg2rad(np.linspace(lo,hi,10)); ps=org+230*np.c_[np.cos(aa),np.sin(aa)]
        ax.fill(np.r_[org[0],ps[:,0],org[0]],np.r_[org[1],ps[:,1],org[1]],color='#e5b57c',alpha=.4)
    ax.set_title(title+f"\n기하 차폐 합계 약 {hit['blocked_deg']:.1f}°",fontsize=13)
    ax.set(xlim=(-125,125),ylim=(-100,100),xlabel='전후 X [mm] · +가 전방',ylabel='좌우 Y [mm]')
    ax.set_aspect('equal');ax.grid(alpha=.2)
fig.suptitle('라이다 높이의 수평 단면 비교 — 차체 중심=축간 중점, +Y=좌측',fontsize=16)
fig.tight_layout();fig.savefig(OUT/'scan_comparison.png',dpi=170)
(OUT/'sections.json').write_text(json.dumps(result,ensure_ascii=False),encoding='utf-8')
