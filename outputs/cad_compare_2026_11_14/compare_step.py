from pathlib import Path
from collections import Counter
import json,math,hashlib
import numpy as np
from OCP.TopoDS import TopoDS_Shape,TopoDS
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_FACE,TopAbs_SOLID,TopAbs_EDGE,TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.gp import gp_Pln,gp_Pnt,gp_Dir
from OCP.TopLoc import TopLoc_Location

OUT=Path(__file__).parent
OLD=OUT.parent/'cad_final_2026_11_13'
def get(p,n):return json.loads((p/n).read_text(encoding='utf-8'))
def save(n,d):(OUT/n).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def shape(p,idx):
    s=TopoDS_Shape()
    assert BRepTools.Read_s(s,str(p/f'part_{idx}.brep'),BRep_Builder())
    return s
def exp(s,t):
    e=TopExp_Explorer(s,t)
    while e.More():
        yield e.Current()
        e.Next()
def xyz(v):
    a,b,c=np.asarray(v).T
    return np.array([-c-64,21.2-a,b+32.2]).T
def box(b):
    lo,hi=xyz([b[:3],b[3:]])
    return np.r_[np.minimum(lo,hi),np.maximum(lo,hi)]

def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def read_groups(p):
    g=get(p,'geometry.json')
    for r in g:
        r['vehicle_bbox_mm']=box(r['bbox']).tolist()
    return g

def matchparts(a,b):
    used=set()
    for old in a:
        score=lambda r:np.linalg.norm(np.asarray(r['bbox'])-old['bbox'])
        pool=[r for r in b if r['id'] not in used and r['name']==old['name']]
        new=min(pool,key=score) if pool else None
        if new:used.add(new['id'])
        yield old,new
    for new in b:
        if new['id'] not in used:yield None,new

def compare(a,b):
    pairs=[]
    for old,new in matchparts(a,b):
        if old is None or new is None:
            pairs.append(dict(old=old,new=new,status='added' if old is None else 'removed'))
            continue
        ob=OLD/f"part_{old['id']}.brep"; nb=OUT/f"part_{new['id']}.brep"
        identical=sha(ob)==sha(nb)
        bd=np.asarray(new['vehicle_bbox_mm'])-old['vehicle_bbox_mm']
        dv=new['volume_mm3']-old['volume_mm3']
        s=shape(OLD,old['id'])
        p=GProp_GProps();BRepGProp.SurfaceProperties_s(s,p)
        df=new['faces']-sum(1 for _ in exp(s,TopAbs_FACE))
        da=new['surface_mm2']-p.Mass()
        result=dict(old=old['id'],new=new['id'],name=new['name'],brep_byte_exact=identical,bbox_delta_mm=bd.tolist(),
                    volume_delta_mm3=dv,surface_delta_mm2=da,face_delta=df,solids_delta=new['solids']-old['solids'])
        result['status']='same_brep' if identical else 'same_metrics' if max(abs(bd))<1e-5 and abs(dv)<.005 and abs(da)<.001 and not df and not result['solids_delta'] else 'changed'
        pairs.append(result)
        if result['status']=='changed': print('DELTA',old['id'],new['id'],new['name'],np.round(bd,4).tolist(),round(dv,4),round(da,4),df,flush=True)
    save('part_diff.json',pairs)
    print(Counter(r['status'] for r in pairs),flush=True)

def cut(s,z):
    op=BRepAlgoAPI_Section(s,gp_Pln(gp_Pnt(0,z-32.2,0),gp_Dir(0,1,0)),False);op.Build()
    assert op.IsDone()
    seg=[]
    for e in exp(op.Shape(),TopAbs_EDGE):
        from OCP.GeomAbs import GeomAbs_Line
        c=BRepAdaptor_Curve(TopoDS.Edge(e))
        ts=np.linspace(c.FirstParameter(),c.LastParameter(),2 if c.GetType()==GeomAbs_Line else 97)
        seg.append(xyz([c.Value(float(t)).Coord() for t in ts])[:,:2].tolist())
    return seg

def mesh(s,precision=.1):
    BRepMesh_IncrementalMesh(s,precision,False,.25,False)
    vv=[];ff=[];off=0
    for f in exp(s,TopAbs_FACE):
        f=TopoDS.Face(f); l=TopLoc_Location(); t=BRep_Tool.Triangulation_s(f,l)
        if t is None:continue
        loc=l.Transformation()
        vv.extend(t.Node(i).Transformed(loc).Coord() for i in range(1,t.NbNodes()+1))
        for i in range(1,t.NbTriangles()+1):
            a,b,c=t.Triangle(i).Get()
            if f.Orientation()==TopAbs_REVERSED:b,c=c,b
            ff.append([a+off-1,b+off-1,c+off-1])
        off+=t.NbNodes()
    return xyz(vv),np.asarray(ff)

if __name__=='__main__':
    prev,new=read_groups(OLD),read_groups(OUT)
    compare(prev,new)
    sections={}
    for vv,p,gg in [('old',OLD,prev),('new',OUT,new)]:
        cuts={}
        band=(122.,140.6)
        pool=[(r,shape(p,r['id'])) for r in gg if r['vehicle_bbox_mm'][2]<=band[1] and r['vehicle_bbox_mm'][5]>=band[0]]
        for h in [122.,125.,128.,130.,131.6,132.6,133.6,134.6,135.6,136.6,137.6,140.6]:
            cuts[str(h)]=[
                dict(id=r['id'],name=r['name'],lines=g)
                for r,s in pool if 'RPLIDAR_C1_official' not in r['name'] and r['vehicle_bbox_mm'][2]<h<r['vehicle_bbox_mm'][5]
                    and (g:=cut(s,h))
            ]
            print(vv,h,[(r['id'],len(r['lines'])) for r in cuts[str(h)]],flush=True)
        sections[vv]=cuts
    save('section_comparison.json',sections)
