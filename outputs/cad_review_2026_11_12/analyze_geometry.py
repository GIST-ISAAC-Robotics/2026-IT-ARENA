from pathlib import Path
import json, math
from OCP.TopoDS import TopoDS_Shape,TopoDS
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_SOLID,TopAbs_EDGE
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.gp import gp_Pln,gp_Pnt,gp_Dir
import numpy as np

OUT=Path(__file__).parent
rows=json.loads((OUT/'assembly.json').read_text(encoding='utf-8'))
groups=[r for r in rows if r['depth']==1]

def readshape(idx):
    s=TopoDS_Shape(); assert BRepTools.Read_s(s,str(OUT/f'part_{idx}.brep'),BRep_Builder())
    return s

def count(s,kind):
    ex=TopExp_Explorer(s,kind); n=0
    while ex.More(): n+=1; ex.Next()
    return n

def xyz(v): return [-v[2]-64,21.2-v[0],v[1]+32.2]

def cylinders(s):
    out=[]; ex=TopExp_Explorer(s,TopAbs_FACE)
    while ex.More():
        f=TopoDS.Face(ex.Current()); a=BRepAdaptor_Surface(f)
        if a.GetType()==GeomAbs_Cylinder:
            c=a.Cylinder(); out.append({'radius':c.Radius(),'axis_origin':list(c.Location().Coord()),'axis_dir':list(c.Axis().Direction().Coord()),'u':[a.FirstUParameter(),a.LastUParameter()],'v':[a.FirstVParameter(),a.LastVParameter()]})
        ex.Next()
    return out

def section(s,height):
    op=BRepAlgoAPI_Section(s,gp_Pln(gp_Pnt(0,height-32.2,0),gp_Dir(0,1,0)),False)
    op.Build(); seg=[]
    if not op.IsDone(): return seg
    ex=TopExp_Explorer(op.Shape(),TopAbs_EDGE)
    while ex.More():
        a=BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        p1,p2=a.FirstParameter(),a.LastParameter()
        ts=np.linspace(p1,p2,40)
        seg.append([xyz(a.Value(float(t)).Coord()) for t in ts])
        ex.Next()
    return seg

if __name__=='__main__':
    result=[]
    for r in groups:
        s=readshape(r['id']); p=GProp_GProps(); BRepGProp.VolumeProperties_s(s,p)
        q=dict(r); q['volume_mm3']=p.Mass(); q['uniform_centroid_mm']=xyz(p.CentreOfMass().Coord()); q['solids']=count(s,TopAbs_SOLID)
        if r['id'] in [1858,2088,2089,2090,2091,2092,2093,2370,2371,2372,2373,2374]: q['cylinders']=cylinders(s)
        result.append(q)
        print(r['id'],r['name'],round(q['volume_mm3'],1),q['uniform_centroid_mm'],flush=True)
    (OUT/'geometry.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
