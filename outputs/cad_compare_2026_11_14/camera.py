from compare_step import *
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
def bbox(s):
    b=Bnd_Box();BRepBndLib.AddOptimal_s(s,b,False,False)
    return box(list(b.CornerMin().Coord())+list(b.CornerMax().Coord()))
data={}
for p,idx in [(OLD,2093),(OUT,2099),(OLD,2087),(OUT,2093),(OLD,10),(OUT,10)]:
    s=shape(p,idx)
    rec=[]
    for face in exp(s,TopAbs_FACE):
        a=BRepAdaptor_Surface(TopoDS.Face(face))
        if a.GetType()!=GeomAbs_Cylinder:continue
        c=a.Cylinder()
        if abs(c.Axis().Direction().Z())<.99:continue
        b=bbox(face)
        if '2093' in str(idx) or idx==2099:
            if abs(c.Location().X()-21.2)<.01 and c.Radius()>2:
                rec.append(dict(radius=c.Radius(),axis_location=xyz(c.Location().Coord()).tolist(),bbox=b.tolist()))
    data[f"{p.name}:{idx}"]=rec
    if rec: print(idx,rec,flush=True)
a=shape(OLD,2370); b=shape(OUT,2376)
for t,u,k in [(b,a,'added'),(a,b,'removed')]:
    o=BRepAlgoAPI_Cut(t,u);o.Build()
    plus=[]
    if o.IsDone():
        for s in exp(o.Shape(),TopAbs_SOLID):
            v=GProp_GProps();BRepGProp.VolumeProperties_s(s,v)
            if abs(v.Mass())<.001:continue
            bb=bbox(s)
            plus.append(dict(volume_mm3=v.Mass(),bbox=bb.tolist()))
            print('camera bracket',k,round(v.Mass(),3), bb.round(4),flush=True)
    data['camera_bracket_'+k]=plus
save('camera_mount_check.json',data)
