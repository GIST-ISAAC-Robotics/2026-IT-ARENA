from compare_step import *
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
def delta(a,b,stem):
    op=BRepAlgoAPI_Cut(a,b);op.SetRunParallel(False);op.Build()
    if not op.IsDone():print(stem,'failed',flush=True);return
    s=op.Shape();v=GProp_GProps();BRepGProp.VolumeProperties_s(s,v)
    for i,solid in enumerate(exp(s,TopAbs_SOLID)):
        vol=GProp_GProps();BRepGProp.VolumeProperties_s(solid,vol)
        if abs(vol.Mass())<.002:continue
        bb=Bnd_Box();BRepBndLib.AddOptimal_s(solid,bb,False,False)
        bounds=box(list(bb.CornerMin().Coord())+list(bb.CornerMax().Coord()))
        print(stem,i,round(vol.Mass(),3),bounds.round(4).tolist(),flush=True)
        v,f=mesh(solid,.04);np.savez_compressed(OUT/f"{stem}_{i}.npz",v=v,f=f)
a=shape(OLD,2092);b=shape(OUT,2098)
delta(b,a,"lower_added");delta(a,b,"lower_removed")
a=shape(OLD,2385); b=list(exp(shape(OUT,2395),TopAbs_SOLID))[1]
delta(b,a,"truss_added");delta(a,b,"truss_removed")
