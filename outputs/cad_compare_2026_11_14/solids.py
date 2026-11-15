from compare_step import *
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common,BRepAlgoAPI_Cut
def bounding(s):
    bb=Bnd_Box();BRepBndLib.AddOptimal_s(s,bb,False,False)
    return box(list(bb.CornerMin().Coord())+list(bb.CornerMax().Coord()))
def summary(s):
    vol=GProp_GProps();BRepGProp.VolumeProperties_s(s,vol)
    return dict(bbox=bounding(s).tolist(),volume_mm3=vol.Mass(),center=xyz(vol.CentreOfMass().Coord()).tolist(),
                faces=sum(1 for f in exp(s,TopAbs_FACE)))
entries=[]
for p,indices in [(OLD,[2385,1858,2384,2089,2091,2092]),(OUT,[2395,1864,2390,2097,2098,1841,1842,1843,1844,1845,1846])]:
    for idx in indices:
        top=shape(p,idx)
        ss=[summary(s) for s in exp(top,TopAbs_SOLID)]
        print(p.name,idx,len(ss),flush=True)
        if idx in [2385,2395,1858,1864,2384,2390,2089,2091,2097,2092,2098]:
            for i,r in enumerate(ss):
                if idx in (1858,1864) and r['bbox'][5]<120: continue
                print(i, [round(j,4) for j in r['bbox']],round(r['volume_mm3'],3),r['faces'],flush=True)
        entries.append(dict(revision='old' if p==OLD else 'new',id=idx,solids=ss))
save('solid_breakdown.json',entries)
def export(p,idx,wanted=None):
    top=shape(p,idx)
    if wanted is None: wanted=list(range(len(list(exp(top,TopAbs_SOLID)))))
    for i,s in enumerate(exp(top,TopAbs_SOLID)):
        if i in wanted:
            v,f=mesh(s,.07); np.savez_compressed(OUT/f"{'old' if p==OLD else 'new'}_{idx}_{i}.npz",v=v,f=f)
for p,idx in [(OLD,2385),(OUT,2395),(OLD,1858),(OUT,2098),(OLD,2092),(OUT,2097),(OLD,2089)]:
    export(p,idx)
