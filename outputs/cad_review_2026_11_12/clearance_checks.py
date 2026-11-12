from analyze_geometry import *
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepCheck import BRepCheck_Analyzer

pairs=[(1859,1841),(1859,1845),(2087,1841),(2087,2368),(2315,2373),(13,1851),(13,2372),(2093,2370)]
cache={i:readshape(i) for pair in pairs for i in pair}
out=[]
for a,b in pairs:
    d=BRepExtrema_DistShapeShape(cache[a],cache[b]);d.Perform()
    item={'a':a,'b':b,'distance_mm':d.Value() if d.IsDone() else None}
    if item['distance_mm'] is not None and item['distance_mm']<1e-4:
        c=BRepAlgoAPI_Common(cache[a],cache[b]);c.Build()
        if c.IsDone():
            p=GProp_GProps(); BRepGProp.VolumeProperties_s(c.Shape(),p)
            item.update(common_solids=count(c.Shape(),TopAbs_SOLID),common_volume_mm3=p.Mass())
    print(item,flush=True);out.append(item)
(OUT/'clearances.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
