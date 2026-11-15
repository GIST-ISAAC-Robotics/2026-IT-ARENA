from pathlib import Path
import json, hashlib, time
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepTools import BRepTools
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
import numpy as np

OUT=Path(__file__).parent
SRC=OUT/'source.step'
assert hashlib.sha256(SRC.read_bytes()).hexdigest()=='da4180b907c8b8a5a59731c3422ffdec7c0ea346c57488e5378b3624bc52a626'
assert not (OUT/'assembly.json').exists(), 'do not overwrite completed extraction'
doc=TDocStd_Document(TCollection_ExtendedString('cad-review'))
reader=STEPCAFControl_Reader()
reader.SetNameMode(True)
reader.SetColorMode(True)
print('READ',flush=True)
assert int(reader.ReadFile("source.step"))==1
assert reader.Transfer(doc)
tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

def name(lab):
    n=TDataStd_Name()
    return n.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(),n) else ''

def bbox(shape):
    if shape.IsNull(): return None
    b=Bnd_Box(); BRepBndLib.AddOptimal_s(shape,b,False,False)
    return list(b.CornerMin().Coord())+list(b.CornerMax().Coord()) if not b.IsVoid() else None

rows=[]; shapes={}
def visit(lab, loc, parent=None, depth=0):
    ref=TDF_Label()
    isref=tool.GetReferredShape_s(lab,ref)
    definition=ref if isref else lab
    world=loc.Multiplied(tool.GetLocation_s(lab)) if isref else loc
    shape=tool.GetShape_s(definition).Moved(world)
    idx=len(rows)
    row={'id':idx,'parent':parent,'depth':depth,'name':name(definition),'instance':name(lab),'bbox':bbox(shape),'assembly':tool.IsAssembly_s(definition)}
    row['location']=list(world.Transformation().Value(i,j) for i in range(1,4) for j in range(1,5)); rows.append(row); shapes[idx]=shape
    if depth<=1: print(idx,depth,row['name'],row['bbox'],flush=True)
    children=TDF_LabelSequence()
    if tool.GetComponents_s(definition,children):
        for i in range(1,children.Length()+1): visit(children.Value(i),world,idx,depth+1)

roots=TDF_LabelSequence(); tool.GetFreeShapes(roots)
for i in range(1,roots.Length()+1): visit(roots.Value(i),TopLoc_Location())
(OUT/'assembly.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'source.json').write_text(json.dumps({'source':str(SRC),'bytes':SRC.stat().st_size,'sha256':hashlib.file_digest(SRC.open('rb'),'sha256').hexdigest(),'roots':roots.Length(),'nodes':len(rows)},indent=2),encoding='utf-8')
print('TREE COMPLETE',len(rows),flush=True)


from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_SOLID
def count(s,kind):
    ex=TopExp_Explorer(s,kind); n=0
    while ex.More(): n+=1; ex.Next()
    return n

groups=[r for r in rows if r['depth']==1 and r['bbox'] is not None]
for r in groups:
    s=shapes[r['id']]; vp=GProp_GProps(); ap=GProp_GProps()
    BRepGProp.VolumeProperties_s(s,vp)
    BRepGProp.SurfaceProperties_s(s,ap)
    r.update(volume_mm3=vp.Mass(),surface_mm2=ap.Mass(),solids=count(s,TopAbs_SOLID),faces=count(s,TopAbs_FACE))
    BRepTools.Write_s(s,str(OUT/f"part_{r['id']}.brep"))
    print("BREP",r["id"],r["faces"],flush=True)
(OUT/'geometry.json').write_text(json.dumps(groups,ensure_ascii=False,indent=2),encoding='utf-8')
print('DONE',flush=True)
