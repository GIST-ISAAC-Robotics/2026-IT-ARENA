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
SRC=Path(r'C:\Users\Jinhyeong\Downloads\11.12 RC카_패키징_작업본 확인용.step')
doc=TDocStd_Document(TCollection_ExtendedString('cad-review'))
reader=STEPCAFControl_Reader()
reader.SetNameMode(True)
reader.SetColorMode(True)
print('READ',flush=True)
assert int(reader.ReadFile(str(SRC)))==1
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
    rows.append(row); shapes[idx]=shape
    if depth<=2: print(idx,depth,row['name'],row['bbox'],flush=True)
    children=TDF_LabelSequence()
    if tool.GetComponents_s(definition,children):
        for i in range(1,children.Length()+1): visit(children.Value(i),world,idx,depth+1)

roots=TDF_LabelSequence(); tool.GetFreeShapes(roots)
for i in range(1,roots.Length()+1): visit(roots.Value(i),TopLoc_Location())
(OUT/'assembly.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'source.json').write_text(json.dumps({'source':str(SRC),'bytes':SRC.stat().st_size,'sha256':hashlib.file_digest(SRC.open('rb'),'sha256').hexdigest(),'roots':roots.Length(),'nodes':len(rows)},indent=2),encoding='utf-8')
print('TREE COMPLETE',len(rows),flush=True)

# Preserve top-level shapes locally for exact cuts and clearance checks.
groups=[r for r in rows if r['depth']==1]
for row in groups:
    BRepTools.Write_s(shapes[row['id']],str(OUT/f"part_{row['id']}.brep"))
print('BREP CACHE COMPLETE',flush=True)
for row in groups:
    idx=row['id']; shape=shapes[idx]
    BRepTools.Write_s(shape,str(OUT/f'part_{idx}.brep'))
    BRepMesh_IncrementalMesh(shape,0.25,False,0.35,True)
    vertices=[]; triangles=[]; offset=0
    exp=TopExp_Explorer(shape,TopAbs_FACE)
    while exp.More():
        face=TopoDS.Face(exp.Current()); location=TopLoc_Location()
        tri=BRep_Tool.Triangulation_s(face,location)
        if tri is not None:
            tr=location.Transformation()
            vertices.extend([tri.Node(i).Transformed(tr).Coord() for i in range(1,tri.NbNodes()+1)])
            for i in range(1,tri.NbTriangles()+1):
                a,b,c=tri.Triangle(i).Get()
                if face.Orientation()==TopAbs_REVERSED: b,c=c,b
                triangles.append([offset+a-1,offset+b-1,offset+c-1])
            offset+=tri.NbNodes()
        exp.Next()
    np.savez_compressed(OUT/f'mesh_{idx}.npz',vertices=np.array(vertices),faces=np.array(triangles))
    print('MESH',idx,len(vertices),len(triangles),flush=True)
print('DONE',flush=True)
