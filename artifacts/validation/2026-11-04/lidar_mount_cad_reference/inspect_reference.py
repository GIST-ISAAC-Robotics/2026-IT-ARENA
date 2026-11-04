"""구형 Fusion 조립본 읽기 전용: 최상위 인스턴스, 임시 프레임·트레이의 솔리드/원통면 형상. 단위 mm."""
import hashlib
import json
import sys
from pathlib import Path
from OCP.Bnd import Bnd_Box
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool as Tool

def rnd(seq): return [round(v,3) for v in seq]
def bbox(shape):
    box=Bnd_Box()
    BRepBndLib.AddOptimal_s(shape,box,False,False)
    vals=box.Get()
    return {'min':rnd(vals[:3]),'max':rnd(vals[3:]),'size':rnd([vals[i+3]-vals[i] for i in range(3)])}
def name(lbl):
    a=TDataStd_Name()
    return a.Get().ToExtString().strip() if lbl.FindAttribute(TDataStd_Name.GetID_s(),a) else ''
def scan_solid(shape):
    edges=[]; ex=TopExp_Explorer(shape,TopAbs_FACE)
    while ex.More():
        f=TopoDS.Face_s(ex.Current()); ad=BRepAdaptor_Surface(f)
        if ad.GetType()==GeomAbs_Cylinder:
            c=ad.Cylinder(); axis=c.Axis().Direction()
            edges.append({'diam':round(2*c.Radius(),3),'dir':rnd(axis.Coord()),'bounds':bbox(f)})
        ex.Next()
    return dict(bounds=bbox(shape),cylinders=edges)
def run(path):
    doc=TDocStd_Document(TCollection_ExtendedString('XmlXCAF')); r=STEPCAFControl_Reader()
    assert r.ReadFile(str(path))==IFSelect_RetDone
    assert r.Transfer(doc)
    st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); roots=TDF_LabelSequence(); st.GetFreeShapes(roots)
    assert roots.Length()==1
    children=TDF_LabelSequence(); Tool.GetComponents_s(roots.Value(1),children,False)
    records=[]
    for i in range(1,children.Length()+1):
        occ=children.Value(i);ref=TDF_Label(); Tool.GetReferredShape_s(occ,ref); n=name(ref)
        shape=Tool.GetShape_s(occ); item=dict(label=n,instance=name(occ),bounds=bbox(shape))
        if n.startswith(('30_','31_','32_')):
            solids=[];ex=TopExp_Explorer(shape,TopAbs_SOLID)
            while ex.More():
                solids.append(scan_solid(ex.Current()));ex.Next()
            item['solids']=solids
        records.append(item)
    with path.open('rb') as f: digest=hashlib.file_digest(f,'sha256').hexdigest()
    mins=[min(r['bounds']['min'][i] for r in records) for i in range(3)]
    maxs=[max(r['bounds']['max'][i] for r in records) for i in range(3)]
    full={'min':mins,'max':maxs,'size':rnd([b-a for a,b in zip(mins,maxs)]),
          'extrema':{k:[[r['label'] for r in records if r['bounds'][k][i]==vals[i]] for i in range(3)] for k,vals in [('min',mins),('max',maxs)]}}
    return {'step_basename':path.name,'bytes':path.stat().st_size,'sha256':digest,'unit':'mm','axes':'native STEP assembly axes','assembly_bounds':full,'top_level':records}
if __name__=='__main__':
    data=run(Path(sys.argv[1]).resolve())
    Path(__file__).with_name('assembly.json').write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print('top-level occurrences:',len(data['top_level']),'selected geometry:',sum('bounds' in it for it in data['top_level']))
