"""Read-only transplant in old STEP. Axes: longitudinal=Y, vertical=Z, mm."""
import sys, json, hashlib
from pathlib import Path
from OCP.gp import gp_Trsf, gp_Vec
from OCP.TopLoc import TopLoc_Location
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'lidar_mount_cad_reference'))
from inspect_reference import (bbox, name, TDocStd_Document, TCollection_ExtendedString,
    STEPCAFControl_Reader, IFSelect_RetDone, XCAFDoc_DocumentTool, Tool, TDF_LabelSequence, TDF_Label)

def run(path):
    with path.open('rb') as f:
        assert hashlib.file_digest(f, 'sha256').hexdigest()=='a93d422d059e38846a3be11174f664cd3dbfc6b794022ea25373d6afbab8b69e'
    doc=TDocStd_Document(TCollection_ExtendedString('XmlXCAF')); r=STEPCAFControl_Reader()
    assert r.ReadFile(str(path))==IFSelect_RetDone and r.Transfer(doc)
    st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); roots=TDF_LabelSequence(); st.GetFreeShapes(roots)
    children=TDF_LabelSequence(); Tool.GetComponents_s(roots.Value(1),children,False)
    parts={}
    for i in range(1,children.Length()+1):
        occ=children.Value(i); ref=TDF_Label(); Tool.GetReferredShape_s(occ,ref)
        idx=name(ref).split('_')[0]
        if idx in ['01','02','10','12']: parts[idx]=Tool.GetShape_s(occ)
    base=parts['12']; b=bbox(base); out=[]
    for x,h in [(30,75),(50,75),(65,75),(50,95),(65,95),(50,115)]:
        tr=gp_Trsf(); tr.SetTranslation(gp_Vec(0, x-(b['min'][1]+b['max'][1])/2, h-29.8-b['min'][2]))
        shape=base.Moved(TopLoc_Location(tr)); row={'x_mm':x,'optical_z_mm':h,'common_mm3':{}}
        for k,label in [('01','battery'),('02','motor_encoder'),('10','servo')]:
            op=BRepAlgoAPI_Common(shape,parts[k]); assert op.IsDone()
            g=GProp_GProps(); BRepGProp.VolumeProperties_s(op.Shape(),g)
            row['common_mm3'][label]=round(g.Mass(),2)
        out.append(row); print(row,flush=True)
    (HERE/'solid_intersections.json').write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__': run(Path(sys.argv[1]).resolve())
