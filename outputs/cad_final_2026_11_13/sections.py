"""확정본의 C1 공칭 광축과 위아래3mm 단면. 외부 물체/배선은 아직 포함하지 않는다."""
from analyze_geometry import *
from OCP.GeomAbs import GeomAbs_Line


def cuts(shape,height):
    op=BRepAlgoAPI_Section(shape,gp_Pln(gp_Pnt(0,height-32.2,0),gp_Dir(0,1,0)),False)
    op.Build()
    assert op.IsDone()
    lines=[]
    ex=TopExp_Explorer(op.Shape(),TopAbs_EDGE)
    while ex.More():
        curve=BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        ts=np.linspace(curve.FirstParameter(),curve.LastParameter(),2 if curve.GetType()==GeomAbs_Line else 65)
        lines.append([xyz(curve.Value(float(t)).Coord())[:2] for t in ts])
        ex.Next()
    return lines


if __name__=='__main__':
    assert not (OUT/'sections.json').exists()
    result={}
    for height in (131.6,134.6,137.6):
        entries=[]
        for r in groups:
            b=r['bbox']
            if r['id']==1858 or not b[1]<height-32.2<b[4]:continue
            lines=cuts(readshape(r['id']),height)
            if lines:entries.append(dict(id=r['id'],name=r['name'],lines=lines))
        result[str(height)]=entries
        print(height,[(r['id'],len(r['lines'])) for r in entries],flush=True)
    (OUT/'sections.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
