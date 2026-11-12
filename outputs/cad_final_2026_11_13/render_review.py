from pathlib import Path
import json, sys
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

OUT=Path(__file__).parent
rows=json.loads((OUT/'assembly.json').read_text(encoding='utf-8'))
byid={r['id']:r for r in rows}
groups=[r for r in rows if r['depth']==1 and r['bbox'] is not None]

def color(name):
    if '타이어' in name or '바퀴' in name: return (.13,.15,.19)
    if 'Jetson Orin' in name: return (.36,.43,.43)
    if 'PM_KAIST' in name: return (.12,.56,.39)
    if '배터리' in name: return (.7,.3,.28)
    if 'RPLIDAR_C1_official' in name: return (.20,.25,.32)
    if '기둥' in name or '트러스' in name: return (.86,.47,.18)
    if '카메라' in name or 'Camera_' in name: return (.26,.42,.63)
    if 'MD20A' in name or 'Adapter' in name or 'ESP32' in name or 'IMU_' in name: return (.08,.55,.50)
    return (.66,.72,.80)

def actor(row):
    p=OUT/f"mesh_{row['id']}.npz"
    data=np.load(p); v=data['vertices']; f=data['faces']
    v=np.c_[-v[:,2]-64.0,21.2-v[:,0],v[:,1]+32.2]
    pts=vtk.vtkPoints(); pts.SetData(numpy_to_vtk(v,deep=True))
    cells=vtk.vtkCellArray(); cells.SetCells(len(f),numpy_to_vtkIdTypeArray(np.c_[np.full(len(f),3),f].astype(np.int64).ravel(),deep=True))
    pd=vtk.vtkPolyData(); pd.SetPoints(pts); pd.SetPolys(cells)
    norm=vtk.vtkPolyDataNormals(); norm.SetInputData(pd); norm.SetFeatureAngle(50)
    mapper=vtk.vtkPolyDataMapper(); mapper.SetInputConnection(norm.GetOutputPort())
    a=vtk.vtkActor(); a.SetMapper(mapper); a.GetProperty().SetColor(color(row['name']))
    a.GetProperty().SetAmbient(.25); a.GetProperty().SetDiffuse(.75)
    return a

def render(stem, direction, up, include=None):
    ren=vtk.vtkRenderer(); ren.SetBackground(.965,.975,.99)
    win=vtk.vtkRenderWindow(); win.SetOffScreenRendering(1); win.SetSize(1400,1250); win.AddRenderer(ren)
    for r in groups:
        if include is None or r['id'] in include: ren.AddActor(actor(r))
    cam=ren.GetActiveCamera(); cam.ParallelProjectionOn(); cam.SetPosition(*direction); cam.SetViewUp(*up); cam.SetFocalPoint(0,0,0)
    ren.ResetCamera(); cam.Zoom(1.10)
    win.Render()
    cap=vtk.vtkWindowToImageFilter(); cap.SetInput(win); cap.Update()
    wr=vtk.vtkPNGWriter(); wr.SetFileName(str(OUT/f'{stem}.png')); wr.SetInputConnection(cap.GetOutputPort()); wr.Write()
    win.Finalize()

if __name__=='__main__':
    render('cad_isometric',(1,-1,0.75),(0,0,1))
    render('cad_top',(0,0,1),(1,0,0))
    render('cad_side',(0,-1,0),(0,0,1))
    render('cad_front',(1,0,0),(0,0,1))
