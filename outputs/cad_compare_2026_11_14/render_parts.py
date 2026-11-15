from compare_step import *
import vtk
from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray
def actor(p,color,opacity=1.):
    d=np.load(p);v=d['v'];f=d['f']
    pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(v,deep=True))
    cells=vtk.vtkCellArray();cells.SetCells(len(f),numpy_to_vtkIdTypeArray(np.c_[np.full(len(f),3),f].astype(np.int64).ravel(),deep=True))
    pd=vtk.vtkPolyData();pd.SetPoints(pts);pd.SetPolys(cells)
    norm=vtk.vtkPolyDataNormals();norm.SetInputData(pd);norm.SetFeatureAngle(50)
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(norm.GetOutputPort())
    a=vtk.vtkActor();a.SetMapper(mapper);a.GetProperty().SetColor(color);a.GetProperty().SetOpacity(opacity);a.GetProperty().SetAmbient(.44)
    return a
def draw(ver,view,d,z=126,zoom=36):
    ren=vtk.vtkRenderer();ren.SetBackground(.095,.12,.17)
    win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(850,560);win.AddRenderer(ren)
    for p in sorted(OUT.glob('old_1858_*.npz')):
        i=int(p.stem.split('_')[-1])
        color=(.68,.7,.73) if i==20 else (.67,.51,.25) if i==18 else (.38,.43,.46)
        ren.AddActor(actor(p,color))
    for grp,c in [('old_2385' if ver=='old' else 'new_2395',(.2,.55,.76)),('old_2092' if ver=='old' else 'new_2098',(.2,.55,.76)),('old_2089' if ver=='old' else 'new_2097',(.43,.46,.5))]:
        for p in OUT.glob(grp+'_*.npz'):ren.AddActor(actor(p,c))
    cam=ren.GetActiveCamera();cam.ParallelProjectionOn();cam.SetPosition(*(np.array([75.4,0,z])+np.array(d)*250));cam.SetViewUp(0,0,1);cam.SetFocalPoint(75.4,0,z);cam.SetParallelScale(zoom)
    ren.ResetCameraClippingRange()
    win.Render();cap=vtk.vtkWindowToImageFilter();cap.SetInput(win);cap.Update()
    wr=vtk.vtkPNGWriter();wr.SetFileName(str(OUT/f'c1_{ver}_{view}.png'));wr.SetInputConnection(cap.GetOutputPort());wr.Write()
    win.Finalize()
for ver in ('old','new'):
    for label,d in [('right',(0,-1,0)),('left',(0,1,0)),('front',(1,0,0)),('detail',(1,1,-.45))]:draw(ver,label,d)
