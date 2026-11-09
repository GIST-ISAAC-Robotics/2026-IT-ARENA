import sys, numpy as np
sys.path.insert(0, '.')
import analyze_stl as A
h, n, t, a = A.load(A.STL)
def clus(level, axis=1, wide_axis=0, gap=8.0):
    segs = A.slice_xy(t, axis, level)
    if not segs: return []
    others=[i for i in range(3) if i!=axis]
    pts=np.concatenate(segs)[:, others]
    w = others.index(wide_axis)
    r=np.round(pts[:,w],2); xs=np.unique(r)
    gaps=np.where(np.diff(xs)>gap)[0]
    out=[]
    for g in np.split(xs,gaps+1):
        p=pts[(r>=g.min())&(r<=g.max())]
        out.append((p.min(0).round(2).tolist(), p.max(0).round(2).tolist()))
    return out
print('bottom levels (Y):')
for y in np.arange(-429.4, -395, 1.0):
    c = clus(y); print(round(y,1), [(x[0][0], round(x[1][0]-x[0][0],2), round(x[1][1]-x[0][1],2)) for x in c])
print('top levels (Y):')
for y in np.arange(-45, 35.5, 2.5):
    c = clus(y); print(round(y,1), [(x[0][0], round(x[1][0]-x[0][0],2), x[0][1], x[1][1]) for x in c])
print('vertical slices X=const (Y,Z clusters by Y):')
for x in (-301.72, -250, -150, -100, -9.22, 80, 200, 283.28):
    c = clus(x, axis=0, wide_axis=1, gap=5.0); print(x, [(q[0], q[1]) for q in c])
