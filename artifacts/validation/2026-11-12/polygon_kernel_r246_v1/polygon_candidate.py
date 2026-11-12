"""경계/면적 판정 불변 비교용 다각형 거리. 공통 소스에 연결하지 않는다."""
import ctypes
import math
from pathlib import Path
import numpy as np

_library=ctypes.CDLL(str(Path(__file__).with_name('libdepth.so')))
_call=_library.arena_polygon_distance
_call.argtypes=[ctypes.c_int64,ctypes.c_int64,ctypes.c_int]+[ctypes.c_void_p]*3
_call.restype=ctypes.c_int


def polygon_distance(points,polygon):
    if not len(polygon):return np.full(len(points),math.inf)
    if len(polygon)==1:return np.linalg.norm(points-polygon[0],axis=1)
    # 동일 합산 순서와 면적 경계. FP 결합/fast math는 사용하지 않는다.
    area=abs(float(np.sum(polygon[:,0]*np.roll(polygon[:,1],-1)-polygon[:,1]*np.roll(polygon[:,0],-1))))
    q=np.ascontiguousarray(points,dtype=np.float64);p=np.ascontiguousarray(polygon,dtype=np.float64)
    if q.shape!=(len(q),2) or p.shape!=(len(p),2):raise ValueError('Nx2 polygon kernel inputs required')
    out=np.empty(len(q),dtype=np.float64)
    code=_call(len(q),len(p),area>1e-15,q.ctypes.data,p.ctypes.data,out.ctypes.data)
    if code:raise RuntimeError('native polygon rejected input')
    return out
