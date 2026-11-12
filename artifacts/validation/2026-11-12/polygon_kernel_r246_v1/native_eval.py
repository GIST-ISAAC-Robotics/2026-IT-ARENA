"""검사 후보의 작은 C ABI. .so가 없으면 실패하며 자동 빌드/대체하지 않는다."""
import ctypes
from pathlib import Path
import numpy as np

_library=ctypes.CDLL(str(Path(__file__).with_name('libdepth.so')))
_call=_library.arena_depth_evaluate
_call.argtypes=[ctypes.c_int64]*4+[ctypes.c_void_p]*9
_call.restype=ctypes.c_int


def evaluate_native(part,chosen,origins,directions,indices,valid,depth,padding,cycle):
    data=[np.ascontiguousarray(v,dtype=dtype) for v,dtype in zip(
        (part,chosen,origins,directions,indices,valid,depth,padding),
        (np.float64,np.int64,np.float64,np.float64,np.int64,np.uint8,np.float64,np.float64))]
    points,selection,o,d,idx,v,z,pad=data
    rows,width=selection.shape;rays=len(o)
    if points.shape!=(rows,2) or o.shape!=(rays,2) or d.shape!=(rays,2) or any(a.shape!=(rays,) for a in (idx,v,z,pad)):
        raise ValueError('native depth dimensions differ')
    out=np.empty(rows,dtype=np.uint8)
    result=_call(rows,width,rays,-1 if cycle is None else int(cycle),
        *[a.ctypes.data for a in data],out.ctypes.data)
    if result:raise RuntimeError('native depth rejected input: '+str(result))
    return out.astype(bool)
