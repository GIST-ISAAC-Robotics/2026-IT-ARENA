"""기본 NumPy 유지. 명시 선택한 별도 자식에서만 두 수치 커널을 사용한다."""
import numpy as np

_backend=None


def configure(mode):
    global _backend
    if mode=='python':
        _backend=None
    elif mode=='native_v1':
        from arena_autonomy import _numeric_kernels
        if _numeric_kernels.ABI_VERSION!=1:raise RuntimeError('numeric kernel ABI differs')
        _backend=_numeric_kernels
    else:raise ValueError('registered numeric backend required')


def depth(part,chosen,origins,directions,indices,valid,depth,padding,cycle):
    if _backend is None:return None
    data=[np.ascontiguousarray(v,dtype=dtype) for v,dtype in zip(
        (part,chosen,origins,directions,indices,valid,depth,padding),
        (np.float64,np.int64,np.float64,np.float64,np.int64,np.uint8,np.float64,np.float64))]
    rows,width=data[1].shape;rays=len(data[2])
    if data[0].shape!=(rows,2) or data[2].shape!=(rays,2) or data[3].shape!=(rays,2) or any(a.shape!=(rays,) for a in data[4:]):
        raise ValueError('native depth dimensions differ')
    out=np.empty(rows,dtype=np.uint8)
    _backend.depth(rows,width,rays,-1 if cycle is None else int(cycle),*data,out)
    return out.astype(bool)


def masked(part,chosen,origins,directions,indices,status,depth,padding):
    if _backend is None:return None
    if not hasattr(_backend,'masked'):
        raise RuntimeError('native masked kernel missing; rebuild arena_autonomy before explicit native use')
    data=[np.ascontiguousarray(v,dtype=dtype) for v,dtype in zip(
        (part,chosen,origins,directions,indices,status,depth,padding),
        (np.float64,np.int64,np.float64,np.float64,np.int64,np.uint8,np.float64,np.float64))]
    rows,width=data[1].shape;rays=len(data[2])
    if data[0].shape!=(rows,2) or data[2].shape!=(rays,2) or data[3].shape!=(rays,2) or any(a.shape!=(rays,) for a in data[4:]):
        raise ValueError('native masked dimensions differ')
    out=np.empty(rows,dtype=np.uint8)
    _backend.masked(rows,width,rays,*data,out)
    return out.astype(bool)


def classification_enabled():
    return _backend is not None


def support_classes(part,chosen,origins,directions,indices,status,depth,padding,cycle):
    if _backend is None:return None
    if not hasattr(_backend,'classes'):
        raise RuntimeError('native classification kernel missing; rebuild arena_autonomy')
    data=[np.ascontiguousarray(v,dtype=dtype) for v,dtype in zip(
        (part,chosen,origins,directions,indices,status,depth,padding),
        (np.float64,np.int64,np.float64,np.float64,np.int64,np.uint8,np.float64,np.float64))]
    rows,width=data[1].shape;rays=len(data[2])
    if data[0].shape!=(rows,2) or data[2].shape!=(rays,2) or data[3].shape!=(rays,2) or any(a.shape!=(rays,) for a in data[4:]):
        raise ValueError('native classification dimensions differ')
    out=np.empty(rows,dtype=np.uint8)
    _backend.classes(rows,width,rays,-1 if cycle is None else int(cycle),*data,out)
    return out


def polygon(points,polygon):
    if _backend is None:return None
    area=abs(float(np.sum(polygon[:,0]*np.roll(polygon[:,1],-1)-polygon[:,1]*np.roll(polygon[:,0],-1))))
    q=np.ascontiguousarray(points,dtype=np.float64);p=np.ascontiguousarray(polygon,dtype=np.float64)
    if q.shape!=(len(q),2) or p.shape!=(len(p),2):raise ValueError('Nx2 polygon kernel inputs required')
    out=np.empty(len(q),dtype=np.float64)
    _backend.polygon(len(q),len(p),int(area>1e-15),q,p,out)
    return out
