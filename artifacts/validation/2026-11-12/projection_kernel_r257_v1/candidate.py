"""원 파일의 검증/객체 생성을 유지하고 scalar 투영 루프만 교체한 후보."""
import ctypes
import importlib.util
from pathlib import Path
import numpy as np

A=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('r257_original_geometry',A/'original_geometry.py')
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
lib=ctypes.CDLL(str(A/'build/libprojection.so'))
ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
lib.arena_project.argtypes=[ctypes.c_int64,ctypes.c_int64,ptr,ptr,ptr,ptr,ptr,ptr]
lib.arena_project.restype=ctypes.c_int

# 기존 검증문을 그대로 실행한다. 공통 소스/모듈은 수정하지 않는다.
source=(A/'original_geometry.py').read_text()
validation=source[source.index('    if type(scan)'):source.index('    for index, (distance')]
namespace=dict(vars(original))
exec('def validate(scan, transforms, now, config, epoch, mask):\n'+validation+'    return sx,sy,yaw,limits\n',namespace)

def geometry(scan,transforms,now,config,epoch,mask):
    sx,sy,yaw,limits=namespace['validate'](scan,transforms,now,config,epoch,mask)
    n=len(scan.ranges_m)
    params=np.array([sx,sy,yaw,scan.range_min_m,scan.range_max_m,limits[0]],dtype=np.float64)
    masks=np.asarray(limits[1],dtype=np.float64).reshape(-1,2)
    if len(masks)>16384:
        yield from baseline_geometry(scan,transforms,now,config,epoch,mask)
        return
    out=np.empty((n,7),dtype=np.float64)
    status=lib.arena_project(n,len(masks),np.asarray(scan.ranges_m),np.asarray(scan.angles_rad),
                            np.ascontiguousarray(transforms,dtype=np.float64),params,masks,out)
    if status:raise ValueError('invalid projection geometry')
    for i,(ox,oy,ux,uy,hx,hy,kind) in enumerate(out.tolist()):
        label=('return','unknown','masked')[int(kind)]
        yield i,(ox,oy),(ux,uy),(hx,hy) if kind==0 else None,label

baseline_geometry=original._geometry
original._geometry=geometry
native_motion_scene=original.native_motion_scene
native_motion_points=original.native_motion_points
