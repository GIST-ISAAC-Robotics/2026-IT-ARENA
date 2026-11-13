"""r467 최적화 사본에 점유 패딩 수치 커널만 더한다. 공통 소스는 변경하지 않는다."""
import ast
import hashlib
import json
from pathlib import Path
import shutil

A=Path(__file__).resolve().parent
SOURCE=A.parent/'current_structure_cost_r467_v1/optimization_v1/stage'
STAGE=A/'stage'
shutil.copytree(SOURCE,STAGE)
package=STAGE/'src/arena_autonomy/arena_autonomy'
known=package/'known_structure.py'
text=known.read_text(encoding='utf8')
old="""    for start in range(0, len(q), 64):
        if len(hits) and np.any(np.linalg.norm(q[start:start+64, None]-hits[None], axis=2) <= pad[valid]):
            return reject('current_known_occupied_padding')
"""
new="""    from arena_autonomy.numeric_kernels import current_mask_occupied
    occupied = current_mask_occupied(q, hits, pad[valid])
    if occupied is None:
        occupied = any(len(hits) and np.any(
            np.linalg.norm(q[start:start+64, None]-hits[None], axis=2) <= pad[valid])
            for start in range(0, len(q), 64))
    if occupied:
        return reject('current_known_occupied_padding')
"""
assert text.count(old)==1
known.write_text(text.replace(old,new),encoding='utf8',newline='\n')
numeric=package/'numeric_kernels.py'
text=numeric.read_text(encoding='utf8')
text+='''

def current_mask_occupied(queries,hits,padding):
    """현재 자기 가림 질의와 실제 반환의 기존 inclusive 패딩 충돌만 계산한다."""
    if _backend is None:return None
    if not hasattr(_backend,'current_mask_occupied'):
        raise RuntimeError('native current mask occupancy kernel missing; rebuild arena_autonomy')
    q=np.ascontiguousarray(queries,dtype=np.float64)
    h=np.ascontiguousarray(hits,dtype=np.float64)
    pad=np.ascontiguousarray(padding,dtype=np.float64)
    if q.shape!=(len(q),2) or h.shape!=(len(h),2) or pad.shape!=(len(h),):
        raise ValueError('native current mask occupancy dimensions differ')
    out=np.empty(1,dtype=np.uint8)
    _backend.current_mask_occupied(len(q),len(h),q,h,pad,out)
    return bool(out[0])
'''
numeric.write_text(text,encoding='utf8',newline='\n')
module=package/'_native/python_module.cpp'
text=module.read_text(encoding='utf8')
declaration='''extern "C" int arena_current_mask_occupied(int64_t,int64_t,const double*,const double*,const double*,uint8_t*);
'''
text=text.replace('struct Buffers {',declaration+'\nstruct Buffers {',1)
function='''static PyObject* current_mask_occupied(PyObject*,PyObject* args) {
    if(PyTuple_Size(args)!=6){PyErr_SetString(PyExc_TypeError,"current_mask_occupied requires two sizes and four buffers");return nullptr;}
    int64_t n[2];
    for(int i=0;i<2;++i){n[i]=PyLong_AsLongLong(PyTuple_GET_ITEM(args,i));if(PyErr_Occurred())return nullptr;}
    if(n[0]<0 || n[0]>1000000 || n[1]<0 || n[1]>16384){
        PyErr_SetString(PyExc_ValueError,"bounded current mask occupancy sizes required");return nullptr;
    }
    int64_t sizes[]={2*n[0],2*n[1],n[1],1};Buffers b;
    if(!b.get(args,2,4,sizes,"dddB"))return nullptr;
    int result;
    Py_BEGIN_ALLOW_THREADS
    result=arena_current_mask_occupied(n[0],n[1],(double*)b.views[0].buf,
        (double*)b.views[1].buf,(double*)b.views[2].buf,(uint8_t*)b.views[3].buf);
    Py_END_ALLOW_THREADS
    if(result){PyErr_SetString(PyExc_ValueError,"numeric current mask occupancy input invalid");return nullptr;}
    Py_RETURN_NONE;
}

'''
text=text.replace('static PyMethodDef methods[]=',function+'static PyMethodDef methods[]=',1)
text=text.replace('    {"polygon",polygon,METH_VARARGS,',
    '    {"current_mask_occupied",current_mask_occupied,METH_VARARGS,"Exact inclusive known-return padding predicate"},\n    {"polygon",polygon,METH_VARARGS,',1)
module.write_text(text,encoding='utf8',newline='\n')
setup=STAGE/'src/arena_autonomy/setup.py'
text=setup.read_text(encoding='utf8')
old="'arena_autonomy/_native/polygon_kernel.cpp']"
assert text.count(old)==1 and "'-ffp-contract=off','-fno-fast-math'" in text
text=text.replace(old,"'arena_autonomy/_native/polygon_kernel.cpp','arena_autonomy/_native/current_mask_kernel.cpp']")
setup.write_text(text,encoding='utf8',newline='\n')
kernel='''// Actual returned-hit padding only; no free-space or bracket/structure policy.
#include <cmath>
#include <cstdint>

extern "C" int arena_current_mask_occupied(
    int64_t queries, int64_t hits, const double* points,
    const double* returned, const double* padding, uint8_t* result) {
    if(queries<0 || hits<0)return 1;
    result[0]=0;
    for(int64_t q=0;q<queries;++q){
        for(int64_t h=0;h<hits;++h){
            const double dx=points[2*q]-returned[2*h];
            const double dy=points[2*q+1]-returned[2*h+1];
            // Preserve NumPy norm's multiply/add/sqrt order and inclusive <=.
            // Existing build requires -ffp-contract=off and -fno-fast-math.
            const double distance=std::sqrt(dx*dx+dy*dy);
            if(distance<=padding[h]){result[0]=1;return 0;}
        }
    }
    return 0;
}
'''
(package/'_native/current_mask_kernel.cpp').write_text(kernel,encoding='utf8',newline='\n')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
changes=[]
for path in sorted(STAGE.rglob('*')):
    if path.is_file():
        original=SOURCE/path.relative_to(STAGE)
        before=sha(original) if original.exists() else None
        after=sha(path)
        if before!=after:changes.append(dict(path=path.relative_to(STAGE).as_posix(),before=before,after=after))
assert len(changes)==5
for path in STAGE.rglob('*.py'):ast.parse(path.read_bytes())
(A/'changes.json').write_text(json.dumps(changes,indent=2))
print(json.dumps(changes))
