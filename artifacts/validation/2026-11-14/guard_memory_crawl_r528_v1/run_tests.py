from pathlib import Path
import json,sys
import numpy as np
A=Path(__file__).resolve().parent
sys.path[:0]=[str(A/'stage/src/arena_autonomy'),str(A/'stage/src/arena_vehicle_interface'),str(A/'stage/tests'),str(A.parent/'field_wall_irregularity_r518_v1/test_deps')]
original=np.cross
def compat(a,b,*args,**kwargs):
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.shape[-1]==bb.shape[-1]==2:return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return original(a,b,*args,**kwargs)
np.cross=compat
import pytest
code=pytest.main(['-q',str(A/'stage/tests/test_guard_memory_command.py'),'--junitxml='+str(A/'focused.xml')])
(A/'test_result.json').write_text(json.dumps(dict(exit_code=int(code),numpy=np.__version__,compatibility='process-local 2D determinant; Python numeric',scope='unit and actual guard control AST without ROS transport')),encoding='utf-8')
raise SystemExit(code)
