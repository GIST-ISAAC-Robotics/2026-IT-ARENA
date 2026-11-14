from pathlib import Path
import hashlib,json,sys
import numpy as np
A=Path(__file__).resolve().parent;ROOT=A.parents[4]
sys.dont_write_bytecode=True
prior=A.parent.parent/'guard_memory_ray_clearance_r532_v1/stage/tests'
sys.path[:0]=[str(A.parent/'stage/src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(prior),
    str(A.parent.parent/'field_wall_irregularity_r518_v1/test_deps')]
candidate=A.parent/'stage/src/arena_autonomy/arena_autonomy/guard_memory_command.py'
before=hashlib.sha256(candidate.read_bytes()).hexdigest()
original=np.cross
def compat(a,b,*args,**kwargs):
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.shape[-1]==bb.shape[-1]==2:return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return original(a,b,*args,**kwargs)
np.cross=compat
import pytest
old=prior/'test_guard_memory_command.py'
selected=['test_epoch_time_motion_failures_cannot_create_permission','test_actual_control_preserves_command_and_input_stops',
          'test_normal_positive_control_without_latch_is_unchanged']
code=pytest.main(['-q','-p','no:cacheprovider',str(A/'test_relevance.py'),
    *[str(old)+'::'+name for name in selected],'--junitxml='+str(A/'result.xml')])
after=hashlib.sha256(candidate.read_bytes()).hexdigest()
assert before==after,'candidate changed during review'
(A/'result.json').write_text(json.dumps(dict(exit_code=int(code),candidate_sha256=before,candidate_unchanged=True,
    source_modified=False,scope='Windows Python fallback; actual guard control AST; no ROS/WSL/Jetson',
    numpy=np.__version__,numpy_compatibility='process-local 2D cross only'),indent=2),encoding='utf-8')
raise SystemExit(code)
