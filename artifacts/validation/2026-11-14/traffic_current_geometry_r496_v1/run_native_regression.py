"""r486의 기존6개 파일 목록 재사용. 공통 import/native만 확인하며 주행하지 않는다."""
from pathlib import Path
import hashlib,json,os,sys,time
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OUT=A/'native_v1'
sys.dont_write_bytecode=True;os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD']='1'
sys.path[:0]=[str(A),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
    str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare import verify_recipe,read,sha
info,_=verify_recipe() # root의 통합 delta/자원 인계가 기록된 다음에만 진행.
assert not (OUT/'receipt.json').exists()
OUT.mkdir(exist_ok=True);(OUT/'run.pid').write_text(str(os.getpid())+'\n')
from arena_autonomy import moving_pass,bypass_entry_trial,numeric_kernels,_numeric_kernels
modules={m.__name__:str(Path(m.__file__).resolve()) for m in (moving_pass,bypass_entry_trial)}
for path in modules.values():assert Path(path).is_relative_to(ROOT/'src')
expected=read(A/'expected_r494_hashes.json')['files']
for row in expected:
    assert (ROOT/row['path']).read_text(encoding='utf-8')==(A.parent/'traffic_current_geometry_r494_v1/stage'/row['path']).read_text(encoding='utf-8')
before={row['path']:sha(ROOT/row['path']) for row in expected}
assert sha(Path(_numeric_kernels.__file__))==info['native_sha256']
numeric_kernels.configure('native_v1');assert numeric_kernels.classification_enabled()
targets=[str(ROOT/'tests'/name) for name in ('test_moving_pass.py','test_moving_pass_medium.py',
    'test_moving_pass_faster.py','test_bypass_entry_trial.py','test_bypass_entry_guide.py',
    'test_moving_pass_current_return.py')]
import pytest
began=time.monotonic()
code=pytest.main(['-q','-p','no:cacheprovider','--basetemp',str(OUT/'tmp'),
    '--junitxml',str(OUT/'pytest.xml'),*targets])
unchanged=all(sha(ROOT/p)==h for p,h in before.items())
receipt=dict(exit_code=code,elapsed_s=time.monotonic()-began,pid=os.getpid(),targets=targets,
    imports=modules,production_source_hashes=before,production_unchanged=unchanged,
    normalized_r494_stage_exact=True,native_library=str(_numeric_kernels.__file__),
    native_sha256=sha(Path(_numeric_kernels.__file__)),native_enabled_after=numeric_kernels.classification_enabled(),
    r494_saved_cases_reexecuted=False,scope='common imports; native_v1; ROS imports only, no nodes/spin/device/Gazebo')
(OUT/'receipt.json').write_text(json.dumps(receipt,indent=2),encoding='utf-8')
assert unchanged
raise SystemExit(code)
