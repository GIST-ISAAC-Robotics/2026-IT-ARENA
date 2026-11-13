"""현재 공통 Python 2파일과 기존 native backend의 제한 회귀. ROS 노드를 생성하지 않는다."""
from pathlib import Path
import hashlib
import importlib
import json
import os
import sys
import time

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
OUT=HERE/'native_v1';OUT.mkdir(exist_ok=True)
sys.dont_write_bytecode=True;os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD']='1'
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
    str(ROOT/'scripts'),str(ROOT/'tests'),str(HERE)]
import arena_autonomy
from arena_autonomy import moving_pass,bypass_entry_trial,numeric_kernels,_numeric_kernels
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
applied=json.loads((HERE/'integration_v1/applied.json').read_text(encoding='utf8'))
modules={m.__name__:str(Path(m.__file__).resolve()) for m in (moving_pass,bypass_entry_trial)}
for f in applied['files']:
    assert sha(ROOT/f['path'])==f['after_sha256'],f
for name,path in modules.items():
    assert Path(path).is_relative_to(ROOT/'src'),(name,path)
numeric_kernels.configure('native_v1')
assert numeric_kernels.classification_enabled()
(OUT/'run.pid').write_text(str(os.getpid())+'\n',encoding='utf8')
print('r486 native regression started pid='+str(os.getpid()),flush=True)
import pytest
targets=[str(ROOT/'tests'/name) for name in ('test_moving_pass.py','test_moving_pass_medium.py',
    'test_moving_pass_faster.py','test_bypass_entry_trial.py','test_bypass_entry_guide.py')]
targets += [str(HERE/'test_current_follow_return.py'),str(HERE/'stage/tests/test_moving_pass_current_return.py')]
began=time.monotonic()
code=pytest.main(['-q','-p','no:cacheprovider','--basetemp',str(OUT/'tmp'),
    '--junitxml',str(OUT/'pytest.xml'),*targets])
receipt={'exit_code':code,'elapsed_s':time.monotonic()-began,'pid':os.getpid(),
    'targets':targets,'imports':modules,'native_library':str(_numeric_kernels.__file__),
    'native_sha256':sha(_numeric_kernels.__file__),'native_enabled_after':numeric_kernels.classification_enabled(),
    'production_inputs':[{**f,'after_test_sha256':sha(ROOT/f['path'])} for f in applied['files']],
    'portable_test_sha256':sha(HERE/'stage/tests/test_moving_pass_current_return.py'),
    'scope':'current common imports; native_v1 numeric kernels; ROS imports only, no node/spin/device/Gazebo'}
(OUT/'receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
assert all(f['after_sha256']==f['after_test_sha256'] for f in receipt['production_inputs'])
sys.exit(code)
