"""이관 후 구형 binary 실패와 재빌드 뒤 새 entrypoint 실제 import를 구별한다."""
import hashlib,json,sys,traceback
from pathlib import Path
from arena_autonomy import numeric_kernels as backend,_numeric_kernels as native

A=Path(__file__).resolve().parent
ROOT=A.parents[4]
phase=sys.argv[1]
assert phase in ('old','new')
path=Path(native.__file__).resolve()
assert path.parent==ROOT/'src/arena_autonomy/arena_autonomy'
assert Path(backend.__file__).resolve()==ROOT/'src/arena_autonomy/arena_autonomy/numeric_kernels.py'
report=dict(phase=phase,native_path=str(path),native_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    wrapper_path=backend.__file__,has_current_mask_occupied=hasattr(native,'current_mask_occupied'))
backend.configure('native_v1')
if phase=='old':
    assert not report['has_current_mask_occupied']
    try:backend.current_mask_occupied([[0.,0.]],[[0.,0.]],[0.])
    except RuntimeError as error:
        report.update(expected_old_binary_failure=True,error=str(error),traceback=traceback.format_exc())
    else:raise AssertionError('old native binary silently accepted new entrypoint')
else:
    assert report['has_current_mask_occupied']
    report['new_entrypoint_result']=backend.current_mask_occupied([[0.,0.]],[[0.,0.]],[0.])
    assert report['new_entrypoint_result'] is True
backend.configure('python')
(A/(phase+'_binary_probe.json')).write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='traceback'}))
