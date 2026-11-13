"""Windows 원 수식 호환으로 소수 단위/점유/상태 회귀만 실행."""
from pathlib import Path
import json
import os
import sys
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.dont_write_bytecode=True;os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD']='1'
sys.path[:0]=[str(HERE),str(ROOT/'artifacts/validation/2026-11-13/complete_plan_trace_r449_v1/test_dependencies'),
    str(ROOT/'tests'),str(ROOT/'scripts')]
import validate_stage
import numpy as np
original_cross=np.cross;cross_calls=0
def compatible_cross(a,b,*args,**kwargs):
    global cross_calls
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.ndim and bb.ndim and aa.shape[-1]==bb.shape[-1]==2:
        cross_calls+=1
        return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return original_cross(a,b,*args,**kwargs)
np.cross=compatible_cross
import pytest
focused='--focused' in sys.argv
version='v3' if focused else 'v2'
targets=([str(HERE/'test_current_follow_return.py')+'::'+name for name in (
    'test_side_return_outside_return_path_does_not_erase_current_free_path',
    'test_published_proposal_contains_only_new_center_path')] if focused else [str(HERE/'test_current_follow_return.py')])
code=pytest.main(['-q','-p','no:cacheprovider','--basetemp',str(HERE/f'pytest_tmp_{version}'),
    '--junitxml',str(HERE/f'pytest_{version}.xml'),*targets])
(HERE/f'test_receipt_{version}.json').write_text(json.dumps({'exit_code':code,'python':sys.executable,
    'numpy':np.__version__,'pytest':pytest.__version__,'two_dimensional_cross_compatibility_calls':cross_calls,
    'compatibility_scope':'진단 runner에서 기본 2D cross만 기존 determinant 공식 사용, 그 외 원 함수 유지. 생산 소스/센서/판정선 변경 아님.'},ensure_ascii=False,indent=2)+'\n',encoding='utf8')
sys.exit(code)
