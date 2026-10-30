"""옆차가 정지 뒤 재출발하는 별도 장면. 기존 움직이는 옆차 실패의 대체 판정이 아니다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
PARENT=ROOT/'artifacts/validation/2026-10-29/rear_reapproach_v1/probe_rear.py'
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
spec=importlib.util.spec_from_file_location('_rear_pause_probe',PARENT)
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)
probe.HEADER_FOLLOW.pop('rear_lane_keep_trial',None)
probe.HEADER_FOLLOW['rear_lane_keep_policy']='observed_fresh_v2'
probe.CASES['B_right_stopped_return']=dict(sign=-1.,side_dx=0.,
    side=[(0.,7.,0.),(7.,12.,1.)],lead=probe.LEAD,duration=12.)

if __name__=='__main__':
    output=Path(sys.argv[sys.argv.index('--output')+1]).resolve()
    probe.main()
    report=json.loads((output/'summary.json').read_text())
    report['sources'][str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report['registered_case']=probe.CASES['B_right_stopped_return']
    report['prior_rolling_failure_retained']=True
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
