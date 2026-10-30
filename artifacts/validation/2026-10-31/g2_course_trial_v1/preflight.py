"""case/규칙 누락, 실제 옵션·설치 소스·고정본 불일치를 주행 전에 검사한다."""
import importlib.util
import json
from pathlib import Path
import sys
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
spec=importlib.util.spec_from_file_location('bump_preflight',ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1/verify_prepared.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
from run import OPTIONS


def verify(name):
    result=old.verify(BASE,name)
    config=json.loads((BASE/'traffic_config.json').read_text());metadata=json.loads((BASE/(name+'_prepared.json')).read_text())
    case=next(c for c in config['cases'] if c['name']==metadata['route']['case'])
    rules=config['acceptance'][case['name']]
    assert all(k in rules for k in ('hold_window_s','minimum_hold_s','maximum_restart_delay_s','restart_speed_mps',
        'minimum_progress_after_restart_m','maximum_final_gap_m','minimum_laps','maximum_actor_laps','surface_policy'))
    assert all(metadata['controller_parameters'].get(k)==v for k,v in OPTIONS.items())
    installed=ROOT/'install/arena_autonomy/lib/python3.12/site-packages/arena_autonomy'
    for source in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'):
        assert (installed/source.name).read_bytes()==source.read_bytes(), 'installed source differs: '+source.name
    executable=ROOT/'install/arena_autonomy/lib/arena_autonomy/g1_g2_handoff_trial'
    assert executable.is_file()
    result.update(case_rules_complete=True,installed_source_same=True,explicit_options=True)
    return result


if __name__=='__main__':print(json.dumps(verify(sys.argv[1])))
