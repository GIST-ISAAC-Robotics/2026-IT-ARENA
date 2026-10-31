"""case/규칙 누락, 실제 옵션·설치 소스·고정본 불일치를 주행 전에 검사한다."""
import importlib.util
import json
from pathlib import Path
import sys
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
spec=importlib.util.spec_from_file_location('bump_preflight',BASE/'verify_prepared.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
from run import OPTIONS,parent
from traffic_actors import actor_specs,overlay_case


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
    assert case['schedule']==[dict(from_s=0.,to_s=4.,speed_mps=.35),dict(from_s=4.,to_s=9.,speed_mps=0.),dict(from_s=9.,to_s=155.,speed_mps=.35)]
    applied=overlay_case(case,parent.OVERLAY)
    specs=actor_specs(config,applied);assert len(specs)==2
    import validate_g1_traffic as validator
    fixture=parent.FIXTURE
    line,scene=validator.load_centerline(fixture)
    road=validator.road_polygon(fixture/'world.sdf');obstacles=validator.obstacle_polygons(fixture/'world.sdf')
    start=validator.actor_start_s(config,scene,0,applied)
    validator.check_route(config,line,applied,start,road,obstacles)
    validator.check_additional_routes(config,applied,specs,scene,0,line,road,obstacles,
        validator.actor_start_s,validator.check_route,validator.actor_pose_at)
    baseline=json.loads((BASE.parent/'g2_restart_during_bypass_v1/traffic_config.json').read_text())
    current=json.loads(json.dumps(config))
    assert current['handoff_trial']['execution_config'].pop('motion_pause_policy')=='bounded_motion_pause_v1'
    assert current==baseline
    assert fixture==ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'
    result.update(case_rules_complete=True,installed_source_same=True,explicit_options=True)
    return result


if __name__=='__main__':print(json.dumps(verify(sys.argv[1])))
