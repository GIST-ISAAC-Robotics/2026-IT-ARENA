"""case/규칙 누락, 실제 옵션·설치 소스·고정본 불일치를 주행 전에 검사한다."""
import importlib.util
import json
from pathlib import Path
import sys
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
spec=importlib.util.spec_from_file_location('bump_preflight',ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1/verify_prepared.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
from run import OPTIONS,parent
from blocked_route import install_route_check


def verify(name):
    result=old.verify(BASE,name)
    config=json.loads((BASE/'traffic_config.json').read_text());metadata=json.loads((BASE/(name+'_prepared.json')).read_text())
    case=next(c for c in config['cases'] if c['name']==metadata['route']['case'])
    rules=config['acceptance'][case['name']]
    assert all(k in rules for k in ('hold_window_s','minimum_hold_s','maximum_restart_delay_s','restart_speed_mps',
        'minimum_progress_after_restart_m','maximum_final_gap_m','minimum_laps','maximum_actor_laps','surface_policy'))
    assert all(metadata['controller_parameters'].get(k)==v for k,v in OPTIONS.items())
    from traffic_actors import overlay_case,actor_specs
    overlay_path=Path(metadata['route']['actor_overlay'])
    assert overlay_path==BASE/'actor_overlay.json'
    effective=overlay_case(case,overlay_path)
    specs=actor_specs(config,effective)
    assert len(specs)==2 and specs[0]['schedule'][-1]['speed_mps']==0.
    original=json.loads((ROOT/'config/tests/e3_stop_restart_full_course_v1.json').read_text())
    assert effective['additional_actors']==original['additional_actors']
    installed=ROOT/'install/arena_autonomy/lib/python3.12/site-packages/arena_autonomy'
    for source in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'):
        assert (installed/source.name).read_bytes()==source.read_bytes(), 'installed source differs: '+source.name
    executable=ROOT/'install/arena_autonomy/lib/arena_autonomy/g1_g2_handoff_trial'
    assert executable.is_file()
    # 실제 하네스와 같은 설정·지오메트리·두 actor 일정 준비를 실행하되 ROS/프로세스는 시작하지 않는다.
    import types
    module=types.ModuleType('blocked_preflight_validator');module.__file__=str(ROOT/'scripts/validate_g1_traffic.py')
    sys.modules[module.__name__]=module
    source=parent.effective_sources()[1][1]
    exec(compile(source,module.__file__,'exec'),module.__dict__)
    module.DEMO_WORLD=parent.FIXTURE;install_route_check(module)
    parsed=module.load_config(parent.CONFIG)
    actual=module.overlay_case(module.case_named(parsed,metadata['route']['case']),parent.OVERLAY)
    actors=module.actor_specs(parsed,actual)
    line,scene=module.load_centerline(parent.FIXTURE)
    start=module.actor_start_s(parsed,scene,0,actual)
    road=module.road_polygon(parent.FIXTURE/'world.sdf');walls=module.obstacle_polygons(parent.FIXTURE/'world.sdf')
    primary=module.check_route(parsed,line,actual,start,road,walls)
    starts,other=module.check_additional_routes(parsed,actual,actors,scene,0,line,road,walls,
        module.actor_start_s,module.check_route,module.actor_pose_at)
    model=module.compose_sdf(actors,[module.actor_pose_at(line,s,a['schedule'],0.,a['lateral_offset_m']) for s,a in zip(starts,actors)],module.actor_sdf)
    assert model and primary['actor_travel_m']==1.4 and other['sampled_actor_pair_min_gap_m']>0
    result['actual_setup_dry_run']=dict(passed=True,primary=primary,additional=other)
    result.update(case_rules_complete=True,installed_source_same=True,explicit_options=True,overlay_case_matches=True,actor_schedules_valid=True)
    return result


if __name__=='__main__':print(json.dumps(verify(sys.argv[1])))
