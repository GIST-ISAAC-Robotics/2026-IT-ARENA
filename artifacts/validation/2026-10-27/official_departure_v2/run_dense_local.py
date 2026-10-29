"""기존2대 어댑터를 보존하고 3/5대 각본·모든 쌍 평가를 연결한다."""
import argparse
import difflib
import hashlib
import inspect
import json
from pathlib import Path
import subprocess
import sys
import types
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import run as departure
import validate_g1_traffic as base_validator
from g1_course_eval import wheel_center_surface
from dense_audit import load_and_check
from audit_local_pursuit_road import road_polygon


# 시험기 예외가 확인된 신호 실패를 기록한 뒤 기존 명시 STOP 경로로 이어가게 한다.
harness_original=(ROOT/'scripts/validate_g1_traffic.py').read_text(encoding='utf-8')
needle="            light = state['light']\n"
assert harness_original.count(needle)==1
harness_effective=harness_original.replace(needle,needle+"            if (output/'signal_driver_failure.json').exists():\n                report['error'] = 'signal_driver_failure: see recorded receipt'\n                break\n")
old_no_green="                raise TimeoutError('green not observed in time')"
assert harness_effective.count(old_no_green)==1
harness_effective=harness_effective.replace(old_no_green,"                report['error'] = 'TimeoutError: green not observed in time'\n                break")
validator=types.ModuleType('dense_v2_harness');validator.__file__=str(ROOT/'scripts/validate_g1_traffic.py')
sys.modules[validator.__name__]=validator
exec(compile(harness_effective,validator.__file__,'exec'),validator.__dict__)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--count',type=int,choices=(5,),required=True);args=parser.parse_args()
    out=args.output.resolve()
    if out.exists() or not out.is_relative_to(BASE):raise ValueError('fresh local directory required')
    if json.loads((BASE/'run1/report.json').read_text(encoding='utf-8')).get('passed') is not True:
        raise ValueError('single-car departure must pass first')
    if json.loads((BASE/'pixel_check.json').read_text(encoding='utf-8')).get('passed') is not True:
        raise ValueError('static pixel check must pass first')
    if json.loads((BASE/'two_car_run1/m5_departure.json').read_text(encoding='utf-8')).get('passed') is not True:
        raise ValueError('two-car bounded departure must pass first')
    if args.count==5 and json.loads((BASE/'dense3_run2/m5_departure.json').read_text(encoding='utf-8')).get('passed') is not True:
        raise ValueError('reviewed three-car departure must pass first')
    fixture=BASE/'fixture';manifest=json.loads((fixture/'manifest.json').read_text(encoding='utf-8'))
    for rel,h in manifest['sources'].items():
        if hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=h:raise ValueError('fixture source changed: '+rel)
    # 전체 교통 판정 중 공식 시퀀스에 없는 노랑 요구만 대조용 사본에서 변경한다.
    text=(ROOT/'scripts/g1_traffic_eval.py').read_text(encoding='utf-8')
    old="{'red', 'yellow', 'green'}.issubset(report.get('signal_states', []))"
    assert text.count(old)==1
    changed=text.replace(old,"{'red', 'green'}.issubset(report.get('signal_states', []))")
    evaluator=types.ModuleType('m5_official_signal_eval');evaluator.__file__=str(ROOT/'scripts/g1_traffic_eval.py')
    sys.modules[evaluator.__name__]=evaluator;exec(compile(changed,evaluator.__file__,'exec'),evaluator.__dict__)
    validator.ev=evaluator;validator.DEMO_WORLD=fixture
    original_load=validator.load_centerline
    validator.load_centerline=lambda:original_load(fixture)
    pixel_comparison=json.loads((BASE/'dense_pixel_candidates_v2.json').read_text(encoding='utf-8'))
    if pixel_comparison.get('expected_comparison') is not True:
        raise ValueError('same-frame local signal comparison required')
    for rel,digest in pixel_comparison['source_sha256'].items():
        if hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=digest:
            raise ValueError('pixel comparison source changed: '+rel)
    launch,params,launch_args=departure.wrapper_launch(out)
    if params['start_signal_mode']!='official_4red_candidate' or launch.count("'official_4red_candidate'")!=1:
        raise ValueError('unexpected base signal mode')
    params=dict(params,start_signal_mode='official_4red_local_candidate')
    launch=launch.replace("'official_4red_candidate'","'official_4red_local_candidate'")
    assert launch.count(str(BASE/'signal_driver.py'))==1
    launch=launch.replace(str(BASE/'signal_driver.py'),str(BASE/'signal_driver_v2.py'))
    actor=out/'evaluation_actor.sdf'
    # SetParameter는 그대로, launch 사본에 평가용 actor 경로만 추가한다.
    original_mapping=repr(launch_args);launch_args=dict(launch_args,evaluation_actor_sdf=str(actor))
    assert launch.count(original_mapping)==1
    launch=launch.replace(original_mapping,repr(launch_args))
    launch_path=BASE/(out.name+'.launch.py')
    with launch_path.open('x',encoding='utf-8') as stream:stream.write(launch)
    def start(log,*pos,**kw):
        bound=inspect.signature(original_start).bind(log,*pos,**kw)
        effective=bound.arguments
        expected=f'evaluation_actor_sdf:={actor}'
        if expected not in effective.get('extra_arguments',()):raise ValueError('actor path mismatch')
        if effective['grid_slot']!=3 or effective['speed_profile']!='g1_compare_1mps':raise ValueError('unexpected launch configuration')
        checks=dict(lidar_rate_hz=10,lidar_acquisition='sequential',lidar_compensation='both',
                    control_rate_hz=50,autonomy_mode='local_pursuit',actuation_mode='virtual_mcu',g1_follow=True)
        if any(effective.get(k)!=v for k,v in checks.items()):raise ValueError('fixed departure settings mismatch')
        return subprocess.Popen(['ros2','launch',str(launch_path)],cwd=ROOT,stdin=subprocess.DEVNULL,
            stdout=log,stderr=log,start_new_session=True)
    original_start=validator.start_demo_process;validator.start_demo_process=start
    patch=BASE/(out.name+'_signal_gate.diff')
    with patch.open('x',encoding='utf-8') as stream:
        stream.write(''.join(difflib.unified_diff(text.splitlines(True),changed.splitlines(True),
            fromfile='original/g1_traffic_eval.py',tofile='official_signal_only/g1_traffic_eval.py')))
    config=BASE/f'dense{args.count}_config.json';overlay=BASE/f'dense{args.count}_actors.json'
    paths=set([Path(__file__),BASE/'run.py',BASE/'signal_driver.py',BASE/'signal_driver_v2.py',BASE/'native/material_client.cpp',BASE/'native/CMakeLists.txt',BASE/'native/build/material_client',BASE/'prepare_dense.py',BASE/'dense_audit.py',config,overlay,launch_path,patch,*fixture.rglob('*')])
    paths.update(ROOT/rel for rel in validator.source_paths(config,actor_overlay=overlay))
    before={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()}
    with zipfile.ZipFile(BASE/(out.name+'_adapter_sources.zip'),'x',zipfile.ZIP_DEFLATED) as archive:
        for rel in before:archive.write(ROOT/rel,rel)
        archive.writestr('effective_g1_traffic_eval.py',changed)
        archive.writestr('effective_validate_g1_traffic.py',harness_effective)
        archive.writestr('harness_failure_stop.diff',''.join(difflib.unified_diff(harness_original.splitlines(True),harness_effective.splitlines(True),fromfile='original',tofile='failure_stop_path'))) 
    sys.argv=[str(ROOT/'scripts/validate_g1_traffic.py'),'--config',str(config),'--case','slow_lead',
        '--seed','0','--mode','g1','--output',str(out),'--actor-overlay',str(overlay),'--record-follow-decisions',
        '--boundary-evidence','corner_distance_candidate','--occupancy-timing','per_ray_candidate',
        '--corridor-fallback','--short-boundary-prefix','--partial-observation-policy','bounded_local_v4',
        '--parallel-hold-trial','--pass-release-trial','--guard-curve-trial']
    code=validator.main()
    unchanged=all(hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==h for rel,h in before.items())
    report=json.loads((out/'report.json').read_text(encoding='utf-8'));evaluation=report.get('evaluation',{})
    first=report.get('case_window',{}).get('green_sim_s')
    poses=[tuple(json.loads(line)) for line in (out/'ego_poses.jsonl').read_text(encoding='utf-8').splitlines()]
    surface=wheel_center_surface(road_polygon(out/'runtime_world.sdf'),[r for r in poses if first is not None and r[0]>=first])
    gap=evaluation.get('gap',{});progress=evaluation.get('progress',{}).get('at_window_end_m')
    lower=gap.get('min_swept_lower_bound_m')
    gates=dict(original_run_valid=report.get('run_valid') is True,source_unchanged=unchanged,
        explicit_stop=bool(report.get('stop_checks')) and all(report['stop_checks'].values()),
        minimum_progress=progress is not None and progress>=1.,
        positive_swept_gap=lower is not None and lower>0 and gap.get('swept_missing_actor_intervals')==0,
        no_overlap=gap.get('sampled_finite_complete') is True and gap.get('sampled_overlap_samples')==0,
        no_contact=gap.get('actor_contact_messages_with_ego')==0,
        two_wheel_surface=surface.get('valid') is True and surface.get('three_plus_samples')==0)
    pairs=load_and_check(out,report)
    gates['all_pair_gaps']=pairs['passed']
    extra=report.get('additional_actors',{})
    gates['all_pair_contacts']=extra.get('combined_contacts',{}).get('valid') is True and not extra.get('combined_contacts',{}).get('events')
    result=dict(scope='registered dense start evaluation, original traffic verdict unchanged',car_count=args.count,pair_metrics=pairs,
        passed=all(gates.values()),gates=gates,original_passed=report.get('passed'),original_gates=report.get('gates'),
        original_exit=code,progress_m=progress,gap=gap,wheel_centers=surface,
        parameters=params,launch_arguments=launch_args,source_unchanged=unchanged,source_sha256=before)
    (out/'m5_departure.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('passed','gates','original_passed','original_exit','progress_m')},ensure_ascii=False))
    # 별도 관찰이 통과해도 원래 실행 실패 코드를 지우지 않는다.
    return code if unchanged else 1


if __name__=='__main__':raise SystemExit(main())
