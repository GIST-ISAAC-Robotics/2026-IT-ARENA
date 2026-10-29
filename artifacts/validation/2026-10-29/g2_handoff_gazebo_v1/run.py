"""G2 명시 진입 1회 Gazebo 시험 어댑터(2026-10-28 official_traffic_v1/run.py 형식).

기존 운영 파일·판정선·5랩 소스는 수정하지 않는다. 교통 하네스는 신호 관련 세 줄만 바꾼 실행 사본을
쓰며(차이는 *.diff로 보존), 기존 G1 판정은 report.json에 그대로 남긴다. G2 판정은 g2_eval.py가 따로 낸다.
"""
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
SIGNAL=ROOT/'artifacts/validation/2026-10-27/official_departure_v2'
WORLD=SIGNAL/'fixture'
FIXTURE=BASE/'fixture'
CONFIG=BASE/'g2_config_v1.json'
CASE='handoff_edge_stopped_right'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replace_once(text,before,after):
    if text.count(before)!=1:raise ValueError('source signature changed: '+before[:100])
    return text.replace(before,after)


def effective_sources():
    """official_traffic_v1과 같은 공식 신호 대응만 적용한다(빨강·초록, 신호 발생기 실패/초록 시간 초과 기록)."""
    evaluator=(ROOT/'scripts/g1_traffic_eval.py').read_text(encoding='utf-8')
    harness=(ROOT/'scripts/validate_g1_traffic.py').read_text(encoding='utf-8')
    new_eval=replace_once(evaluator,"{'red', 'yellow', 'green'}.issubset(report.get('signal_states', []))",
                          "{'red', 'green'}.issubset(report.get('signal_states', []))")
    needle="            light = state['light']\n"
    new_harness=replace_once(harness,needle,needle+
        "            if (output/'signal_driver_failure.json').exists():\n"
        "                report['error'] = 'signal_driver_failure: see recorded receipt'\n"
        "                break\n")
    new_harness=replace_once(new_harness,"                raise TimeoutError('green not observed in time')",
        "                report['error'] = 'TimeoutError: green not observed in time'\n                break")
    new_harness=replace_once(new_harness,"        state['autonomy'] = status\n",
        "        state['autonomy'] = status\n"
        "        status['observed_vision_mode'] = status.get('vision', {}).get('mode')\n"
        "        status['observed_signal_mode'] = status.get('vision', {}).get('signal_mode')\n"
        "        status['observed_race_supervisor'] = status.get('race') is not None\n")
    return [(evaluator,new_eval,'g1_traffic_eval.py'),(harness,new_harness,'validate_g1_traffic.py')]


def launch_source(output):
    settings=dict(g1_follow_enabled=True,g1_g2_simulation_trial=True,g1_trace_path=str(output/'g2_decisions.jsonl'),
        g1_boundary_evidence_mode='corner_distance_candidate',g1_occupancy_timing_mode='per_ray_candidate',
        g1_corridor_fallback_enabled=True,g1_short_boundary_prefix_enabled=False,g1_recent_observation_enabled=False,
        partial_observation_policy='bounded_local_v4',g1_parallel_hold_trial=True,g1_pass_release_trial=True,
        g1_guard_curve_trial=True,g1_traffic_recovery_trial=True,g1_traffic_path_recovery_trial=True,
        start_signal_mode='official_4red_local_candidate',race_control_mode='legacy')
    arguments=dict(track='official',grid_slot='3',headless='true',autonomy='true',traffic_light='false',
        autonomy_mode='local_pursuit',lidar_acquisition='sequential',lidar_compensation='both',lidar_rate_hz='10.0',
        autonomy_control_rate_hz='50.0',depth_camera='false',speed_bump='false',chase_camera='false',
        batch_static_visuals='true',scene_broadcaster='true',speed_profile='g1_compare_1mps',
        render_backend='wsl_nvidia',sensor_wall_timeout_s='30',actuation_mode='virtual_mcu',runtime_artifacts=str(output),
        d435i_profile='low_load_30',evaluation_actor_sdf=str(output/'evaluation_actor.sdf'))
    rows=[f'SetParameter(name={k!r},value={v!r})' for k,v in settings.items()]
    rows+=['IncludeLaunchDescription(PythonLaunchDescriptionSource('+repr(str(FIXTURE/'simulation.launch.py'))+
           '),launch_arguments='+repr(arguments)+'.items())',
           'ExecuteProcess(cmd='+repr(['/usr/bin/python3','-B',str(SIGNAL/'signal_driver_v2.py'),str(output)])+',output="screen")']
    text=('from launch import LaunchDescription\n'
          'from launch.actions import IncludeLaunchDescription, ExecuteProcess\n'
          'from launch.launch_description_sources import PythonLaunchDescriptionSource\n'
          'from launch_ros.actions import SetParameter\n'
          'def generate_launch_description():\n    return LaunchDescription(['+',\n        '.join(rows)+'])\n')
    compile(text,'g2_entry.launch.py','exec')
    return text,settings,arguments


def validate_launch_call(effective,settings,arguments,output):
    expected=dict(speed_profile='g1_compare_1mps',lidar_rate_hz=10,lidar_acquisition='sequential',
                  lidar_compensation='both',control_rate_hz=50,grid_slot=3,autonomy_mode='local_pursuit',
                  actuation_mode='virtual_mcu',speed_bump=False,collision_detector='configured',g1_follow=True,
                  r0_plus=False,lidar_test_profile='',motion_test_profile='',record_inputs=False,runtime_artifacts=str(output))
    if any(effective.get(k)!=v for k,v in expected.items()):
        raise ValueError('requested launch differs from registered G2 trial')
    requested={}
    for item in effective.get('extra_arguments',()):
        key,value=item.split(':=',1)
        if key in requested:raise ValueError('duplicate requested parameter: '+key)
        requested[key]=value
    # 시험 launch가 schema5 기록과 후보 옵션을 명시한다. 하네스 요청값은 별도로 확인한다.
    if set(requested)!={'evaluation_actor_sdf'} or requested['evaluation_actor_sdf']!=arguments['evaluation_actor_sdf']:
        raise ValueError('missing or unexpected trial parameter')
    return True


def write_new(path,content):
    with path.open('x',encoding='utf-8',newline='\n') as stream:stream.write(content)


def prepare(output):
    if output.parent!=BASE or output.exists():raise ValueError('fresh direct child output required')
    preflight=json.loads((BASE/'preflight.json').read_text(encoding='utf-8'))
    if not preflight['passed']:raise ValueError('registered preflight did not pass')
    manifest=json.loads((FIXTURE/'manifest.json').read_text(encoding='utf-8'))
    if any(sha(ROOT/p)!=h for p,h in {**manifest['sources'],**manifest['v2_signal_manifest_sources']}.items()):
        raise ValueError('fixture input changed')
    launch,settings,arguments=launch_source(output)
    launch_path=BASE/(output.name+'.launch.py');write_new(launch_path,launch)
    changes=effective_sources();patches=[]
    for original,changed,filename in changes:
        compile(changed,filename,'exec')
        patch=BASE/(output.name+'_'+filename+'.diff')
        write_new(patch,''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),
            fromfile='original/'+filename,tofile='g2_trial/'+filename)))
        patches.append(patch)
    paths={p for folder in ('src','scripts','config','tests') for p in (ROOT/folder).rglob('*')
           if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.pyc','.pyo')}
    paths.update(p for p in WORLD.rglob('*') if p.is_file())
    paths.update(p for p in FIXTURE.rglob('*') if p.is_file())
    paths.update(BASE.glob('*.py'));paths.update([CONFIG,BASE/'preflight.json'])
    paths.update([SIGNAL/'signal_driver.py',SIGNAL/'signal_driver_v2.py',SIGNAL/'native/build/material_client',launch_path,*patches])
    before={p.relative_to(ROOT).as_posix():sha(p) for p in sorted(paths)}
    with zipfile.ZipFile(BASE/(output.name+'_sources.zip'),'x',zipfile.ZIP_DEFLATED) as archive:
        for relative in before:archive.write(ROOT/relative,relative)
        for _,changed,filename in changes:archive.writestr('effective_'+filename,changed)
    metadata=dict(source_sha256=before,output=str(output),launch_path=str(launch_path),
        controller_parameters=settings,launch_arguments=arguments,fixture=manifest,
        scope=('single G1/G2 automatic handoff trial: right-edge stationary box, ego on centerline, official signal v2 fixture, '
               'rear 30 cm trial camera, bump OFF; not centre target, hidden rear car or hardware proof'),
        route=dict(case=CASE,config=str(CONFIG),seed=0))
    write_new(BASE/(output.name+'_prepared.json'),json.dumps(metadata,ensure_ascii=False,indent=2))
    return metadata


def verify_runtime(output,metadata,report):
    """요청값이 아니라 실제 판단 기록 헤더·상태·신호 ACK·로그에서 G2 실행 여부를 확인한다."""
    from arena_autonomy.bypass_handoff import HandoffExecutionConfig
    from dataclasses import asdict
    with (output/'g2_decisions.jsonl').open(encoding='utf-8') as stream:
        header=json.loads(next(stream))['data']
    follow=header['follow_config']
    sequence=[json.loads(line) for line in (output/'signal_sequence.jsonl').read_text(encoding='utf-8').splitlines()]
    stages=[row['stage'] for row in sequence if 'stage' in row]
    receipts=[receipt for row in sequence if 'stage' in row for receipt in row.get('receipts',[])]
    with (output/'autonomy_status.jsonl').open(encoding='utf-8') as stream:
        statuses=[json.loads(line) for line in stream]
    log=(output/'simulation.log').read_text(encoding='utf-8',errors='replace')
    checks=dict(
        handoff_trace_schema5=header.get('schema')==5 and header.get('controller_mode')=='g1_g2_handoff_trial',
        g2_execution_defaults=header.get('execution_config')==asdict(HandoffExecutionConfig()),
        g1_policy_registered=all(follow.get(k)==v for k,v in json.loads(CONFIG.read_text())['acceptance']['g2']['trace_header_expected']['follow_config'].items()),
        no_g1_trace=not (output/'follow_decisions.jsonl').exists(),
        handoff_node_started='G1/G2 sensor handoff trial' in log,
        actual_vision_mode=bool(statuses) and {r.get('observed_vision_mode') for r in statuses}=={'process'},
        actual_signal_mode=bool(statuses) and {r.get('observed_signal_mode') for r in statuses}=={metadata['controller_parameters']['start_signal_mode']},
        actual_legacy_operation=bool(statuses) and all(r.get('observed_race_supervisor') is False for r in statuses),
        exact_applied_stages=stages==['green_probe','off','red1','red2','red3','red4','green'],
        all_lamp_acks=len(receipts)==70 and all(all(r.get(k) is True for k in ('parsed','executed','service_result','applied')) for r in receipts),
        no_signal_failure=not (output/'signal_driver_failure.json').exists(),
        no_false_start=report.get('false_start') is False)
    return dict(passed=all(checks.values()),checks=checks,stages=stages,receipt_count=len(receipts),status_rows=len(statuses),
                follow_config={k:follow.get(k) for k in ('max_steering_rad','wheelbase_m','wall_offset_m','lateral_acceleration_mps2')},
                controller_traceback_lines=log.count('Traceback'),
                scope='applied G2 controller/signal fixture; driving verdict is separate')


def run(output):
    metadata=json.loads((BASE/(output.name+'_prepared.json')).read_text(encoding='utf-8'))
    if output.parent!=BASE or output.exists() or metadata['output']!=str(output):raise ValueError('fresh prepared output required')
    def unchanged():
        return all((ROOT/p).is_file() and sha(ROOT/p)==digest for p,digest in metadata['source_sha256'].items())
    if not unchanged():raise ValueError('prepared source changed')
    for relative in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):sys.path.insert(0,str(ROOT/relative))
    modules=[]
    for _,changed,filename in effective_sources():
        module=types.ModuleType('g2_trial_'+Path(filename).stem);module.__file__=str(ROOT/'scripts'/filename)
        sys.modules[module.__name__]=module
        exec(compile(changed,module.__file__,'exec'),module.__dict__)
        modules.append(module)
    evaluator,validator=modules
    validator.ev=evaluator
    validator.DEMO_WORLD=WORLD
    original_load=validator.load_centerline
    validator.load_centerline=lambda:original_load(WORLD)
    original_start=validator.start_demo_process
    def start(log,*pos,**kw):
        bound=inspect.signature(original_start).bind(log,*pos,**kw);bound.apply_defaults()
        validate_launch_call(bound.arguments,metadata['controller_parameters'],metadata['launch_arguments'],output)
        return subprocess.Popen(['ros2','launch','--noninteractive',metadata['launch_path']],cwd=ROOT,
            stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
    validator.start_demo_process=start
    sys.argv=[validator.__file__,'--config',str(CONFIG),'--case',CASE,'--seed','0','--mode','g1',
              '--output',str(output),'--record-braking-diagnostics']
    code=validator.main()
    metadata.update(argv=sys.argv,source_unchanged=unchanged(),harness_exit_original_g1=code)
    report=json.loads((output/'report.json').read_text(encoding='utf-8'))
    try:
        metadata['runtime_contract']=verify_runtime(output,metadata,report)
    except (KeyError,ValueError,StopIteration,OSError) as error:
        metadata['runtime_contract']=dict(passed=False,error=f'{type(error).__name__}: {error}')
    sys.path.insert(0,str(BASE))
    import g2_eval_v2
    try:
        evaluation=g2_eval_v2.evaluate(output,json.loads(CONFIG.read_text(encoding='utf-8')))
    except Exception as error:
        evaluation=dict(passed=False,error=f'{type(error).__name__}: {error}')
    write_new(BASE/(output.name+'_g2_evaluation.json'),json.dumps(evaluation,ensure_ascii=False,indent=1))
    metadata['g2_passed']=evaluation.get('passed') is True
    (output/'trial_overlay.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    summary=dict(g2_passed=metadata['g2_passed'],original_g1_passed=report.get('passed'),source_unchanged=metadata['source_unchanged'],
                 runtime_contract=metadata['runtime_contract'].get('passed'),gates=evaluation.get('gates'),
                 run_checks=evaluation.get('run_checks'),replay=(evaluation.get('replay') or {}).get('passed'),
                 error=report.get('error'),abort=report.get('abort'))
    print(json.dumps(summary,ensure_ascii=False,indent=1),flush=True)
    return 0 if metadata['source_unchanged'] and metadata['runtime_contract'].get('passed') and metadata['g2_passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args()
    output=args.output.resolve()
    if args.prepare_only:
        result=prepare(output)
        print(json.dumps(dict(prepared=True,files=len(result['source_sha256']),output=str(output))))
    else:
        raise SystemExit(run(output))
