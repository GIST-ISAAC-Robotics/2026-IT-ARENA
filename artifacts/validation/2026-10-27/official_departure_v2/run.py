"""기존 단독 주행 평가기의 공식 신호 시험 어댑터. 구동/보호/STOP 판정은 유지한다."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import types
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
for d in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):
    sys.path.insert(0,str(ROOT/d))


def wrapper_launch(output):
    settings=dict(g1_follow_enabled=True,g1_boundary_evidence_mode='corner_distance_candidate',
        g1_occupancy_timing_mode='per_ray_candidate',g1_corridor_fallback_enabled=True,
        g1_short_boundary_prefix_enabled=True,g1_recent_observation_enabled=False,
        partial_observation_policy='bounded_local_v4',g1_parallel_hold_trial=True,
        g1_pass_release_trial=True,g1_guard_curve_trial=True,g1_trace_path=str(output/'follow_decisions.jsonl'),
        start_signal_mode='official_4red_candidate',race_control_mode='legacy')
    args=dict(track='official',grid_slot='3',headless='true',autonomy='true',traffic_light='false',
        autonomy_mode='local_pursuit',lidar_acquisition='sequential',lidar_compensation='both',lidar_rate_hz='10.0',
        autonomy_control_rate_hz='50.0',depth_camera='false',speed_bump='false',chase_camera='false',
        batch_static_visuals='true',scene_broadcaster='true',speed_profile='g1_compare_1mps',
        render_backend='wsl_nvidia',sensor_wall_timeout_s='30',actuation_mode='virtual_mcu',runtime_artifacts=str(output),
        d435i_profile='low_load_30')
    rows=[f'SetParameter(name={k!r},value={v!r})' for k,v in settings.items()]
    rows.append('IncludeLaunchDescription(PythonLaunchDescriptionSource('+repr(str(BASE/'fixture/simulation.launch.py'))+
        '),launch_arguments='+repr(args)+'.items())')
    rows.append('ExecuteProcess(cmd='+repr(['/usr/bin/python3','-B',str(BASE/'signal_driver.py'),str(output)])+',output="screen")')
    return ('from launch import LaunchDescription\nfrom launch.actions import IncludeLaunchDescription, ExecuteProcess\n'
        'from launch.launch_description_sources import PythonLaunchDescriptionSource\nfrom launch_ros.actions import SetParameter\n'
        'def generate_launch_description():\n    return LaunchDescription(['+',\n        '.join(rows)+'])\n'),settings,args


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=Path);options=parser.parse_args()
    out=options.output.resolve()
    if out.exists() or not out.is_relative_to(BASE):raise ValueError('fresh local trial directory required')
    fixture=BASE/'fixture';manifest=json.loads((fixture/'manifest.json').read_text(encoding='utf-8'))
    for relative,value in manifest['sources'].items():
        if hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()!=value:raise ValueError('fixture source changed: '+relative)
    launch,settings,args=wrapper_launch(out)
    launch_path=BASE/(out.name+'.launch.py')
    with launch_path.open('x',encoding='utf-8') as stream:stream.write(launch)
    source=ROOT/'scripts/validate_basic_autonomy.py';original=source.read_text(encoding='utf-8')
    old='{"red", "yellow", "green"}.issubset(signal_states)'
    if original.count(old)!=1:raise ValueError('legacy signal gate signature changed')
    changed=original.replace(old,'{"red", "green"}.issubset(signal_states)')
    patch_path=BASE/(out.name+'_evaluator.diff')
    with patch_path.open('x',encoding='utf-8') as stream:
        stream.write(''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),
            fromfile='legacy/validate_basic_autonomy.py',tofile='official signal evaluation only')))
    evaluator=types.ModuleType('official_departure_evaluator');evaluator.__file__=str(source)
    sys.modules[evaluator.__name__]=evaluator
    exec(compile(changed,str(source),'exec'),evaluator.__dict__)
    evaluator.DEMO_WORLD=fixture
    def start(log,*pos,**kw):
        return subprocess.Popen(['ros2','launch',str(launch_path)],cwd=ROOT,stdin=subprocess.DEVNULL,
            stdout=log,stderr=log,start_new_session=True)
    evaluator.start_demo_process=start
    paths=sorted(set([Path(__file__),BASE/'signal_driver.py',BASE/'prepare.py',launch_path,patch_path,source,
        *fixture.rglob('*'),*ROOT.glob('src/arena_autonomy/arena_autonomy/*.py'),
        *ROOT.glob('src/arena_vehicle_interface/arena_vehicle_interface/*.py'),
        ROOT/'src/arena_description/config/vehicle.yaml',ROOT/'src/arena_description/config/b0183_c1.yaml']))
    before={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()}
    with zipfile.ZipFile(BASE/(out.name+'_trial_sources.zip'),'x',zipfile.ZIP_DEFLATED) as archive:
        for relative in before:archive.write(ROOT/relative,relative)
        archive.writestr('effective_evaluator.py',changed)
    sys.argv=[str(source),'--output',str(out),'--actuation-mode','virtual_mcu','--g1-follow',
        '--record-follow-decisions','--g1-boundary-evidence','corner_distance_candidate','--g1-occupancy-timing','per_ray_candidate',
        '--g1-corridor-fallback','--partial-observation-policy','bounded_local_v4','--autonomy-mode','local_pursuit',
        '--grid-slot','3','--disable-speed-bump','--speed-profile','g1_compare_1mps','--lidar-acquisition','sequential',
        '--lidar-compensation','both','--lidar-rate-hz','10','--control-rate-hz','50','--render-backend','wsl_nvidia',
        '--sensor-wall-timeout-s','30','--target-progress-m','1.0','--max-sim-seconds','35',
        '--wall-timeout-s','1500','--stop-wall-timeout-s','180','--shutdown-mode','service']
    code=evaluator.main()
    after={relative:hashlib.sha256((ROOT/relative).read_bytes()).hexdigest() for relative in before}
    metadata=dict(scope='single physical-model car departure; not dense M5, full lap or hardware proof',
        fixture_manifest=manifest,controller_parameters=settings,launch_arguments=args,
        evaluator_change='require red and green, no yellow in official sequence; all other acceptance unchanged',
        argv=sys.argv,source_sha256=before,source_unchanged=before==after,base_exit=code)
    (out/'trial_overlay.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    if before!=after:raise RuntimeError('source changed during trial')
    return code


if __name__=='__main__':raise SystemExit(main())
