"""기존 방지턱·교통1랩의 실행 입력을 보존하고 명시 G1/G2 노드만 연결한다."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
BUMP=ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1'
spec=importlib.util.spec_from_file_location('course_bump_parent',BUMP/'run.py')
bump=importlib.util.module_from_spec(spec);spec.loader.exec_module(bump)
parent=bump.parent
parent.BASE=BASE;parent.CONFIG=BASE/'traffic_config.json'
original_launch=parent.launch_source
original_verify=parent.verify_runtime

OPTIONS=dict(g1_g2_simulation_trial=True,g1_g2_entry_policy='corridor_local_v1',
    g1_g2_corridor_policy='corridor_v2_signed_hold',g1_g2_preparation_policy='observed_entry_space_v1',
    g1_g2_persistence_window='trim_before_compare_v1',g1_g2_follow_entry_space=True,
    g1_g2_reacquire_policy='known_hold_v1')


def launch_source(output):
    _,settings,arguments=original_launch(output)
    settings.update(OPTIONS)
    rows=[f'SetParameter(name={k!r},value={v!r})' for k,v in settings.items()]
    rows += ['IncludeLaunchDescription(PythonLaunchDescriptionSource('+repr(str(BASE/'simulation.launch.py'))+
        '),launch_arguments='+repr(arguments)+'.items())',
        'ExecuteProcess(cmd='+repr(['/usr/bin/python3','-B',str(parent.SIGNAL/'signal_driver_v2.py'),str(output)])+',output="screen")']
    text=('from launch import LaunchDescription\nfrom launch.actions import IncludeLaunchDescription,ExecuteProcess\n'
        'from launch.launch_description_sources import PythonLaunchDescriptionSource\nfrom launch_ros.actions import SetParameter\n'
        'def generate_launch_description():\n    return LaunchDescription(['+',\n        '.join(rows)+'])\n')
    compile(text,'course_trial.launch.py','exec')
    return text,settings,arguments


def verify_runtime(output,metadata,report):
    ordinary=copy.deepcopy(metadata)
    ordinary['controller_parameters']={k:v for k,v in metadata['controller_parameters'].items() if k not in OPTIONS}
    result=original_verify(output,ordinary,report)
    with (output/'follow_decisions.jsonl').open() as stream:header=json.loads(next(stream))['data']
    expected=json.loads(parent.CONFIG.read_text())['handoff_trial']['execution_config']
    checks=dict(schema5=header.get('schema')==5,mode=header.get('controller_mode')=='g1_g2_handoff_trial',
        execution=header.get('execution_config')==expected,
        actual_node='G1/G2 sensor handoff trial' in (output/'simulation.log').read_text(errors='replace'))
    result['handoff_checks']=checks;result['passed']=result['passed'] and all(checks.values())
    return result


parent.launch_source=launch_source;parent.verify_runtime=verify_runtime


def prepare(output):
    metadata=parent.prepare(output)
    extras={BUMP/'run.py',BUMP/'analyze.py',BUMP/'verify_prepared.py',BASE/'traffic_config.json',BASE/'PLAN.md',BASE/'run_trial.sh',
        ROOT/'artifacts/validation/2026-10-28/bump_dynamic_v1/analyze_laps.py',
        ROOT/'artifacts/validation/2026-10-28/bump_dynamic_v1/analyze.py',
        ROOT/'artifacts/validation/2026-10-30/g2_handoff_corridor_gazebo_v1/g2_handoff_eval_core_v3.py'}
    with zipfile.ZipFile(BASE/(output.name+'_sources.zip'),'a',zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(extras):
            key=path.relative_to(ROOT).as_posix();value=parent.sha(path)
            if key in metadata['source_sha256']:assert metadata['source_sha256'][key]==value
            else:archive.write(path,key);metadata['source_sha256'][key]=value
    metadata['scope']='G1/G2 explicit handoff, existing nominal-sensor bump/traffic 155s course; separate evaluation, not hardware proof'
    (BASE/(output.name+'_prepared.json')).write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
    return metadata


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args();output=args.output.resolve()
    if args.prepare_only:print(json.dumps(dict(prepared=True,files=len(prepare(output)['source_sha256']))))
    else:raise SystemExit(parent.run(output))
