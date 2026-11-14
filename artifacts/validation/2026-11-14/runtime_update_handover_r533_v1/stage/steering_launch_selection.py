"""명시 조향 선택을 기존 실행기 인자에 연결한다. 장치/ROS/자식 프로세스 실행 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import sys

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'scripts/runtime_installed_profile.py').is_file())
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import runtime_installed_limits as limits
import runtime_installed_profile as installed
from arena_autonomy.vehicle_geometry import preview_parameters


def guard_arguments(detail='full',result_path=None):
    if detail not in ('full','compact_v1'):raise ValueError('unknown guard status detail')
    if detail=='full':
        if result_path is not None:raise ValueError('guard-result-path requires explicit compact_v1')
        # full은 기존 encoder에서 경로를 읽지 않는다. 빈 ROS CLI 값도 만들지 않는다.
        return ['-p','safety_status_detail:=full']
    path=PurePosixPath(result_path or '')
    if (not path.is_absolute() or path==PurePosixPath('/') or '..' in path.parts
            or '\\' in str(path) or any(c.isspace() or not c.isprintable() for c in str(path))):
        raise ValueError('compact_v1 requires a new absolute Linux guard-result-path')
    # r547와 같은 sidecar 명명/두 매개변수만 선택한다. 원격 폴더/권한은 조회하지 않는다.
    return ['-p','safety_status_detail:=compact_v1','-p',
            'safety_status_evidence_path:='+str(path/'guard_status_contexts.jsonl')]


def selected_arguments(selection_path,*,installed_profile=None,runtime_selection=None,runtime_params=None,
                       guard_detail='full',guard_result_path=None):
    if (installed_profile is None)!=(runtime_selection is None):
        raise ValueError('installed-profile and runtime-selection must be supplied together')
    selection=installed.read_json(selection_path)
    if (selection['schema']!='runtime_source_steering_selection_v1' or selection['source_only'] is not True
            or selection['installed'] is not False or selection['hardware_angle_measured'] is not False):
        raise ValueError('source steering selection must retain CAD/uninstalled provenance')
    sources={}
    for key,prefix in (('steering_profile','config/hardware/'),('vehicle_config','src/arena_description/config/')):
        item=selection[key];name=item['path'];relative=PurePosixPath(name)
        if (not name.startswith(prefix) or relative.is_absolute() or '..' in relative.parts
                or '\\' in name or ':' in name or (ROOT/name).is_symlink()):
            raise ValueError('selected public source path is invalid')
        raw=(ROOT/name).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=item['sha256']:raise ValueError('selected source SHA mismatch: '+name)
        sources[key]=raw
    vehicle=limits.yaml.safe_load(sources['vehicle_config'])['vehicle']
    drive=vehicle['drivetrain'];cap=selection['command_limit_rad']
    if (preview_parameters(vehicle)['max_steering_angle_rad']!=cap
            or drive['max_steering_angle_rad']!=selection['simulation_joint_limit_rad']):
        raise ValueError('selected vehicle central/joint limit differs')
    geometry=dict(wheelbase=drive['wheelbase_m'],track=drive['track_width_m'],wheel_limit=math.pi/6)
    installed.cpp_zero_profile(json.loads(sources['steering_profile']),assembly_id=selection['assembly_id'],
                               command_limit=cap,**geometry)
    actual_limit=cap;params=None
    if installed_profile is not None:
        installed_profile=Path(installed_profile);runtime_selection=Path(runtime_selection)
        installed.verify(installed_profile)
        board=installed.read_json(installed_profile/'runtime_board.json')
        if ((installed_profile/'steering_zero.json').read_bytes()!=sources['steering_profile']
                or any(board['servo'][k]!=v for k,v in dict(command_limit=cap,**geometry).items())):
            raise ValueError('installed steering profile/limit/geometry differs from selected source')
        checked=limits.verify(runtime_selection,installed_profile)
        actual_limit=checked['command_limit_rad']
        if actual_limit>cap:raise ValueError('runtime steering limit exceeds selected source')
        params=['--ros-args','--params-file',str(runtime_selection/'runtime_profile.yaml')]
    if runtime_params is not None:
        profile=limits.profile_values(Path(runtime_params).read_bytes())
        if any(profile[node]['ros__parameters'][limits.PARAMETER]!=actual_limit for node in limits.NODES):
            raise ValueError('runtime params retain a different steering cap; generate both selected limits first')
        if installed_profile is not None:
            limits.verify(runtime_selection,installed_profile,runtime_params=runtime_params)
        params=['--ros-args','--params-file',str(Path(runtime_params))]
    guard_status=guard_arguments(guard_detail,guard_result_path)
    return dict(schema='steering_launch_arguments_v1',selection_file=str(selection_path),
        steering_profile=selection['steering_profile'],vehicle_config=selection['vehicle_config'],
        source_command_limit_rad=cap,effective_command_limit_rad=actual_limit,
        argument_fragments=dict(
            preview=['--vehicle-config',selection['vehicle_config']['path']],
            host=['--max-steering-rad',str(actual_limit)],
            race_ops=['--ros-args','-p','manual_steering_limit_rad:='+str(min(.2,actual_limit))],
            planner=params,guard=params,guard_status=guard_status,
            guard_with_status=(params+guard_status if params is not None else None)),
        guard_detail=guard_detail,guard_path_checked_remotely=False,
        installed_profile_link_verified=installed_profile is not None,
        runtime_params_selected=params is not None,complete_launch_command=False,
        hardware_angle_measured=False,device_io=False,firmware_written=False,nodes_started=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steering-selection',type=Path,required=True)
    parser.add_argument('--installed-profile',type=Path)
    parser.add_argument('--runtime-selection',type=Path)
    parser.add_argument('--runtime-params',type=Path,help='소스 전용으로도 양쪽 실제 YAML 상한을 대조. 구동하지 않음')
    parser.add_argument('--guard-detail',choices=('full','compact_v1'),default='full')
    parser.add_argument('--guard-result-path',help='compact 선택 시 새 Linux 실행결과 절대 경로. r547 sidecar 규칙 사용')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    try:
        result=selected_arguments(args.steering_selection,installed_profile=args.installed_profile,
                                  runtime_selection=args.runtime_selection,runtime_params=args.runtime_params,
                                  guard_detail=args.guard_detail,guard_result_path=args.guard_result_path)
        raw=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
        if args.output:
            with args.output.open('x',encoding='utf-8') as stream:stream.write(raw)
        print(raw,end='');return 0
    except (ValueError,OSError,KeyError,TypeError,limits.yaml.YAMLError) as error:
        print(json.dumps(dict(error=str(error),device_io=False,nodes_started=False)));return 2


if __name__=='__main__':raise SystemExit(main())
