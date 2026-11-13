"""최초 staged 작성 기록. 후속 preview 검증 보완 뒤 refresh_stage.py로 최종 patch/hash를 갱신한다.

이 최초 생성기를 다시 실행하지 않는다. base/staged 원문과 최종 patch가 검토 대상이다.
실행/빌드/제어 호출 없음.
"""
import ast
import difflib
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]


def main():
    changes={}
    def change(name,callback):
        path=ROOT/name;before=path.read_text(encoding='utf8');after=callback(before)
        assert after!=before,('no change',name)
        base=A/'base'/name;stage=A/'staged'/name
        assert not base.exists() and not stage.exists(),('preserve stage',name)
        base.parent.mkdir(parents=True,exist_ok=True);stage.parent.mkdir(parents=True,exist_ok=True)
        base.write_bytes(path.read_bytes());stage.write_text(after,encoding='utf8',newline='\n')
        changes[name]=(before,after)
    def replace(text,old,new):
        assert text.count(old)==1,('ambiguous replacement',old[:80]);return text.replace(old,new)
    def follow(text):
        text=replace(text,'    wheelbase_m: float = .135\n','    wheelbase_m: float = .135\n'
            '    # 옛 trace에 필드가 없으면 당시 측방 기본 기하를 유지한다.\n'
            '    track_m: float = .135\n    wheel_radius_m: float = .0325\n    wheel_width_m: float = .012\n')
        return replace(text,'    def __post_init__(self):\n','    def __post_init__(self):\n'
            "        for name in ('track_m', 'wheel_radius_m', 'wheel_width_m'):\n"
            '            value = getattr(self, name)\n'
            '            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:\n'
            "                raise ValueError(name + ' must be finite and positive')\n")
    change('src/arena_autonomy/arena_autonomy/local_follow.py',follow)
    change('src/arena_autonomy/arena_autonomy/pass_release_trial.py',lambda s:replace(s,
        '                               length_m=c.length_m, width_m=c.width_m, max_steering_rad=c.max_steering_rad,',
        '                               length_m=c.length_m, width_m=c.width_m, max_steering_rad=c.max_steering_rad,\n'
        '                               track_m=c.track_m, wheel_radius_m=c.wheel_radius_m, wheel_width_m=c.wheel_width_m,'))
    change('src/arena_autonomy/arena_autonomy/m3_execution_trial.py',lambda s:replace(s,
        '            road_half_m=c.road_half_m)\n',
        '            road_half_m=c.road_half_m, track_m=c.track_m,\n'
        '            wheel_radius_m=c.wheel_radius_m, wheel_width_m=c.wheel_width_m)\n'))
    change('src/arena_autonomy/arena_autonomy/m3_observed_follow.py',lambda s:replace(s,
        '            max_steering_rad=c.max_steering_rad, road_half_m=c.road_half_m)\n',
        '            max_steering_rad=c.max_steering_rad, road_half_m=c.road_half_m,\n'
        '            track_m=c.track_m, wheel_radius_m=c.wheel_radius_m, wheel_width_m=c.wheel_width_m)\n'))
    change('src/arena_autonomy/arena_autonomy/local_pursuit.py',lambda s:replace(s,
        '            length_m=length, width_m=width,\n',
        '            length_m=length, width_m=width,\n'
        "            track_m=float(self.declare_parameter('vehicle_track_width_m', .135).value),\n"
        "            wheel_radius_m=float(self.declare_parameter('vehicle_wheel_radius_m', .0325).value),\n"
        "            wheel_width_m=float(self.declare_parameter('vehicle_wheel_width_m', .012).value),\n"))
    def launch(text):
        text=replace(text,"    return ({'body_geometry_x': str(offset)},\n            {'vehicle_length_m': float(footprint['length_m']), 'vehicle_width_m': float(footprint['width_m'])})",
            "    from arena_autonomy.vehicle_geometry import planning_parameters\n    return ({'body_geometry_x': str(offset)}, planning_parameters(config))")
        start=text.index('def _equivalent_steering_limit(drivetrain):');end=text.index('\n\ndef _launch_setup(context):',start)
        return text[:start]+'''def _equivalent_steering_limit(drivetrain):
    """기존 중앙 등가각 변환을 hardware preview와 같은 순수 함수로 공유한다."""
    from arena_autonomy.vehicle_geometry import equivalent_steering_limit
    return equivalent_steering_limit(drivetrain)
'''+text[end:]
    change('src/arena_bringup/launch/simulation.launch.py',launch)
    def probe(text):
        text=replace(text,'import argparse\n','import argparse\nimport hashlib\n')
        text=replace(text,'def ros_arguments(args,root):\n','''def vehicle_geometry_override(args):
    path = getattr(args, 'vehicle_config', None)
    if path is None:
        return None
    import yaml
    from arena_autonomy.vehicle_geometry import preview_parameters
    path = Path(path).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    vehicle = yaml.safe_load(raw)['vehicle']
    return dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest(), parameters=preview_parameters(vehicle))


def ros_arguments(args,root):
''')
        text=replace(text,"    if args.estimated_time:result += ['-p','motion_scan_time_policy:=host_estimated_v1']",
            "    geometry = vehicle_geometry_override(args)\n"
            "    if geometry is not None:\n"
            "        for name, value in geometry['parameters'].items():\n"
            "            result += ['-p', name+':='+str(value)]\n"
            "    if args.estimated_time:result += ['-p','motion_scan_time_policy:=host_estimated_v1']")
        text=replace(text,"    p.add_argument('--output',type=Path,required=True)\n",
            "    p.add_argument('--output',type=Path,required=True)\n"
            "    p.add_argument('--vehicle-config',type=Path,default=None,help='명시 차량 YAML의 기하만 전달. 생략 시 기존 preview 설정 유지')\n")
        return replace(text,'            actuator_node_started=False,synthetic_encoder_published=False,time_contract_verified=False,',
            '            actuator_node_started=False,synthetic_encoder_published=False,time_contract_verified=False,\n'
            '            selected_vehicle_geometry=vehicle_geometry_override(args),')
    change('scripts/probe_g1_hardware_shadow.py',probe)
    def runner(text):
        text=replace(text,"    p.add_argument('--output',type=Path,required=True)\n",
            "    p.add_argument('--output',type=Path,required=True)\n"
            "    p.add_argument('--vehicle-config',type=Path,default=None,help='선택 차량 YAML. 생략 시 기존 비구동 preview 기하 유지')\n")
        return replace(text,"    if args.camera:node += ['--camera']",
            "    if getattr(args, 'vehicle_config', None) is not None:\n"
            "        node += ['--vehicle-config', str(args.vehicle_config.expanduser().resolve(strict=True))]\n"
            "    if args.camera:node += ['--camera']")
    change('scripts/run_hardware_sensor_preview.py',runner)
    new='src/arena_autonomy/arena_autonomy/vehicle_geometry.py'
    changes[new]=('',(A/'staged'/new).read_text(encoding='utf8'))
    patches=[];manifest={}
    for name,(before,after) in sorted(changes.items()):
        ast.parse(after,filename=name)
        patches.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),
            fromfile='a/'+name if before else '/dev/null',tofile='b/'+name))
        manifest[name]=dict(base_sha256=hashlib.sha256((ROOT/name).read_bytes()).hexdigest() if before else None,
            staged_sha256=hashlib.sha256((A/'staged'/name).read_bytes()).hexdigest())
    (A/'geometry_contract.patch').write_text(''.join(patches),encoding='utf8',newline='\n')
    (A/'stage_manifest.json').write_text(json.dumps(dict(files=manifest,common_sources_modified=False,
        default_hardware_profile_changed=False,vehicle_yaml_modified=False,controller_calls=0),indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(staged_files=len(manifest),common_sources_modified=False,controller_calls=0)))


if __name__=='__main__':main()
