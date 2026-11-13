#!/usr/bin/env python3
"""검증된 설치 bundle에서 ROS 두 상한과 host 인자를 파생한다. 장치/노드 실행 없음."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'),
                str(ROOT/'src/arena_vehicle_interface')]
import yaml
import runtime_installed_profile as installed
from arena_autonomy import vehicle_geometry

NODES = ('local_pursuit', 'lidar_safety')
PARAMETER = 'max_steering_angle_rad'
FILES = ('runtime_profile.base.yaml', 'runtime_profile.yaml', 'runtime_host_limits.json')
SCHEMA = 'runtime_installed_limits_selection_v1'


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def source_hashes():
    return {name: digest(Path(path).read_bytes()) for name, path in (
        ('runtime_installed_limits.py', __file__),
        ('runtime_installed_profile.py', installed.__file__),
        ('vehicle_geometry.py', vehicle_geometry.__file__))}


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node, deep=False):
    loader.flatten_mapping(node)
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError('duplicate YAML key: '+str(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def profile_values(raw):
    value = yaml.load(raw, Loader=UniqueLoader)
    if type(value) is not dict:
        raise ValueError('runtime YAML must contain both named ROS nodes')
    for node in NODES:
        entry = value.get(node)
        parameters = entry.get('ros__parameters') if type(entry) is dict else None
        limit = parameters.get(PARAMETER) if type(parameters) is dict else None
        if type(limit) not in (int, float) or not math.isfinite(limit) or limit <= 0:
            raise ValueError(node+' requires an existing finite positive '+PARAMETER)
    return value


def selected_contract(folder):
    folder = Path(folder)
    manifest = installed.verify(folder)
    board = installed.read_json(folder/'runtime_board.json')
    original = board['servo']['command_limit']
    limit = vehicle_geometry.float32_limit_not_above(original)
    return dict(assembly_id=manifest['assembly_id'],
                installed_manifest_sha256=digest((folder/'manifest.json').read_bytes()),
                runtime_board_sha256=digest((folder/'runtime_board.json').read_bytes()),
                board_command_limit_rad=original, command_limit_rad=limit)


def derived_profile(base_raw, limit):
    value = copy.deepcopy(profile_values(base_raw))
    for node in NODES:
        if value[node]['ros__parameters'][PARAMETER] < limit:
            raise ValueError(node+' would widen existing steering limit; '
                             'choose an explicitly compatible base profile')
        value[node]['ros__parameters'][PARAMETER] = limit
    # r401 계열 base는 이 선택값이 planner 아래에만 있었다. 새 설치 파생본에만
    # 동일한 명시 구성을 기록하며, 독립 guard는 planner 결과를 입력받지 않는다.
    planner = value['local_pursuit']['ros__parameters']
    guard = value['lidar_safety']['ros__parameters']
    if (planner.get('g1_guard_observation_policy') == 'selected_r181'
            and planner.get('g1_traffic_recovery_trial') is True
            and guard.get('g1_traffic_recovery_trial') is True):
        guard['g1_guard_observation_policy'] = 'selected_r181'
    return value


def host_limits(limit):
    return dict(max_steering_rad=limit, arguments=['--max-steering-rad', repr(limit)])


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2)+'\n').encode('utf-8')


def prepare(installed_profile, runtime_params, output):
    contract = selected_contract(installed_profile)
    base_raw = Path(runtime_params).read_bytes()
    value = derived_profile(base_raw, contract['command_limit_rad'])
    files = dict(zip(FILES, (base_raw,
        yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode('utf-8'),
        json_bytes(host_limits(contract['command_limit_rad'])))))
    if (contract != selected_contract(installed_profile)
            or base_raw != Path(runtime_params).read_bytes()):
        raise ValueError('input changed during runtime selection preparation')
    manifest = dict(schema=SCHEMA, **contract, sources=source_hashes(),
                    files={name: digest(raw) for name, raw in files.items()})
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for name, raw in files.items():
        (output/name).write_bytes(raw)
    (output/'manifest.json').write_bytes(json_bytes(manifest))
    return manifest


def verify(selection, installed_profile, *, runtime_params=None, host_steering_rad=None):
    selection = Path(selection)
    manifest = installed.read_json(selection/'manifest.json')
    contract = selected_contract(installed_profile)
    if (type(manifest) is not dict or set(manifest) != {'schema', 'sources', 'files', *contract}
            or manifest.get('schema') != SCHEMA
            or any(manifest.get(key) != value for key, value in contract.items())
            or manifest.get('sources') != source_hashes()
            or type(manifest.get('files')) is not dict or set(manifest['files']) != set(FILES)):
        raise ValueError('runtime selection differs from selected installed bundle or sources')
    for name, expected in manifest['files'].items():
        if digest((selection/name).read_bytes()) != expected:
            raise ValueError('runtime selection file changed: '+name)
    expected = derived_profile((selection/FILES[0]).read_bytes(), contract['command_limit_rad'])
    if profile_values((selection/FILES[1]).read_bytes()) != expected:
        raise ValueError('runtime YAML differs beyond the two installed steering limits and explicit guard policy selection')
    if installed.read_json(selection/FILES[2]) != host_limits(contract['command_limit_rad']):
        raise ValueError('host arguments differ from installed steering limit')
    if runtime_params is not None and profile_values(Path(runtime_params).read_bytes()) != expected:
        raise ValueError('selected ROS YAML differs from generated runtime profile')
    if host_steering_rad is not None and (
            type(host_steering_rad) not in (int, float) or not math.isfinite(host_steering_rad)
            or host_steering_rad != contract['command_limit_rad']):
        raise ValueError('selected host steering argument differs from generated limit')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    make = commands.add_parser('prepare')
    make.add_argument('--installed-profile', type=Path, required=True)
    make.add_argument('--runtime-params', type=Path, required=True)
    make.add_argument('--output', type=Path, required=True)
    check = commands.add_parser('verify')
    check.add_argument('--selection', type=Path, required=True)
    check.add_argument('--installed-profile', type=Path, required=True)
    check.add_argument('--runtime-params', type=Path)
    check.add_argument('--host-steering-rad', type=float)
    host = commands.add_parser('host-arguments')
    host.add_argument('--selection', type=Path, required=True)
    host.add_argument('--installed-profile', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == 'prepare':
            result = prepare(args.installed_profile, args.runtime_params, args.output)
        else:
            result = verify(args.selection, args.installed_profile,
                runtime_params=getattr(args, 'runtime_params', None),
                host_steering_rad=getattr(args, 'host_steering_rad', None))
        if args.command == 'host-arguments':
            print(json.dumps(host_limits(result['command_limit_rad'])['arguments']))
        else:
            print(json.dumps(dict(assembly_id=result['assembly_id'],
                board_command_limit_rad=result['board_command_limit_rad'],
                command_limit_rad=result['command_limit_rad'], nodes_started=False,
                firmware_modified=False), ensure_ascii=False))
        return 0
    except (ValueError, OSError, OverflowError, TypeError, yaml.YAMLError) as exc:
        print(json.dumps(dict(error=str(exc), nodes_started=False, firmware_modified=False)))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
