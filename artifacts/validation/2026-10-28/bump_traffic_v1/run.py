"""기존 공식 교통 어댑터를 재사용하는 방지턱ON 대조. 기본 소스는 변경하지 않는다."""
import argparse
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
PARENT = BASE.parent/'official_traffic_v1/run.py'
spec = importlib.util.spec_from_file_location('bump_traffic_parent', PARENT)
parent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parent)
parent.BASE = BASE
parent.CONFIG = BASE/'traffic_config.json'
original_launch = parent.launch_source
original_validate = parent.validate_launch_call
original_effective = parent.effective_sources
original_verify = parent.verify_runtime


def launch_source(output):
    text, settings, arguments = original_launch(output)
    text = parent.replace_once(text, "'speed_bump': 'false'", "'speed_bump': 'true'")
    arguments['speed_bump'] = 'true'
    compile(text, 'bump_traffic.launch.py', 'exec')
    return text, settings, arguments


def validate_launch_call(effective, settings, arguments, output):
    if effective.get('speed_bump') is not True or arguments.get('speed_bump') != 'true':
        raise ValueError('explicit bump ON required')
    return original_validate({**effective, 'speed_bump': False}, settings, arguments, output)


def effective_sources():
    changes = original_effective()
    original, harness, filename = changes[1]
    harness = parent.replace_once(harness,
        '    ego, actor, contacts, commands = [], [], [], []',
        '    ego, actor, contacts, commands = [], [], [], []\n    bump_poses = []')
    anchor = "                    ego.append((when, float(p.get('x', 0.)), float(p.get('y', 0.)), yaw_of(pose['orientation'])))"
    harness = parent.replace_once(harness, anchor, anchor +
        "\n                    bump_poses.append(dict(time=when, position=dict(p), orientation=dict(pose['orientation'])))")
    anchor = "        (output/'actor_commands.json').write_text(json.dumps(commands, indent=1), encoding='utf-8')"
    harness = parent.replace_once(harness, anchor, anchor +
        "\n        (output/'bump_pose_trace.json').write_text(json.dumps(bump_poses), encoding='utf-8')")
    compile(harness, 'effective_bump_traffic.py', 'exec')
    return [changes[0], (original, harness, filename)]


def bump_collisions(path):
    world = ET.parse(path).getroot().find('world')
    rows = []
    for model in world.iter('model'):
        for link in model.findall('link'):
            if not link.get('name', '').startswith('safety_bump_'):
                continue
            collisions = []
            for collision in link.findall('collision'):
                surface = collision.find('surface')
                collisions.append(dict(name=collision.get('name'),
                    pose=(collision.findtext('pose') or '').split(),
                    box=(collision.findtext('geometry/box/size') or '').split(),
                    surface=ET.tostring(surface, encoding='unicode') if surface is not None else None))
            rows.append(dict(model=model.get('name'), link=link.get('name'),
                model_pose=(model.findtext('pose') or '').split(),
                pose=(link.findtext('pose') or '').split(), collisions=collisions))
    return rows


def verify_runtime(output, metadata, report):
    result = original_verify(output, metadata, report)
    expected = bump_collisions(parent.FIXTURE/'world.sdf')
    actual = bump_collisions(output/'runtime_world.sdf')
    checks = dict(requested_on=metadata['launch_arguments'].get('speed_bump') == 'true',
        runtime_collision_matches=bool(expected) and expected == actual,
        original_has_collision=any(row['collisions'] for row in expected),
        full_pose_capture=(output/'bump_pose_trace.json').is_file())
    result['bump_checks'] = checks
    result['bump_links'] = len(actual)
    result['passed'] = result['passed'] and all(checks.values())
    return result


parent.launch_source = launch_source
parent.validate_launch_call = validate_launch_call
parent.effective_sources = effective_sources
parent.verify_runtime = verify_runtime


def prepare(output):
    metadata = parent.prepare(output)
    extras = (PARENT, parent.CONFIG, BASE/'README.md',
        BASE.parent/'bump_dynamic_v1/analyze.py', BASE.parent/'bump_dynamic_v1/analyze_laps.py')
    with zipfile.ZipFile(BASE/(output.name+'_sources.zip'), 'a', zipfile.ZIP_DEFLATED) as archive:
        for path in extras:
            relative = path.relative_to(ROOT).as_posix()
            value = parent.sha(path)
            if relative in metadata['source_sha256']:
                assert metadata['source_sha256'][relative] == value
            else:
                archive.write(path, relative)
                metadata['source_sha256'][relative] = value
    metadata['scope'] = ('155s scripted traffic with official signal/rear30cm camera, published bump ON; '
        'nominal sensor error, full 1 lap; not hardware or five-lap proof')
    metadata['route']['config'] = str(parent.CONFIG)
    metadata['bump_collision_baseline'] = bump_collisions(parent.FIXTURE/'world.sdf')
    if not metadata['bump_collision_baseline']:
        raise ValueError('source bump missing')
    (BASE/(output.name+'_prepared.json')).write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    if args.prepare_only:
        metadata = prepare(output)
        print(json.dumps(dict(prepared=True, files=len(metadata['source_sha256']), bump_on=True)))
    else:
        raise SystemExit(parent.run(output))
