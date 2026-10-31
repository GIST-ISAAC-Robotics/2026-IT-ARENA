"""원래 도로/잔디1mm 단차와 방지턱을 유지한45초 상대 양보 감속 시험."""
import argparse
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile
from source_surface import build_fixture

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
PARENT=BASE.parent/'g2_motion_pause_gazebo_v1/run.py'
ORIGINAL=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'
spec=importlib.util.spec_from_file_location('restart_trial_parent',PARENT)
trial=importlib.util.module_from_spec(spec);spec.loader.exec_module(trial)
trial.BASE=BASE;trial.parent.BASE=BASE;trial.parent.CONFIG=BASE/'traffic_config.json'
trial.parent.FIXTURE=BASE/'fixture'
trial.parent.OVERLAY=BASE/'actor_overlay.json'
OPTIONS=trial.OPTIONS
OPTIONS['g1_g2_relative_yield_policy']='bounded_relative_yield_v2'

original_effective_sources=trial.parent.effective_sources


def effective_sources():
    changes=original_effective_sources()
    original,harness,filename=changes[1]
    harness=trial.parent.replace_once(harness,
        '        return check_full_course(config, centerline, case, s0, road, obstacles)',
        '        from partial_route import check_registered_prefix\n'
        '        return check_registered_prefix(config, centerline, case, s0, road, obstacles)')
    compile(harness,filename,'exec')
    return [changes[0],(original,harness,filename)]


trial.parent.effective_sources=effective_sources


def surface_contract(path):
    root=ET.parse(path).getroot()
    grass=root.find(".//model/link[@name='grass']")
    road=root.find(".//model/link[@name='track_surface']")
    original=ET.parse(ORIGINAL/'world.sdf').getroot()
    checks=dict(grass_pose=grass is not None and grass.find('pose') is None,
                road_pose=road is not None and road.find('pose') is None)
    def shape(node):
        return (node.tag,sorted(node.attrib.items()),(node.text or '').strip(),[shape(child) for child in node])
    for name,current in [('grass',grass),('track_surface',road)]:
        expected=original.find(f".//model/link[@name='{name}']")
        # 시각 배치는 달라도 충돌 형상·물성의 XML 구조/값은 같아야 한다.
        checks[name+'_collisions_unchanged']=(current is not None and
            [shape(x) for x in current.findall('collision')]==[shape(x) for x in expected.findall('collision')])
    return checks


def verify_runtime(output,metadata,report):
    result=trial.verify_runtime(output,metadata,report)
    checks=surface_contract(output/'runtime_world.sdf')
    result.update(level_surface_checks=checks,passed=result['passed'] and all(checks.values()))
    return result


trial.parent.verify_runtime=verify_runtime


def prepare(output):
    if not (BASE/'fixture').exists():
        receipt=build_fixture(ORIGINAL,BASE/'fixture')
        (BASE/'fixture_change.json').write_text(json.dumps(receipt,indent=2))
    else:
        receipt=json.loads((BASE/'fixture_change.json').read_text())
        assert trial.parent.sha(ORIGINAL/'world.sdf')==receipt['original_world_sha256']
        assert trial.parent.sha(BASE/'fixture/world.sdf')==receipt['variant_world_sha256']
        assert all(surface_contract(BASE/'fixture/world.sdf').values())
    metadata=trial.prepare(output)
    extras=(PARENT,BASE/'fixture_change.json',BASE/'actor_overlay.json',BASE/'route_reference_config.json',BASE/'build.sh',ORIGINAL/'simulation.launch.py',ORIGINAL/'world.sdf')
    with zipfile.ZipFile(BASE/(output.name+'_sources.zip'),'a',zipfile.ZIP_DEFLATED) as archive:
        for p in extras:
            key=p.relative_to(ROOT).as_posix()
            if key in metadata['source_sha256']:assert metadata['source_sha256'][key]==trial.parent.sha(p)
            else:archive.write(p,key);metadata['source_sha256'][key]=trial.parent.sha(p)
    metadata['route']['duration_s']=45.
    metadata['scope']='45s bounded relative yield on original road-grass step surface; unchanged bump/friction/torque; original full-lap verdict retained separately'
    (BASE/(output.name+'_prepared.json')).write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
    return metadata


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args();output=args.output.resolve()
    if args.prepare_only:print(json.dumps(dict(prepared=True,files=len(prepare(output)['source_sha256']))))
    else:raise SystemExit(trial.parent.run(output))
