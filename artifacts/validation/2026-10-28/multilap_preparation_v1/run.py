"""명시 5랩 시험 어댑터. 대회 제어기/기본 평가기는 변경하지 않는다."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
BUMP_RUN = BASE.parent/'bump_traffic_v1/run.py'
spec = importlib.util.spec_from_file_location('five_lap_bump_parent', BUMP_RUN)
bump = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bump)
bump.BASE = BASE
parent = bump.parent
parent.BASE = BASE
parent.CONFIG = BASE/'traffic_config.json'
parent.OVERLAY = BASE/'actor_overlay.json'
original_verify = parent.verify_runtime


def registered_config():
    config = json.loads(parent.CONFIG.read_text(encoding='utf-8'))
    overlay = json.loads(parent.OVERLAY.read_text(encoding='utf-8'))
    case = config['cases'][0]
    rules = config['acceptance'][case['name']]
    if (len(config['cases']) != 1 or case['name'] != 'stop_restart_full_course'
            or case['duration_s'] != 685. or rules['minimum_laps'] != 5
            or rules['maximum_actor_laps'] != 5.2 or config['launch']['speed_bump'] is not True
            or config.get('five_lap_trial', {}).get('enabled') is not True
            or len(overlay['additional_actors']) != 1
            or overlay['additional_actors'][0]['schedule'][-1]['to_s'] != 685.):
        raise ValueError('explicit registered five-lap configuration required')
    return config, overlay, case


def verify_runtime(output, metadata, report):
    result = original_verify(output, metadata, report)
    config, _, case = registered_config()
    window = report.get('case_window', {})
    route = report.get('route_check', {})
    checks = dict(config_hash=report.get('config_sha256') == parent.sha(parent.CONFIG),
        requested_laps=metadata['route']['minimum_laps'] == 5,
        applied_laps=route.get('minimum_laps') == 5,
        applied_actor_laps=route.get('maximum_actor_laps') == 5.2,
        applied_duration=window.get('duration_s') == case['duration_s'],
        route_is_evaluation_only=metadata.get('route_hook', {}).get('evaluation_only') is True)
    result['five_lap_checks'] = checks
    result['passed'] = result['passed'] and all(checks.values())
    return result


parent.verify_runtime = verify_runtime


def prepare(output):
    config, _, case = registered_config()
    metadata = bump.prepare(output)
    extras = (BUMP_RUN, BUMP_RUN.with_name('analyze.py'), parent.OVERLAY)
    with zipfile.ZipFile(BASE/(output.name+'_sources.zip'), 'a', zipfile.ZIP_DEFLATED) as archive:
        for path in extras:
            relative = path.relative_to(ROOT).as_posix()
            digest = parent.sha(path)
            if relative in metadata['source_sha256']:
                assert metadata['source_sha256'][relative] == digest
            else:
                archive.write(path, relative)
                metadata['source_sha256'][relative] = digest
    metadata['route'].update(duration_s=case['duration_s'], minimum_laps=5,
        config=str(parent.CONFIG), actor_overlay=str(parent.OVERLAY))
    metadata['scope'] = ('685s scripted traffic, official signal/rear30cm camera, bump ON; '
        'ego net five laps, evaluation-triggered STOP; not autonomous lap counting or hardware proof')
    metadata['route_hook'] = dict(module='course_candidate.check_full_course',
        candidate_sha256=parent.sha(BASE/'course_candidate.py'),
        original_module_sha256=parent.sha(ROOT/'scripts/g1_course_eval.py'),
        evaluation_only=True, controller_parameters_changed=False)
    (BASE/(output.name+'_prepared.json')).write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    return metadata


def run(output):
    registered_config()
    import g1_course_eval
    from course_candidate import check_full_course
    original = g1_course_eval.check_full_course
    # 별도 프로세스의 평가 함수만 바꾼다. 제어기 프로세스/지도 입력은 불변이다.
    g1_course_eval.check_full_course = check_full_course
    try:
        return parent.run(output)
    finally:
        g1_course_eval.check_full_course = original


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    if args.prepare_only:
        meta = prepare(output)
        print(json.dumps(dict(prepared=True, files=len(meta['source_sha256']), route=meta['route'])))
    else:
        raise SystemExit(run(output))
