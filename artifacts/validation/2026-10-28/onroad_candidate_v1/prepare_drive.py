"""도로 내 좌우 피추월 비교의 고정 시험기를 생성한다. 실행하지 않는다."""
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
SOURCE = ROOT / 'artifacts/validation/2026-10-28/official_traffic_v1/run.py'


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError('template changed: ' + old[:80])
    return source.replace(old, new)


def main():
    source = SOURCE.read_text(encoding='utf-8')
    changes = [
        ("CONFIG = ROOT / 'config/tests/g1_traffic_full_course_v1.json'", "SIDE = 'left'\nCONFIG = BASE / 'left_config.json'"),
        ("OVERLAY = ROOT / 'config/tests/e3_stop_restart_full_course_v1.json'", "OVERLAY = ROOT / 'config/tests/e3_onroad_encounter_v1.json'\nGRID_SLOT = 3"),
        ("arguments.update(evaluation_actor_sdf=str(output / 'evaluation_actor.sdf'))", "arguments.update(evaluation_actor_sdf=str(output / 'evaluation_actor.sdf'), grid_slot=str(GRID_SLOT))"),
        ('control_rate_hz=50, grid_slot=3,', 'control_rate_hz=50, grid_slot=GRID_SLOT,'),
        ("paths.update(BASE.glob('*.py'))", "paths.update(BASE.glob('*.py'))\n    paths.update([CONFIG, OVERLAY, BASE / 'DRIVE_PLAN.md', BASE / 'preparation.json'])"),
        ("metadata = dict(source_sha256=before, output=str(output), launch_path=str(launch_path),", "metadata = dict(side=SIDE, source_sha256=before, output=str(output), launch_path=str(launch_path),"),
        ("scope='same 155s traffic script; official signal and fixed rear 30cm camera; bump OFF; not dense start or hardware proof'", "scope='12s close on-road pass, historical gap gates preserved; official signal/rear 30cm camera, bump OFF; not a lap or physical safety proof'"),
        ("route=dict(case='stop_restart_full_course', duration_s=155, minimum_laps=1,", "route=dict(case='slow_lead', duration_s=12, minimum_laps=None,"),
        ("if output.parent != BASE or output.exists() or metadata['output'] != str(output):", "if output.parent != BASE or output.exists() or metadata['output'] != str(output) or metadata['side'] != SIDE:"),
        ("'--config', str(CONFIG), '--case', 'stop_restart_full_course',", "'--config', str(CONFIG), '--case', 'slow_lead',"),
        ("parser.add_argument('--prepare-only', action='store_true')", "parser.add_argument('--prepare-only', action='store_true')\n    parser.add_argument('--side', choices=('left','right'), required=True)"),
        ('output = args.output.resolve()\n    if args.prepare_only:', "SIDE = args.side\n    GRID_SLOT = 3 if SIDE == 'left' else 0\n    CONFIG = BASE / (SIDE + '_config.json')\n    OVERLAY = ROOT / ('config/tests/e3_onroad_encounter_v1.json' if SIDE == 'left' else 'config/tests/e3_onroad_right_encounter_v1.json')\n    output = args.output.resolve()\n    if args.prepare_only:"),
    ]
    for old, new in changes:
        source = replace_once(source, old, new)
    source = source.replace('"""공식 신호·고정 카메라를 최신 교통 후보에 연결하는 격리 시험 어댑터.',
                            '"""최신 G1과 공식 신호로 도로 안 좌우 피추월을 비교하는 고정 시험 어댑터.')
    compile(source, 'run_onroad.py', 'exec')
    with (BASE/'run_onroad.py').open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(source)
    parents = {}
    for side, parent in [('left', 'g1_traffic_v1.json'), ('right', 'e3_m3_right_grid_v1.json')]:
        path = ROOT / 'config/tests' / parent
        config = json.loads(path.read_text(encoding='utf-8'))
        parents[side] = dict(path=str(path.relative_to(ROOT)), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        config['registered_at'] = '2026-10-28 before current G1 on-road execution'
        config['scope'] = '최신 G1 도로 안 피추월 12초 제한 비교. 원래 150 mm 간격/앞차 상한 판정은 보존하고 진행·접촉·노면·실제 전방 대상·STOP을 별도 진단한다.'
        config['launch']['temporary_traffic_light_trigger'] = '시험 어댑터가 공식 가로4red/green과 뒤쪽30cm 고정 카메라를 연결한다. 배치 좌표는 팀 가정이며 실제 동작 계약을 별도 검사한다.'
        config['parent_for_current_trial'] = parents[side]
        with (BASE/(side+'_config.json')).open('x', encoding='utf-8', newline='\n') as stream:
            stream.write(json.dumps(config, ensure_ascii=False, indent=2)+'\n')
    metadata = dict(template=str(SOURCE.relative_to(ROOT)), template_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                    parents=parents, changed='signal/camera adapter and current G1 selection only; original acceptance unchanged')
    with (BASE/'preparation.json').open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    print('fixed drive adapter prepared; no simulation started')


if __name__ == '__main__':
    main()
