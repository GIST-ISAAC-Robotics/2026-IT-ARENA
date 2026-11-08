"""새 코스 g1g2 C1 주입(uniform ±30 mm, 50±20 ms, seed 23)의 CLI 경계 표적 시험.

시뮬레이터를 띄우지 않는다. r27 등록 인자에 프로필 한 인자만 더한 명령이 두 거부 지점
(프로필 모드 검사, 등록 G1/G2 비교의 overlay 금지)을 모두 지나 check_route까지 도달하는지,
그리고 그 밖의 조합은 그 전에 거부되는지 확인한다. 출력 폴더는 만들지 않는다.

어느 validate_g1_traffic.py를 시험했는지는 HARNESS_EXPECTED_FILE 환경변수와 대조한다.
"""
import json
import os
from pathlib import Path
import shutil

import pytest
import validate_g1_traffic as harness

REPO = Path(__file__).resolve().parents[5]
STAGING = Path(__file__).resolve().parents[1]
PROFILE = 'config/tests/lidar_uniform30_delay50_jitter20.json'
R27 = REPO/'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/registered.json'
NEVER_CREATED = STAGING/'checks/never_created_output'


def r27_args(*extra, drop=()):
    """r27 registered.json 인자 그대로(OUTPUT만 생성 금지 경로로 치환)."""
    args = json.loads(R27.read_text(encoding='utf-8'))['args']
    args = [str(NEVER_CREATED) if a == 'OUTPUT' else a for a in args]
    for flag in drop:
        index = args.index(flag)
        takes_value = index+1 < len(args) and not args[index+1].startswith('--')
        del args[index:index+(2 if takes_value else 1)]
    return args+list(extra)


def replace(args, flag, value):
    args = list(args)
    args[args.index(flag)+1] = value
    return args


class Reached(RuntimeError):
    pass


@pytest.fixture
def sentinel(monkeypatch):
    """두 거부 지점과 10 Hz sequential 검사 뒤의 check_route에서 멈춘다."""
    seen = {}

    def load_centerline(world):
        seen['world'] = world
        return original_load(world)

    def check_route(*_args, **_kwargs):
        raise Reached('reached_check_route_without_launch')

    original_load = harness.load_centerline
    monkeypatch.setattr(harness, 'load_centerline', load_centerline)
    monkeypatch.setattr(harness, 'check_route', check_route)
    yield seen
    assert not NEVER_CREATED.exists()


def run(monkeypatch, args):
    monkeypatch.chdir(REPO)
    monkeypatch.setattr('sys.argv', args)
    harness.main()


def test_module_identity():
    """시험 대상 파일과 저장소 기준 경로를 기록한다. REPO는 언제나 실제 저장소다."""
    assert harness.REPO == REPO
    expected = os.environ.get('HARNESS_EXPECTED_FILE')
    assert expected, 'HARNESS_EXPECTED_FILE must name the module under test'
    assert Path(harness.__file__).resolve() == Path(expected).resolve()


def test_r27_args_without_profile_reach_route_check(monkeypatch, sentinel):
    """대조: 무주입 r27 인자가 현 트리에서도 같은 단계까지 간다."""
    with pytest.raises(Reached):
        run(monkeypatch, r27_args())
    assert sentinel['world'] == harness.TRACK_WORLDS['official_v2026_11_06']


@pytest.mark.parametrize('profile', [PROFILE, str(REPO/PROFILE), 'config/tests/../tests/lidar_uniform30_delay50_jitter20.json'])
def test_r27_args_with_registered_profile_pass_both_gates(monkeypatch, sentinel, profile):
    """양성: 같은 실제 파일로 해석되는 표기만 허용된다(경로는 resolve 뒤 비교)."""
    loaded = {}
    original = harness.load_test_profile

    def load(path, repo):
        loaded['result'] = original(path, repo)
        return loaded['result']

    monkeypatch.setattr(harness, 'load_test_profile', load)
    with pytest.raises(Reached):
        run(monkeypatch, r27_args('--lidar-test-profile', profile))
    assert loaded['result'][0] == REPO/PROFILE
    assert loaded['result'][1] == dict(mode='uniform', amplitude_m=0.03, bias_m=0., delay_s=0.05, jitter_s=0.02, seed=23)
    assert sentinel['world'] == harness.TRACK_WORLDS['official_v2026_11_06']


PARSER_REJECTIONS = {
    'unrecorded': r27_args('--lidar-test-profile', PROFILE, drop=('--record-follow-decisions',)),
    'previous_official_course': r27_args('--lidar-test-profile', PROFILE, drop=('--track',)),
    'demo_course': r27_args('--lidar-test-profile', PROFILE, drop=('--track', '--official-signal-course')),
    'team_fixture_course': r27_args('--lidar-test-profile', PROFILE, '--team-signal-course',
                                    drop=('--official-signal-course',)),
    'not_paced': replace(r27_args('--lidar-test-profile', PROFILE), '--handoff-reserve', 'moving'),
    'no_reserve': r27_args('--lidar-test-profile', PROFILE, drop=('--handoff-reserve',)),
    'seed1': replace(r27_args('--lidar-test-profile', PROFILE), '--seed', '1'),
    'motion_together': r27_args('--lidar-test-profile', PROFILE, '--motion-test-profile', 'config/tests/motion_imu_drop.json'),
    'other_profile_uniform30': r27_args('--lidar-test-profile', 'config/tests/lidar_uniform30.json'),
    'other_profile_delay80': r27_args('--lidar-test-profile', 'config/tests/lidar_delay80.json'),
    'per_ray_overlay': r27_args('--lidar-test-profile', PROFILE, '--occupancy-timing', 'per_ray_candidate'),
    'fragmented_overlay': r27_args('--lidar-test-profile', PROFILE, '--boundary-evidence', 'fragmented_candidate'),
}


REQUIRES = 'G1/G2 lidar stress requires recorded paced'
ONLY = 'G1/G2 lidar stress accepts only'
REASONS = {
    'unrecorded': '--lidar-test-profile requires G1/M3/G1G2 with decision recording',
    'previous_official_course': REQUIRES, 'demo_course': REQUIRES, 'team_fixture_course': REQUIRES,
    'not_paced': REQUIRES, 'no_reserve': REQUIRES, 'seed1': REQUIRES,
    'motion_together': '--motion-test-profile requires recorded G1/M3',
    'other_profile_uniform30': ONLY, 'other_profile_delay80': ONLY,
    'per_ray_overlay': '--occupancy-timing candidate requires',
    'fragmented_overlay': 'fragmented candidate requires',
}


def rejected(failure, capsys, reason):
    """argparse 거부(code 2). staged 패치 모듈이면 어느 검사가 거부했는지도 확인한다."""
    assert failure.value.code == 2
    if hasattr(harness, 'G1G2_LIDAR_PROFILE'):
        assert reason in capsys.readouterr().err


@pytest.mark.parametrize('name', sorted(PARSER_REJECTIONS))
def test_excluded_combinations_rejected_at_parser(monkeypatch, sentinel, capsys, name):
    with pytest.raises(SystemExit) as failure:
        run(monkeypatch, PARSER_REJECTIONS[name])
    rejected(failure, capsys, REASONS[name])


def test_same_values_at_other_repo_path_rejected(monkeypatch, sentinel, capsys):
    """같은 내용이라도 등록 경로가 아닌 저장소 내부 사본은 거부한다."""
    copy = STAGING/'checks/profile_copy/lidar_uniform30_delay50_jitter20.json'
    copy.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(REPO/PROFILE, copy)
    try:
        with pytest.raises(SystemExit) as failure:
            run(monkeypatch, r27_args('--lidar-test-profile', str(copy)))
        rejected(failure, capsys, ONLY)
    finally:
        copy.unlink()
        copy.parent.rmdir()


def test_profile_outside_repo_rejected(monkeypatch, sentinel, tmp_path):
    outside = tmp_path/'lidar_uniform30_delay50_jitter20.json'
    shutil.copyfile(REPO/PROFILE, outside)
    with pytest.raises(ValueError):
        run(monkeypatch, r27_args('--lidar-test-profile', str(outside)))


def test_altered_profile_values_rejected(monkeypatch, sentinel, capsys):
    """등록 경로라도 읽은 값이 등록값과 다르면 거부한다(파일 변경 대비)."""
    original = harness.load_test_profile

    def altered(path, repo):
        resolved, config = original(path, repo)
        return resolved, dict(config, seed=24)

    monkeypatch.setattr(harness, 'load_test_profile', altered)
    with pytest.raises(SystemExit) as failure:
        run(monkeypatch, r27_args('--lidar-test-profile', PROFILE))
    rejected(failure, capsys, ONLY)


@pytest.mark.parametrize('extra', [('--zero-speed-mode', 'no_lag', '--record-braking-diagnostics'),
                                   ('--physics-step', '0.0005', '--record-braking-diagnostics'),
                                   ('--parallel-hold-trial',)])
def test_other_overlays_still_rejected_in_registered_comparison(monkeypatch, sentinel, extra):
    """주입을 허용해도 등록 비교의 다른 overlay 금지는 그대로다."""
    with pytest.raises(ValueError, match='registered G1/G2 traffic comparison uses its fixed configuration'):
        run(monkeypatch, r27_args('--lidar-test-profile', PROFILE, *extra))


def test_merge_with_profile_rejected(monkeypatch, sentinel):
    monkeypatch.setattr(harness, 'merge_profile_for_case', lambda case: {'name': 'forced_merge_for_test'})
    with pytest.raises(ValueError, match='merge trial requires plain G1'):
        run(monkeypatch, r27_args('--lidar-test-profile', PROFILE))
