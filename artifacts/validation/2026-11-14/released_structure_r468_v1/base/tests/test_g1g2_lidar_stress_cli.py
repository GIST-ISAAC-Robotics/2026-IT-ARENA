"""새 코스 g1g2 C1 주입(uniform ±30 mm, 50±20 ms, seed 23)의 CLI 경계를 시뮬 시작 전에 검사한다.

r27 등록 인자(g1g2_lidar_stress_pair_v1/pair_args.json의 공통 인자)에 등록 프로필 한 인자만 더한
명령이 프로필 모드 검사와 등록 G1/G2 비교의 overlay 금지를 모두 지나 check_route까지 가는지,
그 밖의 조합은 그 전에 거부되는지 본다. 출력·사본 경로는 시험마다 고유하게 만들고 출력은 만들지 않는다.
"""
import json
import os
from pathlib import Path
import shutil
import uuid

import pytest
import validate_g1_traffic as harness

ROOT = Path(__file__).resolve().parents[1]
PROFILE = 'config/tests/lidar_uniform30_delay50_jitter20.json'
REQUESTED = dict(mode='uniform', amplitude_m=0.03, bias_m=0., delay_s=0.05, jitter_s=0.02, seed=23)
PAIR = ROOT/'artifacts/validation/2026-11-08/g1g2_lidar_stress_pair_v1/pair_args.json'


def unique(prefix):
    """다른 시험·동시 실행과 겹치지 않는 artifacts 아래 경로(생성하지 않음)."""
    return ROOT/'artifacts'/f'.{prefix}_{os.getpid()}_{uuid.uuid4().hex}'


def r27_args(output, *extra, drop=(), replace=()):
    args = json.loads(PAIR.read_text(encoding='utf-8'))['common_args']
    args = [str(output) if a == 'OUTPUT' else a for a in args]
    for flag in drop:
        index = args.index(flag)
        takes_value = index+1 < len(args) and not args[index+1].startswith('--')
        del args[index:index+(2 if takes_value else 1)]
    for flag, value in replace:
        args[args.index(flag)+1] = value
    return args+list(extra)


class Reached(RuntimeError):
    pass


class Output(str):
    """생성하지 않을 출력 경로 문자열과 load_centerline에 전달된 월드 기록."""
    def __new__(cls, path, seen):
        value = super().__new__(cls, str(path))
        value.path, value.seen = path, seen
        return value


@pytest.fixture
def output(monkeypatch):
    """두 거부 지점·10 Hz sequential·출력 위치 검사 뒤의 check_route에서 멈춘다."""
    path = unique('g1g2_cli_unused_output')
    seen = {}
    original_load = harness.load_centerline

    def load_centerline(world):
        seen['world'] = world
        return original_load(world)

    def check_route(*_args, **_kwargs):
        raise Reached('reached_check_route_without_launch')

    monkeypatch.setattr(harness, 'load_centerline', load_centerline)
    monkeypatch.setattr(harness, 'check_route', check_route)
    monkeypatch.chdir(ROOT)
    yield Output(path, seen)
    assert not path.exists()


def run(monkeypatch, args):
    monkeypatch.setattr('sys.argv', args)
    harness.main()


def test_harness_uses_repository_root():
    assert harness.REPO == ROOT
    assert harness.G1G2_LIDAR_PROFILE == ROOT/PROFILE and harness.G1G2_LIDAR_REQUESTED == REQUESTED


def test_r27_args_without_profile_reach_route_check(monkeypatch, output):
    with pytest.raises(Reached):
        run(monkeypatch, r27_args(output))
    assert output.seen['world'] == harness.TRACK_WORLDS['official_v2026_11_06']


@pytest.mark.parametrize('profile', [PROFILE, str(ROOT/PROFILE), 'config/tests/../tests/lidar_uniform30_delay50_jitter20.json'])
def test_r27_args_with_registered_profile_pass_both_gates(monkeypatch, output, profile):
    """같은 실제 파일로 해석되는 표기만 허용된다(resolve 뒤 비교)."""
    loaded = {}
    original = harness.load_test_profile

    def load(path, repo):
        loaded['result'] = original(path, repo)
        return loaded['result']

    monkeypatch.setattr(harness, 'load_test_profile', load)
    with pytest.raises(Reached):
        run(monkeypatch, r27_args(output, '--lidar-test-profile', profile))
    assert loaded['result'] == (ROOT/PROFILE, REQUESTED)
    assert output.seen['world'] == harness.TRACK_WORLDS['official_v2026_11_06']


REQUIRES = 'G1/G2 lidar stress requires recorded paced'
ONLY = 'G1/G2 lidar stress accepts only'
P = ('--lidar-test-profile', PROFILE)
PARSER_REJECTIONS = {
    'unrecorded': (P, ('--record-follow-decisions',), (), '--lidar-test-profile requires G1/M3/G1G2 with decision recording'),
    'previous_official_course': (P, ('--track',), (), REQUIRES),
    'demo_course': (P, ('--track', '--official-signal-course'), (), REQUIRES),
    'team_fixture_course': ((*P, '--team-signal-course'), ('--official-signal-course',), (), REQUIRES),
    'not_paced': (P, (), (('--handoff-reserve', 'moving'),), REQUIRES),
    'no_reserve': (P, ('--handoff-reserve',), (), REQUIRES),
    'seed1': (P, (), (('--seed', '1'),), REQUIRES),
    'motion_together': ((*P, '--motion-test-profile', 'config/tests/motion_imu_drop.json'), (), (),
                        '--motion-test-profile requires recorded G1/M3'),
    'other_profile_uniform30': (('--lidar-test-profile', 'config/tests/lidar_uniform30.json'), (), (), ONLY),
    'other_profile_delay80': (('--lidar-test-profile', 'config/tests/lidar_delay80.json'), (), (), ONLY),
    'per_ray_overlay': ((*P, '--occupancy-timing', 'per_ray_candidate'), (), (), '--occupancy-timing candidate requires'),
    'fragmented_overlay': ((*P, '--boundary-evidence', 'fragmented_candidate'), (), (), 'fragmented candidate requires'),
}


@pytest.mark.parametrize('name', sorted(PARSER_REJECTIONS))
def test_excluded_combinations_rejected_at_parser(monkeypatch, output, capsys, name):
    extra, drop, replace, reason = PARSER_REJECTIONS[name]
    with pytest.raises(SystemExit) as failure:
        run(monkeypatch, r27_args(output, *extra, drop=drop, replace=replace))
    assert failure.value.code == 2 and reason in capsys.readouterr().err


def test_same_values_at_other_repo_path_rejected(monkeypatch, output, capsys):
    """같은 내용이라도 등록 경로가 아닌 저장소 내부 사본은 거부한다."""
    folder = unique('g1g2_cli_profile_copy')
    folder.mkdir()
    try:
        copy = folder/'lidar_uniform30_delay50_jitter20.json'
        shutil.copyfile(ROOT/PROFILE, copy)
        with pytest.raises(SystemExit) as failure:
            run(monkeypatch, r27_args(output, '--lidar-test-profile', str(copy)))
        assert failure.value.code == 2 and ONLY in capsys.readouterr().err
    finally:
        shutil.rmtree(folder)


def test_profile_outside_repo_rejected(monkeypatch, output, tmp_path):
    outside = tmp_path/'lidar_uniform30_delay50_jitter20.json'
    shutil.copyfile(ROOT/PROFILE, outside)
    with pytest.raises(ValueError):
        run(monkeypatch, r27_args(output, '--lidar-test-profile', str(outside)))


def test_altered_profile_values_rejected(monkeypatch, output, capsys):
    """등록 경로라도 읽은 값이 등록값과 다르면 거부한다(파일 변경 대비)."""
    original = harness.load_test_profile
    monkeypatch.setattr(harness, 'load_test_profile',
                        lambda path, repo: (original(path, repo)[0], dict(original(path, repo)[1], seed=24)))
    with pytest.raises(SystemExit) as failure:
        run(monkeypatch, r27_args(output, *P))
    assert failure.value.code == 2 and ONLY in capsys.readouterr().err


@pytest.mark.parametrize('extra', [('--zero-speed-mode', 'no_lag', '--record-braking-diagnostics'),
                                   ('--physics-step', '0.0005', '--record-braking-diagnostics'),
                                   ('--parallel-hold-trial',)])
def test_other_overlays_still_rejected_in_registered_comparison(monkeypatch, output, extra):
    """주입을 허용해도 등록 비교의 다른 overlay 금지는 그대로다."""
    with pytest.raises(ValueError, match='registered G1/G2 traffic comparison uses its fixed configuration'):
        run(monkeypatch, r27_args(output, *P, *extra))


def test_merge_with_profile_rejected(monkeypatch, output):
    monkeypatch.setattr(harness, 'merge_profile_for_case', lambda case: {'name': 'forced_merge_for_test'})
    with pytest.raises(ValueError, match='merge trial requires plain G1'):
        run(monkeypatch, r27_args(output, *P))
