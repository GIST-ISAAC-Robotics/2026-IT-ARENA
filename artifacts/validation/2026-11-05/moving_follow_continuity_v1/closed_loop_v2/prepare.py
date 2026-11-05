"""연속 전진율 선택 옵션의 폐루프 비교 1회 등록·실행. 기준(moving_pass_baseline_v1)과 같은 장면·인자에
--handoff-continuous-pace만 더한다. 기준 등록 파일은 읽기만 하며 상대 각본을 제어기에 전달하지 않는다.

python3 prepare.py --register  → registered.json(없을 때만)
python3 prepare.py --run       → 등록과 현재 입력이 같고 run1이 없을 때만 1회 실행
"""
import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
BASE = ROOT/'artifacts/validation/2026-11-05/moving_pass_baseline_v1'
PARENT = HERE.parent
SOURCES = ['src/arena_autonomy/arena_autonomy/bypass_handoff.py', 'src/arena_autonomy/arena_autonomy/g1_g2_node.py',
           'src/arena_bringup/launch/simulation.launch.py', 'scripts/g1_handoff_eval.py', 'scripts/validate_g1_traffic.py',
           'src/arena_autonomy/arena_autonomy/local_follow.py', 'src/arena_autonomy/arena_autonomy/front_patch_motion.py',
           'config/tests/continuous_forward_pace_r8_sha256.json', 'config/tests/lidar_mount_structure_gate_r7_sha256.json',
           'src/arena_description/config/b0183_c1_jr60p20_sim.yaml', 'src/arena_gazebo/worlds/it_arena_official/scene.json',
           'config/tests/official_forward_v1.json']


def h(data):
    return hashlib.sha256(data).hexdigest()


def manifest():
    base_reg = json.loads((BASE/'registered.json').read_text(encoding='utf-8'))
    course = BASE/'course.json'
    assert h(course.read_bytes()) == base_reg['course_sha256'], 'baseline course changed'
    args = list(base_reg['args'])
    out = args.index('--output')
    args[out+1] = (HERE/'run1').relative_to(ROOT).as_posix()
    args.insert(out, '--handoff-continuous-pace')
    baseline = json.loads((PARENT/'baseline_metrics.json').read_text(encoding='utf-8'))[0]
    return dict(
        registered_at='2026-11-05',
        scope='이동 앞차 추종 톱니 보완 선택 옵션의 폐루프 비교 1회. 추월 기능 아님. 다른 장면 없음. closed_loop_v1/run1은 시작 약 40 s 뒤 실행 보조 세션 종료로 중단된 실행기 실패로 보존하며(녹색 전, 제어 결과 없음), 같은 등록 조건을 이 하위 버전에서 1회 다시 실행한다. 추가 재시도 없음.',
        baseline=dict(registration=(BASE/'registered.json').relative_to(ROOT).as_posix(),
                      registration_sha256=h((BASE/'registered.json').read_bytes()),
                      course=course.relative_to(ROOT).as_posix(), course_sha256=base_reg['course_sha256'],
                      run=(BASE/'run1').relative_to(ROOT).as_posix(),
                      metrics_file=(PARENT/'baseline_metrics.json').relative_to(ROOT).as_posix(),
                      metrics_sha256=h((PARENT/'baseline_metrics.json').read_bytes()),
                      below_0p10_time_after3_s=baseline['after3']['below_0p10_time_s'],
                      below_0p10_episodes_after3=baseline['after3']['below_0p10_episodes'],
                      command_below_0p10_after3=baseline['command_after3']['below_0p10'],
                      final_gap_m=baseline['final_gap_m']),
        only_argument_difference=['--handoff-continuous-pace', '--output'],
        source_difference='기준 run1은 r7 이전(local_follow.py r4 after)으로 실행됐다. 이번은 r7(구조 경로 증속 전제 수정)+r8 ON이다. r7은 일반 결측 광선이 있을 때의 구조 증속 후보만 바꾸며, 이 비교에서 r7 단독 효과를 분리하지 않는다.',
        args=args,
        sources={p: h((ROOT/p).read_bytes()) for p in SOURCES},
        setup_sha256=h(Path(__file__).read_bytes()),
        compare_sha256=h((PARENT/'compare.py').read_bytes()),
        review_sha256=h((HERE/'review.py').read_bytes()),
        acceptance=dict(
            validity='run_valid·harness_valid·source_unchanged·STOP 검사 모두 참이어야 제어기 결과로 센다. 아니면 무효(재시도 없음).',
            runtime='판단 기록 머리말 continuous_forward_pace=true, 실행 인자 evaluation_handoff_continuous_pace:=true, report.json source_sha256이 r7/r8 계약 after와 일치(review.py). 아니면 옵션 시험으로 세지 않는다.',
            actor='창 안 앞차 1 s 구간 속도 최소 0.25 m/s 이상(각본 0.30 m/s 계속 이동). 아니면 장면 미성립.',
            gates='기준과 같은 11개 관문(course.json 판정선 불변) 모두 통과. 노면: 차체 노면 밖 0·벽 바퀴 0. 접촉 메시지 0.',
            improvement='첫 3 s 이후 0.10 m/s 미만 시간과 진입 횟수가 모두 기준보다 작고, 명령 0.10 m/s 미만 개수도 기준보다 작다(compare.py 정의).',
            final_gap='판정선 1.2 m 그대로. 기준 여유 6 mm를 그대로 보고하며 완화하지 않는다.',
            note='실시간 처리율(호스트 전원 Turbo 선택 후)은 제어 이득과 비교하지 않는다. 판정은 시뮬 시각 기준.'),
    )


def dump(value):
    return (json.dumps(value, indent=2, ensure_ascii=False)+'\n').encode('utf-8')


if __name__ == '__main__':
    reg = HERE/'registered.json'
    if sys.argv[1:] == ['--register']:
        if reg.exists():
            raise SystemExit('registration already exists')
        with reg.open('xb') as f:
            f.write(dump(manifest()))
    elif sys.argv[1:] == ['--run']:
        if reg.read_bytes() != dump(manifest()):
            raise SystemExit('registration changed')
        if (HERE/'run1').exists():
            raise SystemExit('existing trial output')
        args = manifest()['args']
        args = [str(ROOT/a) if a.startswith(('artifacts/', 'src/')) and a.endswith(('.json', '.yaml')) or a.endswith('/run1') else a
                for a in args]
        raise SystemExit(subprocess.call([sys.executable, '-B', *args], cwd=ROOT))
    else:
        raise SystemExit('use --register or --run')
