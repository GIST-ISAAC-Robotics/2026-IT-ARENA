"""C 대표: r18 중앙 정지차(신호 1 m 앞) 등록 인자 그대로, 실행 사본에 신호 양지주만 추가해 교통 검사기를 1회 실행한다.

validate_g1_traffic.py는 고치지 않고 launch 시작 함수만 감싸 signal_posts 인자를 덧붙인다(r25 run.py와 같은 방식).
평가 정답(상대 자세 등)은 기존 검사기의 평가 전용 범위 그대로이며 제어 입력이 아니다.
"""
import inspect, json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'tests')]
import validate_g1_traffic as target
R18 = ROOT/'artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/registered.json'
out = HERE/'c_stationary_posts/run5'
assert not out.exists()
original = target.start_demo_process
def instrumented(*args, **kwargs):
    bound = inspect.signature(original).bind(*args, **kwargs); bound.apply_defaults()
    bound.arguments['extra_arguments'] = [*bound.arguments['extra_arguments'], 'signal_posts:=stl_61cm_provisional', 'g1_narrow_occluder_bridge:=true']
    return original(*bound.args, **bound.kwargs)
target.start_demo_process = instrumented
registered = json.loads(R18.read_text(encoding='utf-8'))
sys.argv = [str(out) if v == 'OUTPUT' else v for v in registered['args']]
(HERE/'c_stationary_posts/args_run5.json').write_text(json.dumps(dict(source=str(R18.relative_to(ROOT).as_posix()), args=sys.argv,
    added_launch_arguments=['signal_posts:=stl_61cm_provisional','g1_narrow_occluder_bridge:=true']), ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')
raise SystemExit(target.main())
