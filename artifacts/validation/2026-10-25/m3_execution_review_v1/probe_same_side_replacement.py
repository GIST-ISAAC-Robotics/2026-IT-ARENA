"""같은 측면 두 물체가 다른 궤적으로 보일 때 기록 갱신 진단. 운영 변경/안전 증명 아님."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import GRASS, car
from test_local_follow import history
from test_m3_execution_trial import padding
from test_m3_observed_follow import timed_scene


def run():
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    motion = history(end=8.)
    rows = []
    # 자차는 정지, 상대차 A는 0.4 m/s로 뒤로 이동 후 정지. B는 다른 횡위치에서 0.4 m/s 전후진.
    # 이는 합성 기하/상태 시험이며 실제 경기 운전 패턴이나 차량 동역학을 대표하지 않는다.
    for k in range(61):
        t = round(1.+.1*k, 3)
        ax = max(-.65, .3-.4*(t-1.))
        bx = .30+.4*abs(t-3.5)
        a, b = car(ax, .13, -.09), car(bx, .32, -.09)
        scene, path = timed_scene(t, -.09, extra=[a, b])
        frame = mx.ExecutionFrame(path)
        state = mx.ego_state(frame, GRASS)
        grid = mx.ReturnGrid(scene, GRASS)
        old = trial.records.get('left')
        old_gate = None
        if old is not None:
            stop = mx.envelope_distance(1., scene.t_ref_s-scene.source_first_s, .02, trial.execution)
            old_gate = mx.release_gate(old, scene, motion, frame, state, grid, GRASS, trial.execution, stop)
        out = trial.step(scene, [], path, .35, padding(scene), history=motion)
        new = trial.records.get('left')
        old_points = None if old is None else old.points
        new_points = None if new is None else new.points
        points_changed = old_points is not None and new_points is not None and not np.array_equal(old_points, new_points)
        # 실제 물체 식별을 제어기에 전달하지 않는다. 아래 좌표/간격은 사후 기하 설명만이다.
        rows.append({'t': t, 'actor_a_x': ax, 'actor_b_x': bx, 'lateral_body_gap_m': .32-.13-.15,
                     'mode': out['mode'], 'released': bool(out.get('released_to_baseline')),
                     'left_present': out['threats']['left']['present'],
                     'old_gate_reason': None if old_gate is None else old_gate['reason'],
                     'old_gate_allowed_release': None if old_gate is None else old_gate['release'],
                     'record_points_changed': points_changed,
                     'old_record_scene_s': None if old is None else old.scene.t_ref_s,
                     'new_record_scene_s': None if new is None else new.scene.t_ref_s,
                     'new_min_x': None if new is None else float(new.points[:, 0].min()),
                     'new_max_x': None if new is None else float(new.points[:, 0].max()),
                     'new_min_y': None if new is None else float(new.points[:, 1].min()),
                     'new_max_y': None if new is None else float(new.points[:, 1].max())})
    return {'scope': 'two disjoint synthetic boxes, current sensor rays only; no physical driving or association guarantee',
            'rows': rows, 'release_times': [r['t'] for r in rows if r['released']],
            'changed_while_old_gate_blocked': [r['t'] for r in rows if r['record_points_changed']
                                             and r['old_gate_allowed_release'] is False]}


if __name__ == '__main__':
    output = Path(sys.argv[1])
    if output.exists():
        raise FileExistsError(output)
    before = hashlib.sha256(Path(mx.__file__).read_bytes()).hexdigest()
    result = run()
    result['source_sha256'] = before
    result['source_unchanged'] = before == hashlib.sha256(Path(mx.__file__).read_bytes()).hexdigest()
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k: v for k, v in result.items() if k != 'rows'}, ensure_ascii=False))
