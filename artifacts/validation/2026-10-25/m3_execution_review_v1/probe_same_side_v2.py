"""운동 이력 범위를 맞춘 같은 측면 재관측 대조. 물체 정답은 사후 해석에만 사용한다."""
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


def case(kind):
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    rows = []
    for k in range(61):
        t = round(1.+.1*k, 3)
        # 속도 상계 0.4 m/s. 두 물체의 횡면 간격 0.06 m, 벽과 분리.
        ax = max(-.65, .3-.4*(t-1.))
        bx = .30+.4*abs(t-3.5)
        a, b = car(ax, .08, -.09, width=.10), car(bx, .24, -.09, width=.10)
        objects = [a, b] if kind == 'both' else [a] if kind == 'a_only' else [b]
        scene, path = timed_scene(t, -.09, extra=objects)
        motion = history(end=t+.01)
        motion.poses(np.array([t-.2, t]), t)  # 검사 시각의 실제 이력 포함을 먼저 검증
        frame = mx.ExecutionFrame(path)
        state = mx.ego_state(frame, GRASS)
        grid = mx.ReturnGrid(scene, GRASS)
        old = trial.records.get('left')
        gate = None
        if old is not None:
            stop = mx.envelope_distance(1., scene.t_ref_s-scene.source_first_s, .02, trial.execution)
            gate = mx.release_gate(old, scene, motion, frame, state, grid, GRASS, trial.execution, stop)
        out = trial.step(scene, [], path, .35, padding(scene), history=motion)
        new = trial.records.get('left')
        # 반환이 어느 상자에 닿았는지 아래 사후 통계에만 사용. 제어기 전달 금지.
        pts = np.asarray(grid.returns)
        def hit_count(poly):
            lo, hi = poly.min(axis=0), poly.max(axis=0)
            return int(np.sum(np.all((pts >= lo-1e-9) & (pts <= hi+1e-9), axis=1)))
        changed = old is not None and new is not None and not np.array_equal(old.points, new.points)
        rows.append({'t': t, 'a_x': ax, 'b_x': bx, 'a_returns': hit_count(a), 'b_returns': hit_count(b),
            'mode': out['mode'], 'released': bool(out.get('released_to_baseline')),
            'left_present': out['threats']['left']['present'],
            'old_gate_reason': None if gate is None else gate['reason'],
            'old_gate_release': None if gate is None else gate['release'],
            'record_points_changed': changed,
            'new_min_xy': None if new is None else new.points.min(axis=0).tolist(),
            'new_max_xy': None if new is None else new.points.max(axis=0).tolist(),
            'record_reference_s': None if new is None else new.scene.t_ref_s,
            'release_gate': out.get('release_gate')})
    return {'rows': rows, 'release_times': [r['t'] for r in rows if r['released']],
        'hidden_a_while_released': [r['t'] for r in rows if r['released'] and r['a_returns'] == 0 and kind != 'b_only'],
        'changed_old_blocked_after_a_hidden': [r['t'] for r in rows if r['record_points_changed']
            and r['old_gate_release'] is False and r['a_returns'] == 0 and kind == 'both'],
        'motion_unavailable_calls': sum(r['old_gate_reason'] == 'motion_unavailable' for r in rows)}


if __name__ == '__main__':
    output = Path(sys.argv[1])
    if output.exists():
        raise FileExistsError(output)
    before = hashlib.sha256(Path(mx.__file__).read_bytes()).hexdigest()
    result = {'scope': 'static-per-scan synthetic boxes, 0.4 m/s relative translation; not physical driving',
              'source_sha256': before, 'cases': {kind: case(kind) for kind in ('a_only', 'b_only', 'both')}}
    result['source_unchanged'] = before == hashlib.sha256(Path(mx.__file__).read_bytes()).hexdigest()
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({**result, 'cases': {k: {a: b for a, b in v.items() if a != 'rows'}
                                       for k, v in result['cases'].items()}}, ensure_ascii=False))
