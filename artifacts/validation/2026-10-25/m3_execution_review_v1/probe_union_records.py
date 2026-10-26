"""현재 관측 확인 후 가능 영역 합집합 갱신. 런타임 미연결 합성 시제품."""
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import GRASS, car
from test_local_follow import history
from test_m3_observed_follow import timed_scene


def seed_region(record):
    reach = math.hypot(GRASS.max_vehicle_length_m, GRASS.max_vehicle_width_m)
    p = record.points
    body = mx._cells(p.min(axis=0)-reach, p.max(axis=0)+reach)
    body = body[mx._within(body, p, reach) & ~mx._inside_body(body, GRASS)]
    grow = .5*(record.scene.t_ref_s-min(record.acquired))+.025+GRASS.budget.range_bias_m
    region = mx._cells(body.min(axis=0)-grow, body.max(axis=0)+grow)
    region = region[mx._within(region, body, grow)]
    region = region[~mx._inside_body(region, GRASS)]
    pad = mx.ray_padding(record.scene, GRASS, .5)
    return region[~mx.depth_support(record.scene, region, pad)]


class UnionMemory:
    def __init__(self):
        self.record = None
        self.blocked = None
        self.active = False
        self.confirmations = 0

    def update(self, scene, path, motion):
        frame, grid = mx.ExecutionFrame(path), mx.ReturnGrid(scene, GRASS)
        state = mx.ego_state(frame, GRASS)
        _, masks = mx.side_return_masks(frame, state, grid, GRASS)
        present = int(np.sum(masks[0])) >= GRASS.min_threat_points
        self.active |= present
        previous, gate = self.record, None
        if previous is not None and not self.blocked:
            stop = mx.envelope_distance(1., scene.t_ref_s-scene.source_first_s, .02, mx.ExecutionConfig())
            gate = mx.release_gate(previous, scene, motion, frame, state, grid, GRASS, mx.ExecutionConfig(), stop)
            if not gate['ok']:
                self.blocked = gate['reason']
        renewed = False
        if not self.blocked:
            retained = gate['record'] if gate else None
            if present:
                new = mx.threat_record('left', scene, grid, masks[0])
                current = seed_region(new)
                if retained is not None:
                    current = np.unique(np.vstack((current, retained.possible_xy_m)), axis=0)
                # 전체 영역이 현재 반환으로 설명될 때만 확인 시점을 갱신. 부분 관측은 합집합만 갱신.
                renewed = gate is None or (gate['ok'] and gate['blocked_cells'] == 0)
                detected = scene.t_ref_s if renewed else (previous.detected_at_s if previous.detected_at_s is not None else previous.scene.t_ref_s)
                self.record = mx.ThreatRecord('left', scene, new.points, new.acquired, current, detected)
            elif gate:
                self.record = retained
        confirmed = bool(self.active and not present and not self.blocked and gate and gate['release'])
        self.confirmations = self.confirmations+1 if confirmed else 0
        released = self.confirmations >= GRASS.release_fresh
        if released:
            self.active, self.record = False, None
        return {'present': present, 'released': released, 'latched': self.blocked,
                'renewed': renewed, 'cells': 0 if self.record is None or self.record.possible_xy_m is None else len(self.record.possible_xy_m),
                'gate_reason': gate and gate['reason'], 'blocked_cells': gate and gate.get('blocked_cells')}


def run(kind):
    memory, rows = UnionMemory(), []
    for k in range(50):
        t = round(1.+.1*k, 3)
        ax, bx = max(-.65, .3-.4*(t-1.)), .30+.4*abs(t-3.5)
        a, b = car(ax, .08, -.09, width=.10), car(bx, .24, -.09, width=.10)
        objects = [car(.30+.45*(t-1.), .13, -.09)] if kind == 'clear_forward' else [a, b] if kind == 'both' else [a] if kind == 'a_only' else [b]
        scene, path = timed_scene(t, -.09, extra=objects)
        rows.append({'t': t, **memory.update(scene, path, history(end=t+.01))})
    return {'rows': rows, 'release_times': [r['t'] for r in rows if r['released']],
            'max_cells': max(r['cells'] for r in rows), 'latched': memory.blocked}


if __name__ == '__main__':
    output = Path(sys.argv[1])
    if output.exists():
        raise FileExistsError(output)
    before = hashlib.sha256(Path(mx.__file__).read_bytes()).hexdigest()
    result = {'scope': 'offline union feasibility only; no controller connection', 'source_sha256': before,
              'cases': {k: run(k) for k in ('clear_forward', 'a_only', 'b_only', 'both')}}
    result['source_unchanged'] = before == hashlib.sha256(Path(mx.__file__).read_bytes()).hexdigest()
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({**result, 'cases': {k: {a: b for a, b in v.items() if a != 'rows'} for k, v in result['cases'].items()}}, ensure_ascii=False))
