"""새 가능 영역이 이전 영역을 덮는 경우만 재기준화하는 미연결 시제품."""
from dataclasses import replace
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


def current_region(record, frame):
    reach = math.hypot(GRASS.max_vehicle_length_m, GRASS.max_vehicle_width_m)
    body = mx._cells(record.points.min(axis=0)-reach, record.points.max(axis=0)+reach)
    body = body[mx._within(body, record.points, reach) & ~mx._inside_body(body, GRASS)]
    pad = mx.ray_padding(record.scene, GRASS, .5)
    body = body[~mx.depth_support(record.scene, body, pad)]
    grow = .5*(record.scene.t_ref_s-min(record.acquired))+.025+GRASS.budget.range_bias_m
    region = mx._cells(body.min(axis=0)-grow, body.max(axis=0)+grow)
    region = region[mx._within(region, body, grow)]
    _, lateral = frame.project(region)
    region = region[(np.abs(lateral) < GRASS.wall_offset_m-GRASS.wall_tolerance_m) & ~mx._inside_body(region, GRASS)]
    return region[~mx.depth_support(record.scene, region, pad)], body, grow


class CoverageMemory:
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
            elif gate.get('blocked_cells') == 0 and gate['record'] is not None:
                gate['record'] = replace(gate['record'], detected_at_s=scene.t_ref_s)
        renewed, outside = False, None
        if not self.blocked:
            retained = gate['record'] if gate else None
            if present:
                new = mx.threat_record('left', scene, grid, masks[0])
                current, body, grow = current_region(new, frame)
                if retained is not None:
                    old = retained.possible_xy_m
                    # 기존 표본이 새 body+grow 안인지 확인. 합집합에 원래 표본도 실제로 보존한다.
                    covered = mx._within(old, body, grow-1e-9)
                    outside = int(np.sum(~covered))
                    current = np.unique(np.vstack((current, old)), axis=0)
                    renewed = outside == 0 or gate.get('blocked_cells') == 0
                else:
                    renewed = True
                detected = scene.t_ref_s if renewed else (previous.detected_at_s if previous.detected_at_s is not None else previous.scene.t_ref_s)
                self.record = mx.ThreatRecord('left', scene, new.points, new.acquired, current, detected)
            elif gate and retained is not None:
                self.record = retained
        confirmed = bool(self.active and not present and not self.blocked and gate and gate['release'])
        self.confirmations = self.confirmations+1 if confirmed else 0
        released = self.confirmations >= GRASS.release_fresh
        if released:
            self.active, self.record = False, None
        return {'present': present, 'released': released, 'latched': self.blocked,
                'renewed': renewed, 'old_outside_fresh': outside,
                'cells': 0 if self.record is None or self.record.possible_xy_m is None else len(self.record.possible_xy_m),
                'gate_reason': gate and gate['reason']}


def run(kind):
    memory, rows = CoverageMemory(), []
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
    result = {'scope': 'offline current positive observation renewal; original unknown expiry kept; no controller connection', 'source_sha256': before,
              'cases': {k: run(k) for k in ('clear_forward', 'a_only', 'b_only', 'both')}}
    result['source_unchanged'] = before == hashlib.sha256(Path(mx.__file__).read_bytes()).hexdigest()
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({**result, 'cases': {k: {a: b for a, b in v.items() if a != 'rows'} for k, v in result['cases'].items()}}, ensure_ascii=False))
