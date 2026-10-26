"""이전 위협을 새 반환으로 대체하지 않는 장부 대조. 런타임 미연결 합성 시제품."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import GRASS, car
from test_local_follow import history
from test_m3_observed_follow import timed_scene


class Ledger:
    """매 fresh 관측의 가능 영역을 독립 보존. 같은 물체라는 연관 가정 없음.

    현재 release_gate가 해제를 확인한 기록만 제거한다. 만료/운동 누락은 latch하고
    다시 나타난 점으로 해제하지 않는다. 최대 16기록 뒤엔 해제를 거절한다.
    현재 속도/조향 출력에는 연결하지 않는 비용·진행성 비교다.
    """
    def __init__(self):
        self.records = []
        self.blocked = None
        self.active = False
        self.confirmations = 0

    def update(self, scene, path, motion):
        frame = mx.ExecutionFrame(path)
        state = mx.ego_state(frame, GRASS)
        grid = mx.ReturnGrid(scene, GRASS)
        _, masks = mx.side_return_masks(frame, state, grid, GRASS)
        # 합성 시제품은 좌측 3점 촉발만 다룬다. 실제 side_threats 대체가 아니다.
        present = int(np.sum(masks[0])) >= GRASS.min_threat_points
        self.active |= present
        stop = mx.envelope_distance(1., scene.t_ref_s-scene.source_first_s, .02, mx.ExecutionConfig())
        retained, reasons = [], []
        for record in self.records:
            gate = mx.release_gate(record, scene, motion, frame, state, grid, GRASS, mx.ExecutionConfig(), stop)
            reasons.append(gate['reason'])
            if not gate['ok']:
                self.blocked = self.blocked or gate['reason']
            if not gate['release']:
                retained.append(gate['record'])
        self.records = retained
        if present and not self.blocked:
            if len(self.records) >= 16:
                self.blocked = 'record_capacity_exceeded'
            else:
                self.records.append(mx.threat_record('left', scene, grid, masks[0]))
        confirmed = self.active and not present and not self.records and not self.blocked
        self.confirmations = self.confirmations+1 if confirmed else 0
        released = self.confirmations >= GRASS.release_fresh
        if released:
            self.active = False
        return {'present': present, 'records': len(self.records), 'latched': self.blocked,
                'released': released, 'gate_reasons': reasons,
                'cells': sum(0 if r.possible_xy_m is None else len(r.possible_xy_m) for r in self.records)}


def run(kind):
    ledger, rows = Ledger(), []
    for k in range(50):
        t = round(1.+.1*k, 3)
        ax, bx = max(-.65, .3-.4*(t-1.)), .30+.4*abs(t-3.5)
        a, b = car(ax, .08, -.09, width=.10), car(bx, .24, -.09, width=.10)
        if kind == 'clear_forward':
            objects = [car(.30+.45*(t-1.), .13, -.09)]
        else:
            objects = [a, b] if kind == 'both' else [a] if kind == 'a_only' else [b]
        scene, path = timed_scene(t, -.09, extra=objects)
        result = ledger.update(scene, path, history(end=t+.01))
        rows.append({'t': t, **result})
    return {'rows': rows, 'release_times': [r['t'] for r in rows if r['released']],
            'max_records': max(r['records'] for r in rows), 'max_cells': max(r['cells'] for r in rows),
            'latched': ledger.blocked}


if __name__ == '__main__':
    output = Path(sys.argv[1])
    if output.exists():
        raise FileExistsError(output)
    before = hashlib.sha256(Path(mx.__file__).read_bytes()).hexdigest()
    result = {'scope': 'offline ledger feasibility only; no controller connection', 'source_sha256': before,
              'cases': {k: run(k) for k in ('clear_forward', 'a_only', 'b_only', 'both')}}
    result['source_unchanged'] = before == hashlib.sha256(Path(mx.__file__).read_bytes()).hexdigest()
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({**result, 'cases': {k: {a: b for a, b in v.items() if a != 'rows'}
                                       for k, v in result['cases'].items()}}, ensure_ascii=False))
