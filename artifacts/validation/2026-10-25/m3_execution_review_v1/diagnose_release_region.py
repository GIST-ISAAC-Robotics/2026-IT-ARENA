"""run19 복귀 보류의 관측 원인만 읽기 전용 분해한다. 출력/제어/관문은 바꾸지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from arena_autonomy.follow_trace import replay_events
import arena_autonomy.m3_execution_trial as mx


def main():
    source, output = map(Path, sys.argv[1:])
    if output.exists():
        raise FileExistsError(output)
    raw = source.read_bytes()
    events = [json.loads(x) for x in raw.decode().splitlines()]
    rows = []
    original_gate, original_support = mx.release_gate, mx.depth_support

    def traced_gate(record, scene, history, frame, state, grid, cfg, execution, stop_distance):
        snapshots = []
        capture = 12.5 <= scene.t_ref_s <= 14.0

        def traced_support(s, q, pad):
            result = original_support(s, q, pad)
            if capture and s is scene:
                snapshots.append((np.array(q), np.array(pad), np.array(result)))
            return result

        mx.depth_support = traced_support
        try:
            detail = original_gate(record, scene, history, frame, state, grid, cfg, execution, stop_distance)
        finally:
            mx.depth_support = original_support
        if capture and detail['ok'] and snapshots:
            queries, pad, supported = snapshots[-1]
            rest = queries[~supported]
            along, lateral = frame.project(rest)
            ds = along-state['s0']
            hits = np.asarray(grid.returns).reshape(-1, 2)
            hit_pad = np.asarray([pad[i] for i,r in enumerate(scene.rays) if r.status == 'return'])
            surface = np.any(np.linalg.norm(rest[:,None,:]-hits[None,:,:], axis=2) <= hit_pad[None,:], axis=1)
            blocked = ~(mx._both_sides_returned(scene, rest) & (surface | (ds > cfg.band_ahead_m)))
            assert int(blocked.sum()) == detail['blocked_cells']
            origins = np.array([r.ray_origin_xy_m for r in scene.rays])
            directions = np.array([r.direction_xy for r in scene.rays])
            statuses = np.array([r.status for r in scene.rays])
            categories = Counter()
            for q in rest[blocked]:
                d = q-origins
                angle = np.arctan2(directions[:,0]*d[:,1]-directions[:,1]*d[:,0],
                                   directions[:,0]*d[:,0]+directions[:,1]*d[:,1])
                lo = int(np.argmin(np.where(angle >= -1e-10, np.maximum(angle,0.), np.inf)))
                hi = int(np.argmin(np.where(angle <= 1e-10, np.maximum(-angle,0.), np.inf)))
                categories['/'.join(sorted((str(statuses[lo]), str(statuses[hi]))))] += 1
            blocked_xy = rest[blocked]
            row = {k:detail[k] for k in ('reason','region_cells','possible_cells','blocked_cells','grow_m','body_cells')}
            row.update(reference_s=scene.t_ref_s, side=record.side,
                       nearest_ray_status_pairs=dict(categories),
                       blocked_xy_m=blocked_xy.tolist(),
                       blocked_longitudinal_range_m=[float(ds[blocked].min()),float(ds[blocked].max())] if blocked.any() else None,
                       blocked_lateral_range_m=[float(lateral[blocked].min()),float(lateral[blocked].max())] if blocked.any() else None)
            rows.append(row)
        return detail

    mx.release_gate = traced_gate
    try:
        report = replay_events(events)
    finally:
        mx.release_gate, mx.depth_support = original_gate, original_support
    result = dict(scope='instrumented original decision replay; only source-matched existing region classifications, no policy change',
                  calls=report['calls'], matched=report['matched'], passed=report['passed'],
                  input_sha256=hashlib.sha256(raw).hexdigest(),
                  input_unchanged=hashlib.sha256(source.read_bytes()).hexdigest()==hashlib.sha256(raw).hexdigest(), rows=rows)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k != 'rows'}, ensure_ascii=False))
    for r in rows:
        print(json.dumps({k:v for k,v in r.items() if k!='blocked_xy_m'}, ensure_ascii=False))
    assert result['passed'] and result['input_unchanged'] and rows


if __name__ == '__main__':
    main()
