"""하나의 강체라는 가정에서 가능한 영역이 줄어드는지 진단. 제어 채택·물체 연관 증거가 아니다."""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

from arena_autonomy.follow_trace import replay_events
import arena_autonomy.m3_execution_trial as mx


def all_diameter_constraints(queries, points, radii):
    """같은 물체의 모든 측정점이라는 전제에서만 필요한 조건이다."""
    if not len(points):
        raise ValueError('positive measured points required')
    distance = np.linalg.norm(queries[:,None,:]-points[None,:,:], axis=2)
    return np.all(distance <= radii[None,:]+1e-10, axis=1)


def main():
    source, output = map(Path, sys.argv[1:])
    if output.exists():
        raise FileExistsError(output)
    raw = source.read_bytes()
    events = [json.loads(x) for x in raw.decode().splitlines()]
    rows, seen = [], set()
    original_gate, original_support = mx.release_gate, mx.depth_support

    def compared_gate(record, scene, history, frame, state, grid, cfg, execution, stop_distance):
        key = (record.side, record.scene.t_ref_s)
        capture = record.possible_xy_m is None and key not in seen
        captured = []

        def observe(s, q, pad):
            result = original_support(s, q, pad)
            if capture and s is scene:
                captured.append((np.array(q), np.array(result)))
            return result

        mx.depth_support = observe
        try:
            result = original_gate(record, scene, history, frame, state, grid, cfg, execution, stop_distance)
        finally:
            mx.depth_support = original_support
        if capture and captured:
            seen.add(key)
            queries, free = captured[-1]
            points = mx.transport_points(record.points, history, record.scene.t_ref_s, scene.t_ref_s)
            radii = (math.hypot(cfg.max_vehicle_length_m,cfg.max_vehicle_width_m)
                     + execution.history_motion_bound_mps*(scene.t_ref_s-record.acquired)
                     + .025+cfg.budget.range_bias_m + math.sqrt(2)*mx.RELEASE_GRID_M)
            keep = all_diameter_constraints(queries, points, radii)
            rest = queries[~free]
            hypothetical = queries[~free & keep]
            pad = mx.ray_padding(scene,cfg,execution.history_motion_bound_mps)
            before_masked = mx.masked_only(scene,rest,pad)
            after_masked = mx.masked_only(scene,hypothetical,pad)
            rows.append(dict(reference_s=scene.t_ref_s,side=record.side,measured_points=len(points),
                old_reason=result['reason'], old_possible_cells=int((~free).sum()),
                conditional_possible_cells=int((~free & keep).sum()),
                old_masked_only_cells=int(before_masked.sum()), conditional_masked_only_cells=int(after_masked.sum()),
                point_extent_m=np.ptp(points,axis=0).tolist(),
                removed_possible_xy_m=queries[~free & ~keep].tolist(),
                meaning='all returns belonging to one rigid body is NOT established; reduction cannot be used as free space'))
        return result

    # 반례: 서로 다른 두 물체가 각각 지름 .25 m 이하여도 모든 점을 한 물체로 묶으면
    # 첫 물체의 실제 점유 (0,0)까지 제거된다. 단순 교집합은 연관 없이 채택할 수 없다.
    q=np.array([[0.,0.]])
    observations=np.array([[0.,0.],[.4,0.]])
    rejected_real_occupancy=not bool(all_diameter_constraints(q,observations,np.array([.25,.25]))[0])
    assert rejected_real_occupancy
    mx.release_gate = compared_gate
    try:
        report=replay_events(events)
    finally:
        mx.release_gate,mx.depth_support=original_gate,original_support
    result=dict(scope='conditional single-body geometry diagnostic only; no source or control changes',
                calls=report['calls'],matched=report['matched'],passed=report['passed'],rows=rows,
                multiple_objects_counterexample_rejects_real_occupancy=rejected_real_occupancy,
                runtime_adoptable=False,input_sha256=hashlib.sha256(raw).hexdigest(),
                input_unchanged=hashlib.sha256(source.read_bytes()).hexdigest()==hashlib.sha256(raw).hexdigest())
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},ensure_ascii=False))
    for row in rows:
        print(json.dumps({k:v for k,v in row.items() if k!='removed_possible_xy_m'},ensure_ascii=False))
    assert result['passed'] and result['input_unchanged'] and rows


if __name__=='__main__':
    main()
