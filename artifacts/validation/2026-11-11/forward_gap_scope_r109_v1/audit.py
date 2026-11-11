"""r107 마지막 정지의 원천/현재 결측 방향. 저장 판단을 바꾸지 않는다."""
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.scan_input import scan_angles, scan_cell_widths, scan_times, scan_max_gap
from arena_autonomy.scene_motion import relative_poses
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision


def main():
    source = A.parent/'scan_start_phase_r107_v1/two_actor_run1/run1/follow_decisions.jsonl'
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    rows = [json.loads(x) for x in source.open(encoding='utf8')]
    follow, motion = make_controller(rows[0]['data'])
    original = PartialObservationPolicy.evaluate
    active, captures, mismatches = {}, [], []

    def inspect(self, scan, points, history, now, speed, *args, **kwargs):
        result = original(self, scan, points, history, now, speed, *args, **kwargs)
        if active['seq'] not in (919, 929, 939, 959, 969) or result.get('reason') != 'forward_gap_too_wide':
            return result
        values = np.asarray(scan.ranges)
        angles = (scan_angles(scan)+history.config.lidar_yaw_rad+math.pi)%math.tau-math.pi
        order = np.argsort(angles)
        front = order[np.abs(angles[order]) <= math.radians(100)+1e-6]
        missing = ~(np.isfinite(values)&(values>=scan.range_min)&(values<=scan.range_max))
        excused = getattr(self, 'structure_excused', None)
        if excused is not None:
            missing &= ~np.asarray(excused, dtype=bool)
        ids = np.flatnonzero(missing[front])
        runs = np.split(ids, np.flatnonzero(np.diff(ids)>1)+1)
        cells = scan_cell_widths(scan)
        stamps = scan_times(scan)
        step = scan_max_gap(scan)
        groups = []
        for run in runs:
            indices = front[run]
            poses = relative_poses(history, stamps[indices], now)
            current = (angles[indices]+poses[:, 2]+math.pi)%math.tau-math.pi
            central = ((np.abs(angles[indices])<=math.radians(30)+step/2)
                       | (np.abs(current)<=math.radians(30)+step/2))
            neighbors = front[[run[0]-1, run[-1]+1]] if run[0]>0 and run[-1]<len(front)-1 else []
            groups.append(dict(indices=indices.tolist(), source_deg=np.degrees(angles[indices]).tolist(),
                current_deg=np.degrees(current).tolist(), cell_deg=np.degrees(cells[indices]).tolist(),
                width_deg=math.degrees(float(sum(cells[indices]))),
                central_cell_width_deg=math.degrees(float(sum(cells[indices][central]))),
                touches_forward=bool(np.any(central)), neighbor_indices=list(map(int, neighbors)),
                neighbor_ranges=[float(values[i]) for i in neighbors]))
        captures.append(dict(seq=active['seq'], reference_s=now, checked_speed_mps=speed,
                             result=result, groups=groups))
        return result

    PartialObservationPolicy.evaluate = inspect
    try:
        for row in rows[1:]:
            if row['kind']=='reset':
                follow.reset()
            elif row['kind']=='call':
                active['seq']=row['seq']
                actual=freeze(replay_call(follow,row['data']['input'],motion))
                if not equal_output(comparable_decision(actual),comparable_decision(row['data']['output'])):
                    mismatches.append(row['seq'])
    finally:
        PartialObservationPolicy.evaluate=original
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    with (A/'audit.json').open('x',encoding='utf8') as stream:
        json.dump(freeze(dict(source=source.relative_to(ROOT).as_posix(),sha256=digest,
                            mismatches=mismatches,captures=captures)),stream,indent=2)
    print(json.dumps(dict(mismatches=mismatches,captures=[dict(seq=c['seq'],groups=c['groups']) for c in captures])))


if __name__=='__main__':
    main()
