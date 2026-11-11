"""기존 평가 한 번 전에 요청만 감속하는 저장 입력 대조. 주행 증명은 아니다."""
import hashlib
import inspect
import json
from pathlib import Path
import textwrap
import types

import numpy as np
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.scan_input import scan_angles, scan_cell_widths, scan_max_gap
from arena_autonomy.measurement_scan import MeasurementScan

ROOT = Path(__file__).resolve().parents[4]
TRACE = ROOT/'artifacts/validation/2026-11-11/c1_native_sampling_v1/solo_time_fix/follow_decisions.jsonl'
OUTPUT = ROOT/'build/c1_native_sampling_2026_11_11/partial_requested_speed_compare.json'

def proposal(policy, scan, yaw, requested, explained):
    if not policy.lateral_metric_enabled or requested <= .2:
        return requested
    values = np.asarray(scan.ranges, dtype=float)
    angles = (scan_angles(scan)+yaw+np.pi) % (2*np.pi)-np.pi
    front = np.argsort(angles)
    front = front[np.abs(angles[front]) <= np.deg2rad(100)+1e-6]
    missing = ~(np.isfinite(values)&(values>=scan.range_min)&(values<=scan.range_max))
    if explained:
        missing[list(explained)] = False
    ids = np.flatnonzero(missing[front])
    if not len(ids):
        return requested
    runs = np.split(ids, np.flatnonzero(np.diff(ids)>1)+1)
    cells = scan_cell_widths(scan)
    widest = max(float(sum(cells[front[r]])) if isinstance(scan, MeasurementScan)
                 else len(r)*scan_max_gap(scan) for r in runs)
    return min(requested, .2) if widest > np.deg2rad(3)+1e-7 else requested

old = inspect.getsource(LocalFollow._partial_command)
needle = 'cfg=self.config\n        observation='
assert old.count(needle)==1
new = old.replace(needle, '''cfg=self.config
        requested=proposal(self.partial_policy,raw_scan,history.config.lidar_yaw_rad,output[0],explained_now)
        if requested!=output[0]:
            output=(requested,output[1],{**output[2],'partial_requested_speed_candidate':requested})
        observation=''', 1)
namespace = dict(LocalFollow._partial_command.__globals__, proposal=proposal)
exec(compile(textwrap.dedent(new), '<partial-request-candidate>', 'exec'), namespace)
candidate_method = namespace['_partial_command']
rows=[]; mismatches=0
for line in TRACE.open(encoding='utf-8'):
    event=json.loads(line);data=event['data']
    if event['kind']=='header':
        motion=MotionConfig(**data['motion_config']);cfg=FollowConfig(**data['follow_config'])
        baseline=LocalFollow(cfg);candidate=LocalFollow(cfg)
        candidate._partial_command=types.MethodType(candidate_method,candidate)
    elif event['kind']=='reset':baseline.reset();candidate.reset()
    elif event['kind']=='call':
        a,b=[replay_call(c,data['input'],motion) for c in (baseline,candidate)]
        mismatches+=not equal_output(comparable_decision(freeze(a)),comparable_decision(data['output']))
        rows.append(dict(seq=event['seq'],time_s=data['input']['reference_s'],measured=data['input']['speed_mps'],
            original=a[0],candidate=b[0],original_reason=a[2]['reason'],candidate_reason=b[2]['reason'],
            proposal=b[2].get('partial_requested_speed_candidate'),
            observation=b[2].get('partial_observation',{}).get('reason')))
summary=dict(scope='saved_input_counterfactual_not_closed_loop',runtime_modified=False,
    trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),baseline_mismatches=mismatches,
    calls=len(rows),original_zero_to_positive=sum(r['original']==0 and r['candidate']>0 for r in rows),
    positive_to_zero=sum(r['original']>0 and r['candidate']==0 for r in rows),
    proposal_calls=sum(r['proposal'] is not None for r in rows),last=rows[-1])
if OUTPUT.exists():raise FileExistsError(OUTPUT)
OUTPUT.write_text(json.dumps(dict(summary=summary,rows=rows),indent=2),encoding='utf-8')
print(json.dumps(summary))
