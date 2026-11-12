"""r338 저장 현재 입력의 작은 양수 상한 비교. 실제 발행/주행 없음."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
spec = importlib.util.spec_from_file_location('r296_measure', A.parent/'current_input_jetson_r296_v1/measure_cost.py')
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)


def main():
    assert not (A/'report.json').exists()
    source = A.parent/'published_route_drive_r338_v1/run1/follow_decisions.jsonl'
    with source.open() as stream:
        header = json.loads(next(stream))['data']
        calls = [e['data'] for e in map(json.loads, stream)
                 if e['kind'] == 'call' and 'current_rebase' in e['data']]
    selected = sorted(set(np.linspace(0, len(calls)-1, 12, dtype=int).tolist()))
    cfg = old.FollowConfig(**header['follow_config'])
    motion = old.MotionConfig(**header['motion_config'])
    old.configure('native_v1')
    outputs, rows = [], []
    fields = ('speed_mps','steering_rad','reason','reference_s','original_reference_s','source_first_s','path_source_s')
    for index in selected:
        record = calls[index]
        current = record['current_rebase']['input']
        d, scan, history = old.restore_call(current, motion)
        scene = old.motion_scene(scan, history, d['reference_s'], timing_verified=d['timing_verified'],
                                epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
        before = old.freeze(old.candidate.rebase(record['output'], old.plan_reference(record['input']),
                                  current, scene, history, cfg, require_current_region=True))
        expected = record['current_rebase']['output']
        same = old.equal_output({k:before.get(k) for k in fields}, {k:expected.get(k) for k in fields})
        if not same:
            (A/'replay_mismatch.json').write_text(json.dumps(dict(index=index, before=before, expected=expected), indent=2))
            raise AssertionError('original operational replay mismatch')
        candidates = []
        for cap in (.45, .35, .25, .15):
            assert 0 < cap < record['output'][0]
            low = [cap, record['output'][1], record['output'][2]]
            start = time.perf_counter_ns()
            result = old.freeze(old.candidate.rebase(low, old.plan_reference(record['input']), current,
                                                     scene, history, cfg, require_current_region=True))
            elapsed = (time.perf_counter_ns()-start)/1e6
            candidates.append(dict(cap_mps=cap, result=result, elapsed_ms=elapsed))
            region = (result.get('current_route_proof') or {}).get('region', {})
            rows.append(dict(index=index, reference_s=d['reference_s'], measured_speed_mps=d['speed_mps'],
                cap_mps=cap, actual_target_mps=result['speed_mps'], reason=result['reason'],
                original_steering_rad=before['steering_rad'], new_steering_rad=result['steering_rad'],
                tail_m=region.get('masked_tail_extent_m'), unobserved=region.get('unobserved'),
                masked_front=region.get('masked_front_samples'), elapsed_ms=elapsed))
        outputs.append(dict(index=index, original_match=same, input=record, before=before, candidates=candidates))
    sources = {source.relative_to(ROOT).as_posix():hashlib.sha256(source.read_bytes()).hexdigest()}
    for module in list(sys.modules.values()):
        path = getattr(module, '__file__', None)
        if path:
            path = Path(path).resolve()
            if path.is_file() and path.is_relative_to(ROOT) and path.suffix in ('.py','.so'):
                sources[path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    summary = {str(cap):dict(positive=sum(r['actual_target_mps']>0 for r in rows if r['cap_mps']==cap),
        count=sum(r['cap_mps']==cap for r in rows)) for cap in (.45,.35,.25,.15)}
    (A/'report.json').write_text(json.dumps(dict(scope='fixed input only; no controller connection',
        originals=len(calls), selected=selected, original_command_matches=len(selected), source_hashes=sources,
        summary=summary, rows=rows), indent=2)+'\n')
    (A/'evidence.json').write_text(json.dumps(outputs, separators=(',',':'))+'\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
