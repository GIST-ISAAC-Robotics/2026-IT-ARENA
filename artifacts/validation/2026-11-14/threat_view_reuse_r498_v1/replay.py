"""Windows 동일 Python backend의 현재 공통/후보 순차 비교용. ARM 비용시험 아님."""
import copy
import dataclasses
import hashlib
import json
import sys
import time
from types import FunctionType, MethodType
from collections import deque
from pathlib import Path

A = Path(__file__).resolve().parent
R = A.parents[3]
variant = sys.argv[1]
S = A/variant
sys.path[:0] = [str(A/'pydeps'), str(S/'src/arena_autonomy'), str(S/'src/arena_vehicle_interface')]
import numpy as np
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.follow_trace import freeze, comparable_decision
import arena_autonomy.pass_release_trial as ps
import arena_autonomy.traffic_solo_gap as gap


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf8')


def state(value, seen=None, path='$'):
    if seen is None:seen={}
    if value is None or isinstance(value, (str, bool, int, float, np.generic, np.ndarray)):
        return freeze(value)
    if isinstance(value, (FunctionType, MethodType)):
        return dict(callable=value.__module__+'.'+value.__qualname__)
    if id(value) in seen:return dict(state_ref=seen[id(value)])
    seen[id(value)]=path
    if isinstance(value, dict):
        return {str(k): state(v,seen,path+'.'+str(k)) for k, v in value.items()}
    if isinstance(value, (list, tuple, deque)):
        return [state(v,seen,path+'['+str(i)+']') for i,v in enumerate(value)]
    if isinstance(value, (set, frozenset)):
        return sorted((state(v,seen,path+'.set') for v in value), key=lambda v: json.dumps(v, sort_keys=True))
    if dataclasses.is_dataclass(value):
        return {f.name: state(getattr(value, f.name),seen,path+'.'+f.name) for f in dataclasses.fields(value)}
    if hasattr(value, '__dict__') and type(value).__module__.startswith('arena_autonomy'):
        return dict(type=type(value).__qualname__, attributes=state(vars(value),seen,path+'.attributes'))
    raise TypeError('unrecorded state type: '+str(type(value)))


def main():
    outdir = A/'windows_run_v3'/variant
    outdir.mkdir(parents=True, exist_ok=False)
    p = R/'artifacts/validation/2026-11-14/jetson_lap_cost_r493_v1/bundle/inputs.json'
    raw = p.read_bytes()
    data = json.loads(raw)
    header = copy.deepcopy(data['header'])
    changes = {}
    for name, value in [('numeric_execution', 'python'), ('current_corner_budget', 'off')]:
        changes[name] = dict(original=header.get(name), derived=value)
        header[name] = value
    dump(outdir/'derived_header.json', dict(header=header, changes=changes,
        scope='plan only; Python backend; not native equivalence or current-corner validation'))
    view_calls = []
    actual_view = ps.threat_view
    def measured_view(scene, path, cfg):
        view_calls.append(dict(scene=id(scene), path=copy.deepcopy(path), config=id(cfg)))
        return actual_view(scene, path, cfg)
    ps.threat_view = measured_view
    matches = []
    if hasattr(gap, 'ThreatViewTransfer'):
        original_match = gap.ThreatViewTransfer.matches
        def matched(owner, scene, path, cfg):
            answer = original_match(owner, scene, path, cfg)
            matches.append(dict(hit=answer, scene_same=owner.scene is scene,
                                config_equal=owner.config == cfg,
                                path_equal=np.array_equal(owner.path, np.asarray(path if path is not None else []).reshape(-1,2))))
            return answer
        gap.ThreatViewTransfer.matches = matched
    evaluator = Evaluator('plan', header)
    generation = None
    rows, resets = [], []
    started = time.perf_counter()
    try:
        with (outdir/'outputs.jsonl').open('w', encoding='utf8') as stream:
            for index, item in enumerate(data['plans']):
                key = item['generation'], item['sequence']
                if key[0] != generation:
                    evaluator.reset(item['reset_reason'])
                    resets.append(dict(index=index, generation=key[0], reason=item['reset_reason']))
                    generation = key[0]
                view_calls.clear(); matches.clear()
                before = json.dumps(item['input'], sort_keys=True, allow_nan=False)
                output = evaluator(item['input'])
                assert before == json.dumps(item['input'], sort_keys=True, allow_nan=False)
                current_state = state(evaluator.follow)
                record = dict(key=key, output=output, state=current_state)
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
                row = dict(index=index, key=key, reference_s=item['input']['reference_s'],
                           threat_view_calls=len(view_calls), transfers=list(matches))
                rows.append(row)
                if key[1] in (717, 1304, 1311):
                    print(json.dumps(row), flush=True)
                if (index+1)%100 == 0:
                    print(json.dumps(dict(completed=index+1)), flush=True)
    finally:
        evaluator.close()
    assert hashlib.sha256(p.read_bytes()).digest() == hashlib.sha256(raw).digest()
    summary = dict(variant=variant, rows=rows, plan_calls=len(rows), resets=resets,
        input_unchanged=True, input_sha256=hashlib.sha256(raw).hexdigest(),
        total_view_calls=sum(r['threat_view_calls'] for r in rows),
        transfer_hits=sum(v['hit'] for r in rows for v in r['transfers']),
        transfer_fallbacks=sum(not v['hit'] for r in rows for v in r['transfers']),
        elapsed_wall_s=time.perf_counter()-started, numpy_version=np.__version__,
        state_injected=False, repeated_calls=0, native_used=False, throughput_claim=False)
    dump(outdir/'summary.json', summary)
    print(json.dumps({k:v for k,v in summary.items() if k != 'rows'}), flush=True)


if __name__ == '__main__':
    main()
