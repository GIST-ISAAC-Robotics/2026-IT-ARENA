"""고정된 실제 저장 current만 순차 평가한다. 장치/ROS/구동 접근 없음."""
import functools
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
VERSION = sys.argv[1]
assert VERSION in ('base', 'stage')
SOURCE = ROOT / VERSION / 'src'
sys.path[:0] = [str(SOURCE / 'arena_autonomy'), str(SOURCE / 'arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.runtime_dual_state import current_request
from arena_autonomy import pass_release_trial as ps, known_structure
from arena_autonomy.race_runtime_policy import profile_evidence


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def main():
    data = json.loads((ROOT / 'inputs.json').read_bytes())
    records, headers = data['records'], data['headers']
    requests = {r['id']: current_request(r['plan_output'], r['plan_input'], r['current'],
                retained=r['retained'], crop_current_selection=True) for r in records}
    hashes = {key: digest(value) for key, value in requests.items()}
    workers, constructor_ms = {}, {}
    for name, header in headers.items():
        start = time.perf_counter_ns()
        workers[name] = Evaluator('current', header)
        constructor_ms[name] = (time.perf_counter_ns() - start) / 1e6
    rows, swaps, active = [], [], []
    def install(module, name):
        original = getattr(module, name)
        @functools.wraps(original)
        def measured(*args, **kwargs):
            start = time.perf_counter_ns()
            result = original(*args, **kwargs)
            elapsed = time.perf_counter_ns() - start
            active.append(dict(function=module.__name__ + '.' + name, ms=elapsed / 1e6))
            return result
        swaps.append((module, name, original))
        setattr(module, name, measured)
    first_outputs = {}
    try:
        first = records[0]
        start = time.perf_counter_ns()
        warmup_output = workers[first['dataset']](requests[first['id']])
        warmup = dict(id=first['id'], ms=(time.perf_counter_ns() - start) / 1e6,
                      output_sha256=digest(warmup_output), included_in_statistics=False)
        for round_index in range(3):
            if round_index == 1:
                install(ps, 'motion_region')
                install(profile_evidence, 'profile_evidence')
                if hasattr(known_structure, 'current_mask_shadow'):
                    install(known_structure, 'current_mask_shadow')
            for record in records:
                key = record['id']
                assert digest(requests[key]) == hashes[key]
                active = []
                start = time.perf_counter_ns()
                output = workers[record['dataset']](requests[key])
                elapsed_ms = (time.perf_counter_ns() - start) / 1e6
                assert digest(requests[key]) == hashes[key]
                assert output['runtime_authorized'] is False and output['independent_guard_required'] is True
                if round_index == 0:
                    first_outputs[key] = output
                assert output == first_outputs[key], (key, round_index, 'instrumentation changed output')
                if key == first['id']:
                    assert output == warmup_output
                rows.append(dict(id=key, version=VERSION, round=round_index, evaluator_ms=elapsed_ms,
                    instrumented=round_index > 0, helper_calls=active, output=output,
                    output_sha256=digest(output), input_sha256=hashes[key], input_unchanged=True,
                    uninstrumented_output_exact=True,
                    original_x86_output_exact=output == record['expected']))
    finally:
        for module, name, original in reversed(swaps):
            setattr(module, name, original)
        for worker in workers.values():
            worker.close()
    imports = {name: str(Path(module.__file__).resolve()) for name, module in sys.modules.items()
        if name.startswith(('arena_autonomy', 'arena_vehicle_interface')) and getattr(module, '__file__', None)}
    assert all(Path(path).is_relative_to(SOURCE) for path in imports.values()), imports
    assert len(rows) == 12
    result = dict(version=VERSION, pid=os.getpid(), machine=os.uname().machine, rows=rows,
        warmup=warmup, constructor_ms=constructor_ms, imports=imports,
        measured_calls=12, warmup_calls=1, instrumentation_output_exact=True,
        physical_measurement=False, sensor_access=False, ros_started=False)
    with (ROOT / 'results' / (VERSION + '.json')).open('x') as out:
        json.dump(result, out, indent=2)
    print(json.dumps(dict(version=VERSION, calls=12, warmups=1, instrumentation_output_exact=True)))


if __name__ == '__main__':
    main()
