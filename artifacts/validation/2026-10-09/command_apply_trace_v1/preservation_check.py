#!/usr/bin/env python3
"""Gate B 실행 보존 검사: 새 빌드(추적 끔) 실행 사본이 10/8 R1과 같은지, 추적 켬 실행과의 차이가 추적 요소뿐인지.

물리 결과로 계측 영향을 판정하지 않는다(반복 비재현이 조사 대상이기 때문). 인자: <출력 json> <off 실행> [on 실행...]
"""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from analyze_apply_trace import instrumentation_invariance  # noqa: E402

R1 = ROOT/'artifacts/validation/2026-10-08/scheduled_phase_v1/o1ms_p10_a'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def keys(run):
    with Path(run, 'braking_diagnostics.jsonl').open(encoding='utf-8') as stream:
        return sorted(json.loads(stream.readline()))


def main():
    output, off, *on = sys.argv[1:]
    off = ROOT/off
    result = {'reference_r1': str(R1.relative_to(ROOT)), 'off_run': str(off.relative_to(ROOT)),
              'off_vs_r1': {'runtime_vehicle_identical': sha(off/'runtime_vehicle.sdf') == sha(R1/'runtime_vehicle.sdf'),
                            'runtime_world_identical': sha(off/'runtime_world.sdf') == sha(R1/'runtime_world.sdf'),
                            'diagnostic_keys_identical': keys(off) == keys(R1)},
              'off_vs_on': {str(Path(r)): instrumentation_invariance(off, ROOT/r) for r in on}}
    result['passed'] = all(result['off_vs_r1'].values()) and all(all(v.values()) for v in result['off_vs_on'].values())
    with open(output, 'x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
