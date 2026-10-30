"""기존 개방형 폐루프를 같은 조건에서 비교한다. 후보 기록은 비운영 schema905다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import patch
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(BASE), str(ROOT/'src/arena_autonomy'),
               str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
from candidate import MovingReserveCandidate
import arena_autonomy.bypass_handoff as runtime
from arena_autonomy.follow_trace import FollowTrace, replay_events

SCHEMA = 905
SCOPE = 'offline_g2_moving_reserve_v1'
spec = importlib.util.spec_from_file_location('openframe_previous',
    ROOT/'artifacts/validation/2026-10-29/g2_openframe_admission_v1/probe_shape.py')
shape = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shape)


def candidate_hash():
    return hashlib.sha256((BASE/'candidate.py').read_bytes()).hexdigest()


class CandidateTrace(FollowTrace):
    def emit(self, kind, data):
        if kind == 'header':
            if data.get('schema') != 5:
                raise ValueError('expected handoff base header')
            data = dict(data, schema=SCHEMA, scope=SCOPE,
                        offline_candidate_sha256=candidate_hash())
        return super().emit(kind, data)


def replay_candidate(events):
    def adapted():
        first = True
        for event in events:
            if first:
                d = event['data']
                if (event.get('kind') != 'header' or d.get('schema') != SCHEMA
                        or d.get('scope') != SCOPE
                        or d.get('offline_candidate_sha256') != candidate_hash()):
                    raise ValueError('unmatched offline candidate record')
                d = dict(d, schema=5, scope='g1_g2_handoff_decision_stage_only')
                d.pop('offline_candidate_sha256')
                event = dict(event, data=d)
                first = False
            yield event
    # 원시 기록은 고치지 않는다. 검증된 후보 기록만 이 프로세스의 후보로 재생한다.
    with patch.object(runtime, 'BypassHandoffFollow', MovingReserveCandidate):
        return replay_events(adapted())


def main():
    pos = sys.argv.index('--variant')
    variant = sys.argv[pos+1]
    if variant not in ('existing', 'motion_v1'):
        raise ValueError('unknown variant')
    del sys.argv[pos:pos+2]
    out = BASE/sys.argv[sys.argv.index('--output')+1]
    own = {p.relative_to(ROOT).as_posix(): p.read_bytes()
           for p in (Path(__file__), BASE/'candidate.py')}
    if variant == 'motion_v1':
        shape.common.BypassHandoffFollow = MovingReserveCandidate
        shape.common.FollowTrace = CandidateTrace
        shape.common.replay_events = replay_candidate
    shape.BASE = BASE
    shape.main()
    report = json.loads((out/'report.json').read_text())
    report['offline_variant'] = variant
    if variant == 'motion_v1':
        for case in report['cases']:
            case['offline_schema905_replay'] = case.pop('schema5_replay')
    with zipfile.ZipFile(out/'sources.zip', 'a', zipfile.ZIP_DEFLATED) as archive:
        for name, content in own.items():
            archive.writestr(name, content)
    report['candidate_source_sha256'] = {n: hashlib.sha256(v).hexdigest() for n,v in own.items()}
    report['source_unchanged'] &= all((ROOT/n).read_bytes() == v for n,v in own.items())
    (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    if not report['source_unchanged']:
        raise RuntimeError('candidate source changed')


if __name__ == '__main__':
    main()
