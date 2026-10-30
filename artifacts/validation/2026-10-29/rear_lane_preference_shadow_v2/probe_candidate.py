"""현재 직선 선호 후보의 합성 폐루프. 운영 src와 이전 기록을 변경하지 않는다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
PARENT = BASE.parent/'rear_reapproach_v1/probe_rear.py'
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
spec = importlib.util.spec_from_file_location('parent_rear_probe', PARENT)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)

from candidate import StraightLanePreferenceSelector
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.follow_trace import FollowTrace
import arena_autonomy.follow_trace as trace_module

POLICY = 'offline_observed_lane_speed_v2'
DIGEST = hashlib.sha256((BASE/'candidate.py').read_bytes()+b'\0'+(BASE/'parallel_speed.py').read_bytes()).hexdigest()


class CandidateFollow(LocalFollow):
    def __init__(self, config):
        super().__init__(config)
        self.pass_selector = StraightLanePreferenceSelector(config)
        self.reset()


class CandidateTrace(FollowTrace):
    def emit(self, kind, data):
        if kind == 'header':
            data = {**data, 'schema': 901, 'scope': POLICY, 'candidate_sha256': DIGEST}
        return super().emit(kind, data)


def candidate_replay(events):
    def adapted():
        for index, event in enumerate(events):
            if index == 0:
                header = event['data']
                if (header.get('schema') != 901 or header.get('scope') != POLICY
                        or header.get('candidate_sha256') != DIGEST):
                    raise ValueError('wrong offline policy or candidate digest')
                event = {**event, 'data': {**header, 'schema': 1, 'scope': 'g1_decision_stage_only'}}
            yield event
    original = trace_module.LocalFollow
    trace_module.LocalFollow = CandidateFollow
    try:
        report = trace_module.replay_events(adapted())
        report['scope'] = POLICY
        return report
    finally:
        trace_module.LocalFollow = original


if __name__ == '__main__':
    probe.CASES['B_late_reapproach_left'] = dict(sign=1., side_dx=0.,
        side=[(0., 7., .15), (7., 12., .6)], lead=probe.LEAD, duration=12.)
    probe.LocalFollow = CandidateFollow
    probe.FollowTrace = CandidateTrace
    probe.replay_events = candidate_replay
    output = Path(sys.argv[sys.argv.index('--output')+1]).resolve()
    probe.main()
    sources = [Path(__file__), BASE/'candidate.py', BASE/'parallel_speed.py', PARENT]
    with (output/'offline_candidate_manifest.json').open('x', encoding='utf-8') as f:
        json.dump(dict(policy=POLICY, candidate_sha256=DIGEST,
                       sources={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sources},
                       trace_schema=901, generic_replay_rejected=True,
                       scope='synthetic sensor/ideal bicycle/shadow safety only; no ROS integration'),
                  f, ensure_ascii=False, indent=2)
