"""오프라인 상태 후보 비교: 10/29 v4 합성 장면을 런타임 observed_measured_v1과 fresh 확인 기억 후보로 실행한다.

10/30 rear_right_combined_stall_v1/probe_beside.py 복사본에서 후보 교체 대상만 바꿨다.

합성 순차 LiDAR·이상 자전거 모델·그림자 보호층이며 Gazebo/ROS/실물이 아니다. 기존 v4 산출물과 운영 코드는 바꾸지 않는다.
--policy runtime: 현재 런타임 정책 그대로(기준). --policy memory: 평행 후보만 FreshMemoryParallelTrial로 교체.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
V4 = ROOT/'artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4'
sys.path[:0] = [str(BASE), str(V4)]
import probe_candidate as v4  # noqa: E402  (부모 probe_rear 로드·경로 설정 재사용)
from arena_autonomy.local_follow import LocalFollow  # noqa: E402
from arena_autonomy.follow_trace import FollowTrace  # noqa: E402
import arena_autonomy.follow_trace as trace_module  # noqa: E402

POLICY = None
FILES = [Path(__file__), BASE/'candidate_speed.py', BASE/'candidate_state.py']
DIGEST = hashlib.sha256(b'\0'.join(p.read_bytes() for p in FILES)).hexdigest()


class RuntimeFollow(LocalFollow):
    """하네스 헤더(timed)에서 v4와 같은 조합을 명시 정책으로 고른다."""
    def __init__(self, config):
        super().__init__(replace(config, rear_lane_keep_trial=True, rear_lane_keep_policy='observed_measured_v1'))
        if POLICY == 'memory':
            from candidate_state import FreshMemoryParallelTrial
            self.parallel_trial = FreshMemoryParallelTrial(self.config)
        self.reset()


class Trace(FollowTrace):
    def emit(self, kind, data):
        if kind == 'header':
            data = {**data, 'schema': 902, 'scope': f'offline_{POLICY}_1030', 'candidate_sha256': DIGEST}
        return super().emit(kind, data)


def replay(events):
    def adapted():
        for index, event in enumerate(events):
            if index == 0:
                header = event['data']
                if header.get('schema') != 902 or header.get('candidate_sha256') != DIGEST:
                    raise ValueError('wrong offline policy digest')
                event = {**event, 'data': {**header, 'schema': 1, 'scope': 'g1_decision_stage_only'}}
            yield event
    original = trace_module.LocalFollow
    trace_module.LocalFollow = RuntimeFollow
    try:
        return trace_module.replay_events(adapted())
    finally:
        trace_module.LocalFollow = original


if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--policy', choices=('runtime', 'memory'), required=True)
    option, rest = parser.parse_known_args()
    POLICY = option.policy
    sys.argv = [sys.argv[0], *rest]
    probe = v4.probe
    for name, sign in [('B_gazebo_left', 1.), ('B_gazebo_right', -1.)]:
        probe.CASES[name] = dict(sign=sign, side_dx=0., side=[(0., 7., .15), (7., 12., 1.)], lead=probe.LEAD, duration=12.)
    probe.LocalFollow = RuntimeFollow
    probe.FollowTrace = Trace
    probe.replay_events = replay
    output = Path(sys.argv[sys.argv.index('--output')+1]).resolve()
    probe.main()
    sources = [*FILES, v4.PARENT, V4/'probe_candidate.py',
               ROOT/'src/arena_autonomy/arena_autonomy/measured_parallel_speed_trial.py',
               ROOT/'src/arena_autonomy/arena_autonomy/measured_parallel_trial.py',
               ROOT/'src/arena_autonomy/arena_autonomy/measured_lane_preference_trial.py']
    (output/'probe_manifest.json').write_text(json.dumps(dict(
        policy=POLICY, digest=DIGEST, scope='synthetic sequential lidar and ideal bicycle; not Gazebo or hardware',
        sources={p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}),
        indent=2)+'\n', encoding='utf-8')
