"""Gazebo v2와 같은 재접근 일정의 좌우·복합 오차 대조. 합성 전용."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import probe_candidate as candidate


if __name__ == '__main__':
    args, rest = argparse.ArgumentParser(add_help=False), None
    args.add_argument('--policy', choices=('current', 'candidate'), required=True)
    option, rest = args.parse_known_args()
    sys.argv = [sys.argv[0], *rest]
    probe = candidate.probe
    for name, sign in [('B_gazebo_left', 1.), ('B_gazebo_right', -1.)]:
        probe.CASES[name] = dict(sign=sign, side_dx=0.,
            side=[(0., 7., .15), (7., 12., 1.)], lead=probe.LEAD, duration=12.)
    if option.policy == 'candidate':
        probe.LocalFollow = candidate.CandidateFollow
        probe.FollowTrace = candidate.CandidateTrace
        probe.replay_events = candidate.candidate_replay
    output = Path(sys.argv[sys.argv.index('--output')+1]).resolve()
    probe.main()
    sources = [Path(__file__), candidate.BASE/'candidate.py', candidate.BASE/'parallel_speed.py',
               candidate.BASE/'probe_candidate.py', candidate.BASE/'parallel_state.py', candidate.PARENT]
    data = dict(policy=option.policy, candidate_policy=candidate.POLICY if option.policy == 'candidate' else None,
        candidate_sha256=candidate.DIGEST if option.policy == 'candidate' else None,
        scope='synthetic sequential lidar and ideal bicycle; not Gazebo or hardware',
        sources={p.relative_to(candidate.ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (output/'followup_manifest.json').write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
