"""최신 기록의 판단 일치와 단계별 비용. PC 고정 입력이며 ROS/Jetson 마감 증명이 아니다."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'),
               str(BASE.parent/'onroad_candidate_v1'), str(BASE.parent/'traffic_residual_v1')]
from compare_saved import preflight, sha
from safety_shadow import ShadowSafety
from arena_autonomy.follow_trace import replay_call, thaw, freeze, comparable_decision, equal_output
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy


def stats(values):
    return dict(count=len(values), **dict(zip(('median','p95','p99','max'),
                map(float,np.percentile(values,[50,95,99,100])))),
                over_20ms=sum(v>20 for v in values)) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    header, expected = preflight(args.trace)
    config = FollowConfig(**header['follow_config'])
    assert config.traffic_recovery_trial and config.partial_observation_policy == 'bounded_local_v4'
    motion = MotionConfig(**header['motion_config'])
    args.output.mkdir(exist_ok=False)
    paths = [p for directory in ('src/arena_autonomy','src/arena_vehicle_interface')
             for p in (ROOT/directory).rglob('*.py') if '__pycache__' not in p.parts]
    paths += [Path(__file__), BASE.parent/'onroad_candidate_v1/compare_saved.py',
              BASE.parent/'traffic_residual_v1/safety_shadow.py']
    sources = {p.relative_to(ROOT).as_posix():sha(p) for p in paths}
    before = sha(args.trace)
    with zipfile.ZipFile(args.output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths: archive.write(p,p.relative_to(ROOT).as_posix())
    (args.output/'parameters.json').write_text(json.dumps(dict(
        trace=str(args.trace),input_sha256=before,source_sha256=sources),indent=2),encoding='utf-8')
    controller = LocalFollow(config); guard = None; calls = 0; previous_source = None
    costs = defaultdict(list); groups = defaultdict(list); reasons = Counter(); worst = []
    with args.trace.open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind']=='reset':
                controller.reset();guard=None;previous_source=None
            if event['kind']!='call':continue
            data=thaw(event['data']['input'])
            if guard is None:guard=ShadowSafety(RecoveryPolicy(True,True,True),config,data['rear_half_angle_deg'])
            started=time.perf_counter_ns();cpu=time.thread_time_ns()
            result=replay_call(controller,data,motion)
            middle=time.perf_counter_ns()
            safe,status=guard.step(data,motion,result[:2])
            end=time.perf_counter_ns();cpu_end=time.thread_time_ns()
            assert equal_output(comparable_decision(freeze(result)),
                comparable_decision(event['data']['output'])), (event['seq'],data['reference_s'])
            assert 0<=safe[0]<=result[0]+1e-12
            planner=(middle-started)/1e6;protection=(end-middle)/1e6;total=(end-started)/1e6
            for name,value in (('planner_ms',planner),('safety_ms',protection),
                               ('total_ms',total),('thread_cpu_ms',(cpu_end-cpu)/1e6)):
                costs[name].append(value)
            reason=str(result[2].get('reason'));reasons[reason]+=1;groups['reason:'+reason].append(total)
            raw=data['scan'];source=(raw['sec'],raw['nanosec'])
            groups['new_scan' if source!=previous_source else 'reused_scan'].append(total)
            previous_source=source
            worst.append(dict(seq=event['seq'],t=data['reference_s'],reason=reason,
                safety_reason=status.get('reason'),planner_ms=planner,safety_ms=protection,total_ms=total))
            worst=sorted(worst,key=lambda row:row['total_ms'],reverse=True)[:20]
            calls+=1
    assert calls==expected and sha(args.trace)==before
    assert all(sha(ROOT/p)==digest for p,digest in sources.items())
    summary=dict(scope='single serialized fixed-input PC replay, planner matches recording; independent safety function cost',
        calls=calls,planner_matches=calls,input_unchanged=True,source_unchanged=True,
        costs={k:stats(v) for k,v in costs.items()},groups={k:stats(v) for k,v in groups.items()},
        reasons=dict(reasons),slowest_calls=worst,online_deadline_verified=False,
        timing_excludes=['file/json/compare','ROS/DDS','vision','actuator','real sensor and host clock mapping'],
        safety_note='stateful shadow on recorded decision inputs; not replay of independent ROS callback timing')
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(calls=calls,planner_matches=calls,costs=summary['costs'])),flush=True)


if __name__=='__main__':main()
