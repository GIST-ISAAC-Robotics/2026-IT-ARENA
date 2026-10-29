"""기존 G1 교통 1랩 입력에서 새 우회 진입 제안/사전 감속만 관찰한다.

제안은 적용하지 않는다. 제안 직후 G2/반환 누적만 비우고 기존 G1 이력은
계속 유지하므로 실제 G1/G2 폐루프 재생이나 새 완주 결과가 아니다.
"""
from collections import Counter
from dataclasses import asdict
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,comparable_decision,freeze,equal_output

INPUT=ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1/run4/follow_decisions.jsonl'


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',choices=('run1','run2','run3'),default='run1')
    args=parser.parse_args()
    out=BASE/args.output;out.mkdir(exist_ok=False)
    sources=[p for p in (ROOT/'src').rglob('*.py') if '__pycache__' not in p.parts]+[Path(__file__)]
    hashes={p.relative_to(ROOT).as_posix():digest(p) for p in sources}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sources:archive.write(p,p.relative_to(ROOT))
    counts=Counter();proposals=[];capped=[];normal_mismatch=[];cost=[];last_seq=-1;footer=None
    with INPUT.open(encoding='utf-8') as source,(out/'rows.jsonl').open('x',encoding='utf-8') as detail:
        for line in source:
            event=json.loads(line)
            if footer is not None or event['seq']!=last_seq+1:raise ValueError('input gap/trailing event')
            last_seq=event['seq'];kind,data=event['kind'],event['data']
            if kind=='header':
                if last_seq!=0 or data['schema']!=1:raise ValueError('expected preserved G1 input')
                ctl=BypassHandoffFollow(FollowConfig(**data['follow_config']),allow_simulation_trial=True)
                motion=MotionConfig(**data['motion_config']);header=data
            elif kind=='reset':ctl.reset();counts['reset']+=1
            elif kind=='footer':footer=data
            elif kind=='call':
                began=time.perf_counter();actual=freeze(replay_call(ctl,data['input'],motion))
                cost.append((time.perf_counter()-began)*1000);counts['calls']+=1
                d=actual[2];normal=d.get('normal');expected=data['output'];at=data['input']['reference_s']
                if normal is None:
                    counts['normal_unavailable']+=1
                elif equal_output(comparable_decision([expected[0],expected[1],normal]),comparable_decision(expected)):
                    counts['normal_detail_matched']+=1
                else:
                    counts['normal_detail_mismatched']+=1
                    if len(normal_mismatch)<5:normal_mismatch.append(dict(time_s=at,actual=normal,expected=expected[2]))
                counts[d.get('reason','missing_reason')]+=1
                if d.get('reason')=='handoff_entry':
                    proposals.append(dict(reference_s=at,selected=d.get('bypass',{}).get('selected'),
                        persistence=d.get('persistence'),normal_reason=expected[2].get('reason'),
                        normal_speed_mps=expected[0],measured_speed_mps=data['input']['speed_mps']))
                elif d.get('reason')=='handoff_follow' and actual[0]<expected[0]-1e-8:
                    capped.append(dict(reference_s=at,before=expected[0],after=actual[0],
                        normal_reason=expected[2].get('reason'),patch=d.get('persistence')))
                detail.write(json.dumps(dict(seq=event['seq'],reference_s=at,requested=actual[:2],
                    expected=expected[:2],reason=d.get('reason'),persistence=d.get('persistence'),
                    proposal=d.get('bypass',{}).get('selected'),processing_ms=cost[-1]))+'\n')
                if d.get('reason')=='handoff_entry':
                    # 제안된 경로를 실제로 주행하지 않은 저장 입력이다. 이후 G2
                    # 상태를 이 궤적에 계속 이송해 가짜 복귀/성공을 만들지 않는다.
                    ctl.mode='FOLLOW';ctl.bypass.reset();ctl.monitor.reset();ctl.reacquire=0
            else:raise ValueError('unexpected event')
    if footer is None or not counts['calls']:raise ValueError('incomplete input')
    if any(footer.get(k)!=0 for k in ('capture_errors','failed','dropped','pending')) or footer.get('worker_alive') is not False:
        raise ValueError('invalid input footer')
    if footer.get('written')!=last_seq or footer.get('accepted')!=last_seq:raise ValueError('footer count mismatch')
    ordered=sorted(cost)
    report=dict(input=str(INPUT.relative_to(ROOT)),input_sha256=digest(INPUT),counts=dict(counts),
        execution_config=asdict(ctl.execution),follow_config=header['follow_config'],
        source_sha256=hashes,source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()),
        proposals=proposals,capped_calls=len(capped),first_capped=capped[:15],
        normal_detail_mismatch_examples=normal_mismatch,
        processing_ms=dict(median=ordered[len(ordered)//2],p95=ordered[int(.95*(len(ordered)-1))],max=max(cost)),
        scope='Admission/prepare-cap shadow on stored G1 trajectory. G2 proposal is discarded after recording; no output actuation, new closed loop, free-space proof or lap completion.')
    with (out/'report.json').open('x',encoding='utf-8') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:report[k] for k in ('counts','capped_calls','processing_ms','source_unchanged')},indent=2))
    print('proposals',len(proposals))


if __name__=='__main__':main()
