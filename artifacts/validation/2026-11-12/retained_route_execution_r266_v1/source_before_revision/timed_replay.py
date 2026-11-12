"""저장 센서 시간표를 실제 프로세스로 재생한다. 명령은 파일만 기록한다.

센서/차체 운동은 기록된 궤적이며 이번 출력에 반응하지 않는다. 따라서 명령
갱신 비용/순서 시험이지 주행, 충돌, 정지 거리 증명이 아니다.
"""
import argparse
import bisect
import copy
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from arena_autonomy.decision_delivery import source_identity
from route_delivery import RouteProcess

A=Path(__file__).resolve().parent; ROOT=A.parents[3]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=('new_only','retained'),required=True)
    parser.add_argument('--duration',type=float,default=15.)
    parser.add_argument('--plan-ms',type=float,default=147.)
    parser.add_argument('--current-ms',type=float,default=34.)
    parser.add_argument('--out',required=True)
    args=parser.parse_args();target=Path(args.out);target.mkdir(exist_ok=False)
    trace=ROOT/'artifacts/validation/2026-11-12/traffic_solo_gap_runtime_r225_v1/two_actor_run1/run1/follow_decisions.jsonl'
    events=[json.loads(line) for line in trace.open()]
    header=events[0]['data'];header['numeric_execution']='native_v1'
    calls=[e['data']['input'] for e in events if e['kind']=='call']
    refs=[c['reference_s'] for c in calls]
    begin=refs[0]
    worker=RouteProcess(header,work_timeout=3.,test_floor_s=args.plan_ms/1000,
                        current_test_floor_s=args.current_ms/1000)
    published=[];rows=[];discards={};last_source=None;previous=calls[0]['previous_steering_rad']
    last_application=begin-.1;submitted=0;started=time.monotonic();deadline=started+args.duration
    def record(kind,**kwargs):rows.append(dict(kind=kind,wall_s=time.monotonic()-started,**kwargs))
    try:
        while time.monotonic()<deadline:
            now=begin+time.monotonic()-started
            index=bisect.bisect_right(refs,now)-1
            if index<0:continue
            current=copy.deepcopy(calls[index])
            current['previous_steering_rad']=previous
            # 기록의 시각/운동/거리/이력은 그대로. 후보의 조향 예산은 100ms다.
            current['dt_s']=.1
            for row in worker.collect():
                record('completed',stage=row['stage'],sequence=row['sequence'],
                       elapsed_ms=(row['completed_wall_s']-row['began_wall_s'])*1000,
                       reason=row.get('discard_reason'),route_check=row.get('route_check'))
                if row.get('discard_reason'):
                    reason=row['discard_reason'];discards[reason]=discards.get(reason,0)+1
            if worker.fault:raise RuntimeError(worker.fault)
            state=dict(now_s=now,previous_steering_rad=previous,side=current['side'],
                       motion_epoch=current['epoch'],sensor_epoch=source_identity(current)[0])
            pending=worker.ready
            prepared=worker.prepared(**state)
            if prepared and prepared['kind']=='candidate':
                result=prepared['result'];out=result['output']
                published.append(dict(now_s=now,reference_s=out['reference_s'],speed=out['speed_mps'],
                    steering=out['steering_rad'],retained=result['route_check']['retained'],
                    original_path_source_s=result['plan']['output'][2]['normal']['path_source_s'],
                    current_source_first_s=source_identity(result['input'])[1],
                    proof=out.get('current_route_proof')))
                worker.commit_published((result['generation'],result['sequence']),**state)
                previous=out['steering_rad'];last_application=now
            elif prepared and prepared.get('reason')!='positive_application_interval':
                reason=prepared['reason'];discards[reason]=discards.get(reason,0)+1
                record('not_applied',reason=reason,now_s=now,current_ref=pending['input']['reference_s'],
                    current_first=source_identity(pending['input'])[1],old_ref=pending['plan']['input']['reference_s'])
            saved=worker.saved_plan
            if saved and not worker._usable(saved):
                # ROS 발행 없이 새 상태/0 결과를 명령 경계로만 기록하고 옛 경로를 비운다.
                out=saved['output']
                record('non_recheck_plan',speed=out[0],state=out[2].get('handoff_state'))
                worker.discard_plan((saved['generation'],saved['sequence']))
                if out[0]<=0:
                    worker.retained=None
                    published.append(dict(now_s=now,reference_s=saved['input']['reference_s'],
                                          speed=0.,steering=previous,retained=False,unchecked_state_boundary=True))
                    last_application=now
            source=source_identity(current)
            if args.mode=='retained' or worker.saved_plan is not None:
                worker.offer_next_current(current)
            # 10Hz 기록 입력을 처음 받은 25ms 안에만 계획 제출. 옛 입력 시간을 바꾸지 않는다.
            if source!=last_source and 0<=now-current['reference_s']<=.025:
                if worker.offer_plan(current):last_source=source;submitted+=1
            time.sleep(.001)
    finally:
        worker.invalidate('end_of_bounded_replay')
        cleanup=worker.close()
    gaps=[];ages=[]
    for old,new in zip(published,published[1:]):
        if old['speed']>0:
            gaps.append(new['now_s']-old['now_s']);ages.append(new['now_s']-old['reference_s'])
    def stats(values):
        return None if not values else dict(count=len(values),p50_ms=float(np.percentile(values,50)*1000),
            p95_ms=float(np.percentile(values,95)*1000),max_ms=max(values)*1000)
    report=dict(mode=args.mode,duration_s=args.duration,plan_floor_ms=args.plan_ms,current_floor_ms=args.current_ms,
        submitted=submitted,published=len(published),positive=sum(r['speed']>0 for r in published),
        retained_positive=sum(r['speed']>0 and r['retained'] for r in published),
        gaps=stats(gaps),previous_command_age_before_next=stats(ages),expired_positive_intervals=sum(x>=.2 for x in ages),
        discards=discards,cleanup=cleanup,trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        scope='saved trajectory; actual processes; no ROS output, independent guard, physical response or driving proof')
    (target/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (target/'answers.json').write_text(json.dumps(published)+'\n')
    (target/'events.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    print(json.dumps(report),flush=True)
    assert all(not r['forced'] and r['exitcode']==0 for r in cleanup)


if __name__=='__main__':main()
