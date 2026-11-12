"""저장 교통 입력의 후보 계산 비용. 장치·ROS·차량 출력에 접근하지 않는다."""
from collections import Counter
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
             str(A.parent/'adjacent_mount_gap_r149_v1')]
import candidate
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, comparable_decision, freeze, equal_output, controller_mode
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.scene_motion import motion_scene


def timed(owner, inputs, motion):
    begin=time.perf_counter_ns()
    data,scan,history=restore_call(inputs,motion)
    restored=time.perf_counter_ns();projected=restored
    try:
        scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
            epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        projected=time.perf_counter_ns()
        out=owner.command(scene,history,data['speed_mps'],data['side'],data['previous_steering_rad'],
            data['dt_s'],**({'raw_scan':scan} if owner.config.partial_observation_policy!='strict' else {}))
    except (MotionUnavailable,ValueError) as error:
        owner.reset()
        out=0.,0.,dict(reason='follow_input_stop',behavior='HOLD',input_error=str(error),mode=controller_mode(owner))
    end=time.perf_counter_ns()
    return out,dict(restore_ms=(restored-begin)/1e6,projection_ms=(projected-restored)/1e6,
        decision_ms=(end-projected)/1e6,total_ms=(end-begin)/1e6)


def dist(values):
    return dict(count=len(values),p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)),
        maximum=max(values),over20=sum(v>20 for v in values),over100=sum(v>100 for v in values)) if values else dict(count=0)


def different_paths(a,b,path=''):
    if equal_output(a,b):return []
    if isinstance(a,dict) and isinstance(b,dict):
        return [v for k in sorted(a.keys()|b.keys()) for v in
            ([path+'/'+k] if k not in a or k not in b else different_paths(a[k],b[k],path+'/'+k))]
    if isinstance(a,list) and isinstance(b,list) and len(a)==len(b):
        return [v for i,(x,y) in enumerate(zip(a,b)) for v in different_paths(x,y,path+'/'+str(i))]
    return [path]


def main():
    p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');p.add_argument('--label',required=True)
    args=p.parse_args()
    manifest_path=A/'bundle_manifest.json'
    manifest=json.loads(manifest_path.read_bytes()) if not args.prepare else None
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    unchanged=lambda:manifest is None or all(sha(ROOT/name)==m['sha256'] for name,m in manifest['files'].items())
    assert unchanged()
    trace=A.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl'
    events=[json.loads(line) for line in trace.open()]
    records=[];summary={}
    for mode,install in [('baseline',candidate.prior.prior.install),('candidate',candidate.install)]:
        install();owner,motion=make_controller(events[0]['data'])
        expected_path=A/(mode+'_expected.jsonl')
        expected={} if args.prepare else {r['seq']:r['output'] for r in map(json.loads,expected_path.open())}
        saved=[];rows=[];mismatches=[];command_mismatches=[];fields=Counter();outputs=[]
        for row in events[1:-1]:
            if row['kind']=='reset':owner.reset()
            if row['kind']!='call':continue
            output,cost=timed(owner,row['data']['input'],motion)
            output=comparable_decision(freeze(output))
            reference=comparable_decision(row['data']['output']) if mode=='baseline' else expected.get(row['seq'])
            if reference is not None and not equal_output(output,reference):
                mismatches.append(row['seq']);fields.update(different_paths(output,reference))
                if not equal_output(output[:2],reference[:2]):command_mismatches.append(row['seq'])
            outputs.append(dict(seq=row['seq'],output=output))
            if args.prepare:saved.append(dict(seq=row['seq'],output=output))
            normal=output[2].get('normal') or {}
            retry=normal.get('selector_horizon_recheck') or {}
            rows.append(dict(seq=row['seq'],mode=mode,positive=output[0]>0,attempted=bool(retry),
                accepted=retry.get('applied',False),**cost))
        if args.prepare:
            with expected_path.open('x') as f:
                for row in saved:f.write(json.dumps(row)+'\n')
        summary[mode]=dict(calls=len(rows),mismatches=mismatches,command_mismatches=command_mismatches,different_fields=dict(fields),
            groups={name:{stage:dist([r[stage] for r in subset]) for stage in
                ('restore_ms','projection_ms','decision_ms','total_ms')} for name,subset in
                [('all',rows),('positive',[r for r in rows if r['positive']]),
                 ('retry',[r for r in rows if r['attempted']]),('retry_accepted',[r for r in rows if r['accepted']]) ]})
        records.extend(rows)
        with (A/(args.label+'_'+mode+'_outputs.jsonl')).open('x') as f:
            for row in outputs:f.write(json.dumps(row)+'\n')
    result=dict(results=summary,source_unchanged=unchanged(),trace_sha256=sha(trace),
        scope='One CPU pass per mode; full output differences retained, command comparison separate. Strict equivalence may fail. Restore included; parse/comparison excluded. Not closed-loop/ROS.')
    assert result['source_unchanged']
    with (A/(args.label+'_costs.jsonl')).open('x') as f:
        for row in records:f.write(json.dumps(row)+'\n')
    with (A/(args.label+'_summary.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result))


if __name__=='__main__':main()
