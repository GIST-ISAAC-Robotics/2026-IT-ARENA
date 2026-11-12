"""실제 spawn 현재 검사: 저속 재개와 높은 현재 속도 거절의 연결 확인."""
import json,time
from pathlib import Path
from worker_candidate import install,load_current_candidate
from overlap import Lane,evaluate_current
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.follow_trace import equal_output
from arena_autonomy.numeric_kernels import configure
import route_proof
A=Path(__file__).resolve().parent


def main():
    data=json.loads((A.parent/'current_region_policy_r281_v1/profile_inputs.json').read_bytes())
    header=data['header'];header['numeric_execution']='native_v1'
    calls=[data['calls'][i] for i in (79,85,240,517)]
    motion=MotionConfig(**header['motion_config']);cfg=FollowConfig(**header['follow_config'])
    configure('native_v1');original=route_proof.rebase
    route_proof.rebase=load_current_candidate().rebase
    try:expected=[evaluate_current(c['output'],c['submitted'],c['current'],motion,cfg) for c in calls]
    finally:route_proof.rebase=original
    assert any(e['speed_mps']==0 for e in expected) and any(e['speed_mps']>0 for e in expected)
    install();lane=Lane('current',header,capacity=4_000_000,timeout=3.,test_floor_s=0.)
    rows=[]
    try:
        for i,c in enumerate(calls):
            lane.offer(dict(output=c['output'],submitted=c['submitted'],current=c['current']),0,i)
            deadline=time.monotonic()+10.;result=None
            while result is None and time.monotonic()<deadline:
                lane.connection.poll(.1);result=lane.collect()
            assert result is not None and equal_output(expected[i],result['output'])
            rows.append(dict(index=i,reference_s=c['current']['reference_s'],speed_mps=result['output']['speed_mps']))
    finally:closed=lane.close()
    assert not closed['forced'] and closed['exitcode']==0
    report=dict(equal=len(rows),calls=rows,closed=closed,scope='spawn current check; no ROS or drive proof')
    (A/'process_test.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
