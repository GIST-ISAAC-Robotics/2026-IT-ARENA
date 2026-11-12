"""동일 저장 입력의 순수 후보와 실제 spawn 계획 프로세스 출력 일치."""
import json,time
from pathlib import Path
from worker_candidate import install,load_candidate
from overlap import Lane
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import freeze,replay_call,equal_output
from arena_autonomy.numeric_kernels import configure

A=Path(__file__).resolve().parent


def main():
    with (A.parent/'parallel_ros_runtime_r274_v1/two_actor_run1/run1/follow_decisions.jsonl').open() as f:
        header=json.loads(next(f))['data']
        calls=[e['data']['input'] for line in f if (e:=json.loads(line))['kind']=='call'
               and 54.<=e['data']['input']['reference_s']<54.6]
    assert len(calls)>=3
    configure('native_v1');header['numeric_execution']='native_v1'
    patch=load_candidate();patch.install();install()
    follow,motion=make_controller(header)
    expected=[freeze(replay_call(follow,c,motion)) for c in calls]
    patch.uninstall()
    assert expected[-1][0]>0 and expected[0][2]['normal']['single_wall_reacquire']['side']=='right'
    lane=Lane('plan',header,capacity=4_000_000,timeout=3.,test_floor_s=0.)
    rows=[]
    try:
        for i,c in enumerate(calls):
            lane.offer(c,0,i,None)
            deadline=time.monotonic()+10.
            result=None
            while result is None and time.monotonic()<deadline:
                lane.connection.poll(.1);result=lane.collect()
            assert result is not None
            assert equal_output(expected[i],result['output'])
            rows.append(dict(sequence=i,reference_s=c['reference_s'],speed_mps=result['output'][0]))
    finally:closed=lane.close()
    assert not closed['forced'] and closed['exitcode']==0
    record=dict(equal=len(rows),calls=rows,closed=closed,scope='spawn planner only; no ROS/drive proof')
    (A/'process_test.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record))


if __name__=='__main__':main()
