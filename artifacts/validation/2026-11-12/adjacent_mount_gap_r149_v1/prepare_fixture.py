"""r148의 실제 거절 질의를 추출한다. 광선·기하를 새로 꾸미지 않는다."""
import hashlib
import json
from pathlib import Path
import sys

import candidate
import arena_autonomy.rear_fringe_trial as rear
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call

A=Path(__file__).resolve().parent
TRACE=A.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    digest=hashlib.sha256(TRACE.read_bytes()).hexdigest()
    rows=[json.loads(line) for line in TRACE.open()]
    candidate.prior.install()
    owner,motion=make_controller(rows[0]['data'])
    captured=[]
    def profile(frame,event,result):
        if event=='return' and frame.f_code is rear._bounded_nonreturn.__code__ and result.get('reason')=='unbounded_nonreturn':
            f=frame.f_locals
            captured.append(dict(queries=f['q'].tolist(),padding=f['pad'].tolist(),result=result))
    for row in rows[1:-1]:
        if row['kind']=='reset':owner.reset();continue
        if row['kind']!='call':continue
        try:
            if row['seq']==605:sys.setprofile(profile)
            replay_call(owner,row['data']['input'],motion)
        finally:sys.setprofile(None)
        if row['seq']==605:
            assert len(captured)==1
            fixture=dict(header=rows[0]['data'],input=row['data']['input'],seq=605,
                         trace_sha256=digest,geometry=captured[0])
            with (A/'fixture.json').open('x') as f:json.dump(fixture,f,separators=(',',':'))
            break
    assert hashlib.sha256(TRACE.read_bytes()).hexdigest()==digest
    print('captured',len(captured))


if __name__=='__main__':main()
