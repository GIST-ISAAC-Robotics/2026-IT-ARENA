"""r358의 시간 재생기/기록은 유지하고 공통 backend의 제출 선택만 비교한다."""
import importlib.util
import json
from pathlib import Path
import sys
from candidate import ScheduledProcess

A=Path(__file__).resolve().parent
sys.path.insert(0,str(A.parent/'runtime_factory_r365_v1'))
sys.path.insert(0,str(A.parent/'shared_ray_timed_r358_v1'))
from factory_reference import evaluator_factory, policy_support
spec=importlib.util.spec_from_file_location('r369_original_timed', A.parent/'shared_ray_timed_r358_v1/timed_replay.py')
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)


class ReplayProcess(ScheduledProcess):
    def __init__(self,*args,**kwargs):
        self.completed_costs={}
        super().__init__(*args,**kwargs)

    def collect(self):
        rows=super().collect()
        for row in rows:
            plan=row if row['stage']=='plan' else row.get('plan',{})
            output=plan.get('output') or [0.,0.,{}]
            state=(output[2].get('normal') or {}).get('selector_mode','none')
            key=row['stage']+':'+str(state)
            elapsed=(row['completed_wall_s']-row['began_wall_s'])*1000
            self.completed_costs.setdefault(key,[]).append(elapsed)
        return rows

    def close(self):
        # r358 재생기는 lane 목록을 받는다. 공통 backend의 실제 cleanup 원문은 그대로 둔다.
        return super().close()['lanes']

def main():
    option=sys.argv.pop(1)
    if option not in ('baseline','skip_released'):raise ValueError('explicit scheduling mode required')
    created=[]
    def factory(header,*,test_floor_s,current_test_floor_s,**kwargs):
        if test_floor_s!=0 or current_test_floor_s!=0 or header.get('r358_classification')!='shared':
            raise ValueError('no timing floor or alternate numeric policy in this comparison')
        worker=ReplayProcess(dict(header,r365_policy='r353'),supported=policy_support,
            evaluator_factory=evaluator_factory,avoid_superseded_retained=(option=='skip_released'),**kwargs)
        created.append(worker)
        return worker
    original.CurrentDeadlineProcess=factory
    output=Path(sys.argv[sys.argv.index('--out')+1])
    try:
        original.main()
    finally:
        if created:
            (output/'scheduling.json').write_text(json.dumps(dict(mode=option,
                skipped=created[0].skipped_retained_submissions,cleanup=created[0].cleanup,
                completed_costs_ms=created[0].completed_costs),indent=2)+'\n')

if __name__=='__main__':main()
