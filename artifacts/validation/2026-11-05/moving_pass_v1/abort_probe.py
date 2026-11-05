"""저장 입력의 추종 대안 거절 이유를 읽는다. 대안을 승인하거나 명령을 변경하지 않는다."""
from collections import Counter
from pathlib import Path
import json
import sys
from arena_autonomy.moving_pass import MovingPassAdapter
from arena_autonomy.follow_trace import replay_events
from review import lines
HERE=Path(__file__).resolve().parent
def main(name):
    evidence=MovingPassAdapter._abort_evidence;attempt=MovingPassAdapter._try_abort
    counts=Counter();alternatives=Counter();examples={}
    def inspect(self,scene,speed,blocked):
        out=evidence(self,scene,speed,blocked)
        key=(out is not None,len(self.blocked_sources)>=3,speed<=.15,self.tracker.current is not None,self.guide.phase)
        counts[str(key)]+=1
        return out
    def observe(self,*args):
        call=self.owner._base_command
        def record(normal):
            n=normal[2].get('normal',{})
            key=(normal[0]>0,self.owner.mode,n.get('fresh_boundary_accepted'),n.get('boundary_from_memory'),
                 n.get('reason'),n.get('selector_choice'),str(n.get('selector_sides',{}).get('right')))
            key=str(key);alternatives[key]+=1
            if key not in examples:
                examples[key]=dict(speed=normal[0], reason=normal[2].get('reason'),normal=n,abort=args[7])
            return normal
        def observed(*a,**k):return record(call(*a,**k))
        if len(args)>8 and args[8] is not None:record(args[8])
        self.owner._base_command=observed
        try:return attempt(self,*args)
        finally:self.owner._base_command=call
    MovingPassAdapter._abort_evidence=inspect;MovingPassAdapter._try_abort=observe
    try:replay_events(lines(HERE/'closed_loop_v3/run1/follow_decisions.jsonl'))
    finally:MovingPassAdapter._abort_evidence=evidence;MovingPassAdapter._try_abort=attempt
    with (HERE/(name+'.json')).open('x',encoding='utf-8') as f:
        json.dump(dict(counts=counts,alternatives=alternatives,examples=examples),f,indent=1);f.write('\n')
    print(counts);print(alternatives.most_common(4))
if __name__=='__main__':main(sys.argv[1] if len(sys.argv)>1 else 'abort_probe1')
