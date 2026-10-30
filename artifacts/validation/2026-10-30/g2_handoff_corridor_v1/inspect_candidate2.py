"""기존 전환기에 새 G2 선택만 연결한 합성 대조. 운영 코드 변경 없음."""
from collections import Counter
from dataclasses import replace
import hashlib, importlib.util, json
from pathlib import Path
import sys, zipfile
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
spec=importlib.util.spec_from_file_location('prior_closed',BASE.parent.parent/'2026-10-29/g2_follow_reserve_v1/probe_closed_loop.py')
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig

from candidate import CorridorHandoff

class Inspect(CorridorHandoff):
    last=None
    def __init__(self,*args,**kw):
        kw['execution']=replace(kw['execution'],bypass=BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold'))
        super().__init__(*args,**kw);self.diagnostics=[];Inspect.last=self
    def command(self,*args,**kw):
        out=super().command(*args,**kw);d=out[2];bg=d.get('bypass') or {};ad=d.get('entry_admission') or {}
        self.diagnostics.append(dict(t=args[0].t_ref_s,speed=args[2],requested=out[:2],reason=d['reason'],state=self.mode,
            bypass_reason=bg.get('reason'),selected=bg.get('selected'),admission=ad,persistence=d.get('persistence'),
            path_retained=self.bypass.guide.path is not None,monitors={k:len(v.rows) for k,v in self.bypass.guide.monitors.items()}))
        return out

def main():
    out=BASE/'candidate2';out.mkdir(exist_ok=False)
    files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),*list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')),
           Path(probe.__file__),ROOT/'scripts/probe_bypass_entry_sequence.py',ROOT/'scripts/bypass_probe_sensors.py']
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name in hashes:z.write(ROOT/name,name)
    probe.BypassHandoffFollow=Inspect
    probe.CASES['center_combined']=(0.,1.2,0.,None,'combined')
    probe.CASES['center_stop_combined']=(0.,1.2,.25,2.,'combined')
    reports=[]
    probe.CASES['center_moving_combined']=(0.,1.2,.25,None,'combined')
    probe.CASES['center_restart_combined']=(0.,1.2,.25,2.,'combined',1.5)
    for case in ('center_combined','center_stop_combined','center_moving_combined','center_restart_combined'):
        for policy,reserve in (('observed_entry_space_v1',True),):
            tag=case+'__'+policy;trace=(out/(tag+'.jsonl')).resolve()
            summary,records=probe.rollout(case,policy,trace,duration=16.,window='trim_before_compare_v1',reserve=reserve,reacquire='known_hold_v1')
            diag=Inspect.last.diagnostics
            replay_controller=Inspect(execution=Inspect.last.execution,allow_simulation_trial=True)
            events=[json.loads(line) for line in trace.open()];motion=MotionConfig(**events[0]['data']['motion_config'])
            count=matched=0
            for event in events[1:]:
                if event['kind']=='reset':replay_controller.reset()
                elif event['kind']=='call':
                    d=event['data'];count+=1
                    matched+=equal_output(freeze(replay_call(replay_controller,d['input'],motion)),d['output'])
            summary['replay']=dict(scope='custom offline candidate replay, not runtime schema5',calls=count,matched=matched)
            summary['mismatch_count']=count-matched
            summary['bypass_reasons']=dict(Counter(d['bypass_reason'] for d in diag))
            summary['admission_rejections']=sum(d['admission'].get('admitted') is False for d in diag)
            summary['max_face_history_rows']=max((max(d['monitors'].values()) for d in diag),default=0)
            (out/(tag+'.records.json')).write_text(json.dumps(records)+'\n')
            (out/(tag+'.diagnostics.json')).write_text(json.dumps(diag)+'\n')
            reports.append(summary);print(json.dumps(summary),flush=True)
    result=dict(scope='sequential sensor synthetic bicycle; no Gazebo or hardware proof',reports=reports,source_sha256=hashes,
        source_unchanged=all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==v for n,v in hashes.items()))
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['source_unchanged'] and all(r['mismatch_count']==0 for r in reports) else 1
if __name__=='__main__':raise SystemExit(main())
