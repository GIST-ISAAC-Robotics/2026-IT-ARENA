"""실행 중인 A/B 소스를 건드리지 않는 제어 모듈 overlay. 나중 적용 전 원본 해시 대조 필수."""
import ast,hashlib,json
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3];P='src/arena_autonomy/arena_autonomy/'
overlay=A/'overlay/arena_autonomy';overlay.mkdir(parents=True,exist_ok=True)
before={};after={}
def read(name):
    p=ROOT/P/name;b=p.read_bytes();before[P+name]=hashlib.sha256(b).hexdigest()
    return b.decode().replace('\r\n','\n')
def change(text,old,new):
    assert text.count(old)==1,(old[:90],text.count(old))
    return text.replace(old,new,1)
def save(name,text):
    ast.parse(text);(overlay/name).write_bytes(text.encode());after[P+name]=hashlib.sha256(text.encode()).hexdigest()

t=read('bypass_entry_trial.py')
t=change(t,'padding_provider=None, reference_provider=None):','padding_provider=None, reference_provider=None, pursuit_provider=None):')
t=change(t,'    reference_fn = straight_reference if reference_provider is None else reference_provider','    pursuit = pursue_path if pursuit_provider is None else pursuit_provider\n    reference_fn = straight_reference if reference_provider is None else reference_provider')
t=change(t,'            cap, steer, remaining = pursue_path(path, speed, config, age,','            cap, steer, remaining = pursuit(path, speed, config, age,')
t=change(t,'        steer = float(np.clip(steer, previous_steering-3*dt, previous_steering+3*dt))',"        if pursuit_provider is not None:\n            row['preview_unclipped_steering_rad'] = steer\n        steer = float(np.clip(steer, previous_steering-3*dt, previous_steering+3*dt))")
old="""        result=_compare_entries(scene,now,speed,previous_steering,dt,config=self.config,entry=self.entry,
            guide=dict(reference=reference,path=path,offset_m=self.offset,transition_m=self.transition,
                outer_side=self.side,
                stationary_outer_boundary=self.stationary_outer_boundary and self.wall_confirmations>=2),
            padding_provider=self.padding_provider)
"""
new="""        comparison=dict(config=self.config,entry=self.entry,
            guide=dict(reference=reference,path=path,offset_m=self.offset,transition_m=self.transition,
                outer_side=self.side,
                stationary_outer_boundary=self.stationary_outer_boundary and self.wall_confirmations>=2),
            padding_provider=self.padding_provider)
        result=_compare_entries(scene,now,speed,previous_steering,dt,**comparison)
        if getattr(self,'adaptive_preview',False) and self.entry.speed_cap_mps <= .35:
            from functools import partial
            from arena_autonomy.adaptive_entry_preview import compare_shorter_previews,shorter_pursuit
            result=compare_shorter_previews(result,
                lambda lookahead,previous: _compare_entries(scene,now,speed,previous,dt,
                    pursuit_provider=partial(shorter_pursuit,lookahead_m=lookahead),**comparison),
                history=history,now=now,speed=speed,previous=previous_steering,dt=dt)
"""
t=change(t,old,new);save('bypass_entry_trial.py',t)

t=read('bypass_entry_follow.py')
t=change(t,"        detail={**result,'mode':'g2_entry_trial'", "        if chosen is None and result.get('preview_alignment'):\n            steering=result['preview_alignment']['requested_rad']\n        detail={**result,'mode':'g2_entry_trial'")
save('bypass_entry_follow.py',t)

t=read('bypass_handoff_corridor_trial.py')
t=change(t,"        if (self.path is None or primary.get('reason')!='no_entry_command_candidate'", "        if (self.path is None or primary.get('preview_alignment') or primary.get('reason')!='no_entry_command_candidate'")
t=change(t,"                if selected is not None:\n                    return dict(alternative,slow_command_trial=", "                if selected is not None or alternative.get('preview_alignment'):\n                    return dict(alternative,slow_command_trial=")
save('bypass_handoff_corridor_trial.py',t)

t=read('bypass_handoff.py')
t=change(t,'    stopped_entry_centering: bool = False','    stopped_entry_centering: bool = False\n    adaptive_entry_preview: bool = False')
t=change(t,'        if type(self.stopped_entry_centering) is not bool',"        if type(self.adaptive_entry_preview) is not bool or (self.adaptive_entry_preview and self.entry_policy != 'corridor_local_v1'):\n            raise ValueError('adaptive preview requires explicit corridor handoff')\n        if type(self.stopped_entry_centering) is not bool")
t=change(t,'            configure_guide(self,guide_type)','            configure_guide(self,guide_type)\n            self.bypass.guide.adaptive_preview=self.execution.adaptive_entry_preview')
save('bypass_handoff.py',t)

helper=(A/'adaptive_entry_preview.py').read_text(encoding='utf8');save('adaptive_entry_preview.py',helper)
# overlay/__init__.py에서 원 저장소 위치를 검증한다. 현재 파일은 root/artifacts/validation/date/task/overlay/pkg 아래다.
assert (overlay/'__init__.py').resolve().parents[6] == ROOT
(overlay/'__init__.py').write_text("from pathlib import Path\n__path__.append(str(Path(__file__).resolve().parents[6]/'src/arena_autonomy/arena_autonomy'))\n",encoding='utf8')
(A/'overlay_manifest.json').write_text(json.dumps(dict(scope='staged modules only; no installed source or runtime files changed',before=before,after=after),indent=2)+'\n',encoding='utf8')
print('staged',list(after))
