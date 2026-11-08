"""실행 중인 A/B 소스를 건드리지 않는 제어 모듈 overlay. 나중 적용 전 원본 해시 대조 필수."""
import ast,hashlib,json
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3];P='src/arena_autonomy/arena_autonomy/'
overlay=A/'overlay/arena_autonomy';overlay.mkdir(parents=True,exist_ok=True)
before={};after={};newlines={}
def read(name):
    p=ROOT/P/name;b=p.read_bytes();before[P+name]=hashlib.sha256(b).hexdigest()
    newlines[name]='\r\n' if b'\r\n' in b else '\n'
    return b.decode().replace('\r\n','\n')
def change(text,old,new):
    assert text.count(old)==1,(old[:90],text.count(old))
    return text.replace(old,new,1)
def save(name,text):
    ast.parse(text);data=text.replace('\n',newlines.get(name,'\n')).encode()
    (overlay/name).write_bytes(data);after[P+name]=hashlib.sha256(data).hexdigest()

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
t=change(t,old,new)
t=change(t,"        # 현재 속도는 target이 낮아져도 낮아졌다고 가정하지 않는다.","""        # 짧은 주시거리 후보는 점유/제동에서 이미 0인 명령을 관측 비교 없이 거부한다.
        # 양수 후보의 관측 검사를 생략하지 않으며 기본 비교의 출력은 바꾸지 않는다.
        if pursuit_provider is not None and (clearance <= 0. or target <= 0.):
            row.update(reason='known_occupied_command_hold',
                motion_region=dict(evaluated=False,reason='occupied_command_already_rejected'))
            result['candidates'].append(row)
            continue
        # 현재 속도는 target이 낮아져도 낮아졌다고 가정하지 않는다.""")
t=change(t,"        if command_key not in regions:\n            regions[command_key] = entry_motion_region", """        if pursuit_provider is not None and command_key not in regions:
            from arena_autonomy.adaptive_entry_preview import sampled_region_rejection
            rejected=sampled_region_rejection(scene,speed,target,steer,previous_steering,dt,config,lateral,
                padding if stationary_boundaries or 'outer_boundary_padding' in result else None)
            if rejected is not None:
                regions[command_key]=rejected
        if command_key not in regions:
            regions[command_key] = entry_motion_region""")
save('bypass_entry_trial.py',t)

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
