"""범위 제한 stage 수치와 원 입력 보존. 차량/ROS 실행 없이 Windows에서만 계산."""
import hashlib
from collections import Counter
from test_stage import *
from arena_autonomy.follow_trace import freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.race_runtime_policy.evaluator import supported
from arena_autonomy.runtime_dual_delivery import RuntimeDualDelivery


def brief(output):
    if output is None:
        return None
    return dict(speed_mps=output[0], steering_rad=output[1], detail=output[2])


files = ['local_follow.py', 'local_path.py', 'moving_pass.py', 'bypass_wall_fit.py',
    'bypass_entry_trial.py', 'race_runtime_policy/current_route_priority.py',
    'runtime_dual_delivery.py', 'runtime_dual_state.py']
source_hashes = {name: hashlib.sha256((ROOT/'src/arena_autonomy/arena_autonomy'/name).read_bytes()).hexdigest()
                 for name in files}
assert source_hashes['bypass_wall_fit.py'] == 'c95440591c58fcb9ac058ba8d59b45d6419a12ea42a7148abf2c108b4d8c370e'
report = dict(scope='Windows stage saved inputs and synthetic ray intersections; no closed-loop or hardware proof',
    source_sha256=source_hashes, numpy_version=np.__version__, cases=[], saved=[], delivery=[])

for name, kwargs in [('shifted_wall',dict(shift=.16)),
                     ('short_curve',dict(shift=.08,end=.50,curved=True)),
                     ('real_occupied',dict(shift=.16,obstacle=True))]:
    scene, raw = sensor(**kwargs)
    normal = command(LocalFollow(CFG),scene)
    after = command(CurrentWallReacquire(CFG),scene)
    report['cases'].append(dict(name=name, evaluated_geometry=kwargs, baseline=brief(normal),
        candidate=brief(after), returns=len(scene.occupied_endpoints_xy_m),
        statuses=dict(Counter(ray.status for ray in scene.rays)),
        cap_sensitivity={str(cap):brief(current_wall_candidate(scene,0.,'left',0.,.1,CFG,cap_mps=cap))
                         for cap in (.05,.10,.20)}))

owner = active_owner_with_old_wall(old_shift=-.20)
chain = []
for now, shift in ((.7,.16),(.8,.16),(.9,.08),(1.,.08),(1.1,.08),(1.2,.08)):
    scene, raw = sensor(now,shift=shift,closed=True,actor=True)
    adapter = owner.moving_pass
    before = adapter.command(scene,history(),0.,'left',0.,.1,raw_scan=raw)
    after = connect_moving_pass(adapter,before,scene,history(),0.,'left',0.,.1,raw)
    chain.append(dict(reference_s=now,evaluated_wall_shift_m=shift,before=brief(before),after=brief(after),active=adapter.active))
    if not adapter.active:
        break
report['real_adapter_recovery_sequence'] = chain
continued = []
for now in (1.3,1.4,1.5):
    scene, raw = sensor(now,shift=.08,closed=True,actor=1.65)
    adapter = owner.moving_pass
    before = adapter.command(scene,history(),0.,'left',0.,.1,raw_scan=raw)
    after = connect_moving_pass(adapter,before,scene,history(),0.,'left',0.,.1,raw)
    continued.append(dict(reference_s=now,output=brief(after),active=adapter.active))
    if not adapter.active:
        break
report['near_actor_then_more_room_continuation'] = continued
assert continued[-1]['output']['speed_mps'] > .10 and not continued[-1]['active']

owner = active_owner_with_old_wall(old_shift=-.20, actor_x=1.65)
normal_chain = []
for now, shift in ((.7,.16),(.8,.16),(.9,.08),(1.,.08),(1.1,.08),(1.2,.08)):
    scene, raw = sensor(now,shift=shift,closed=True,actor=1.65)
    adapter = owner.moving_pass
    before = adapter.command(scene,history(),0.,'left',0.,.1,raw_scan=raw)
    after = connect_moving_pass(adapter,before,scene,history(),0.,'left',0.,.1,raw)
    normal_chain.append(dict(reference_s=now,before=brief(before),after=brief(after),active=adapter.active))
    if not adapter.active:
        break
report['real_adapter_normal_recovery_sequence'] = normal_chain
assert normal_chain[-1]['after']['speed_mps'] > .10 and not normal_chain[-1]['active']
assert normal_chain[-1]['after']['detail']['handoff_state'] == 'FOLLOW'

# 정상 속도 복귀의 한계도 보존한다. 긴 벽이어도 정면 UNKNOWN의 기존 partial
# hold가 release_count를 다시 지우는 입력에서는 low cap만 계속 나올 수 있다.
owner = active_owner_with_old_wall(old_shift=-.20, actor_x=1.65)
open_chain = []
for now, shift in ((.7,.16),(.8,.16),(.9,.08),(1.,.08),(1.1,.08),(1.2,.08)):
    scene, raw = sensor(now,shift=shift,actor=1.65)
    adapter = owner.moving_pass
    before = adapter.command(scene,history(),0.,'left',0.,.1,raw_scan=raw)
    after = connect_moving_pass(adapter,before,scene,history(),0.,'left',0.,.1,raw)
    open_chain.append(dict(reference_s=now,output=brief(after),active=adapter.active,
        prepared_hold_reason=None if adapter.handoff_follow is None else adapter.handoff_follow.hold_reason,
        prepared_release_count=None if adapter.handoff_follow is None else adapter.handoff_follow.release_count))
report['open_front_existing_partial_limit'] = open_chain

selected = [('continuous_pass_preview_drive_r513_v1',{95,110}),
            ('current_guide_extent_drive_r515_v1',{219})]
saved_for_delivery = None
for folder,sequences in selected:
    path=A.parent/folder/'run1/follow_decisions.jsonl'
    payload=path.read_bytes(); plans={}; expected={}; header=None
    for line in payload.splitlines():
        row=json.loads(line); kind,data=row['kind'],row['data']
        if kind=='runtime_evaluator_header':header=data['header']
        elif kind=='runtime_plan_submitted' and data['sequence'] in sequences:plans[data['sequence']]=data
        elif kind=='runtime_plan_completed' and data['sequence'] in sequences:expected[data['sequence']]=data['output']
    for seq,data in plans.items():
        info,raw,h=restore_call(data['input'],MotionConfig(**header['motion_config']))
        scene=motion_scene(raw,h,info['reference_s'],timing_verified=info['timing_verified'],
            epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
        cfg=FollowConfig(**header['follow_config'])
        result=current_wall_candidate(scene,abs(info['speed_mps']),info['side'],
                                      info['previous_steering_rad'],info['dt_s'],cfg)
        report['saved'].append(dict(source=folder,sequence=seq,sha256=hashlib.sha256(payload).hexdigest(),
            original_recorded_output=expected.get(seq),independent_helper=brief(result),
            source_mutated=False,not_closed_loop=True,measured_speed_mps=info['speed_mps'],
            previous_steering_rad=info['previous_steering_rad'],occupied_points=len(scene.occupied_endpoints_xy_m)))
        saved_for_delivery=data
    assert path.read_bytes()==payload

# 실제 delivery 클래스의 비보관 분기. 자식 계산/ROS 발행은 없는 메모리 fixture다.
class Worker:
    fault=None;ready=None;retained=None
    def __init__(self,plan): self.saved_plan=plan
    def collect(self): return []
    def _usable(self,plan): return supported(plan['output'])
    def discard_plan(self,key): self.saved_plan=None

sample=chain[0]['after']; output=(sample['speed_mps'],sample['steering_rad'],sample['detail'])
data=saved_for_delivery['input']; first=min(data['scan']['measurements']['acquired_at_s'])
assert output[0]>0 and not supported(output)
for label,delay,angle_delta in [('fresh',0.,0.),('source_expired',.31,0.),('previous_changed',0.,.01)]:
    plan=dict(input=data,output=output,generation=1,sequence=1)
    worker=Worker(plan); delivery=RuntimeDualDelivery(worker,lambda:data['reference_s'])
    item=delivery.poll(data['reference_s']+delay,
        previous_steering_rad=data['previous_steering_rad']+angle_delta,
        side=data['side'],motion_epoch=data['epoch'],sensor_epoch=data['scan']['measurements']['source_epoch'])
    report['delivery'].append(dict(case=label,kind=item['kind'],reason=item.get('reason'),
        output_positive=item.get('output',[0])[0]>0,policy='existing_nonretained_plan_boundary',
        memory_fixture_not_ros=True))

report['source_unchanged_during_diagnostic'] = all(
    hashlib.sha256((ROOT/'src/arena_autonomy/arena_autonomy'/name).read_bytes()).hexdigest()==value
    for name,value in source_hashes.items())
(A/'diagnostic.json').write_text(json.dumps(freeze(report),indent=2),encoding='utf-8')
print(json.dumps(dict(cases=[(r['name'],r['baseline']['speed_mps'],r['candidate']['speed_mps']) for r in report['cases']],
    chain=[(r['reference_s'],r['before']['detail']['reason'],r['after']['speed_mps'],r['after']['detail']['reason'],r['active']) for r in chain],
    normal_chain=[(r['reference_s'],r['after']['speed_mps'],r['after']['detail']['reason'],r['active']) for r in normal_chain],
    saved=[(r['source'],r['sequence'],None if r['independent_helper'] is None else r['independent_helper']['speed_mps']) for r in report['saved']],
    delivery=report['delivery'],source_unchanged=report['source_unchanged_during_diagnostic'])))
