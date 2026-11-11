"""명시 launch-prefix 시험에서만 쓰는 현재 FOLLOW 재계산 연결."""
import time

from rebase import rebase
from arena_autonomy.follow_trace import snapshot_call,freeze
from arena_autonomy.g1_g2_node import status_detail
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.scene_motion import motion_scene


def command(node,points):
    node.async_inputs_ready=True
    item=node.decision_delivery.poll(node.now_s(),previous_steering_rad=node.last_steering,
        side=node.side,motion_epoch=node.motion.counts['resets'],
        sensor_epoch=getattr(node.scan,'source_epoch',0))
    if item is None:return None
    if item['kind']=='fault':
        return 0.,node.last_steering,dict(reason='decision_process_fault',input_error=item['reason'])
    evidence=dict(input=item['input'],output=item['result']['output'],
        delivery={k:v for k,v in item.items() if k not in ('input','result','output')},
        compute={k:v for k,v in item['result'].items() if k!='output'})
    if item['kind']=='discarded':
        node.follow_trace.emit('call',evidence)
        node.follow.reset();node.follow_trace.emit('reset',dict(reason=item['reason']))
        return None
    output=item['output'];reference=item['reference_s']
    normal=output[2].get('normal') or {}
    supported=(output[0]>0 and output[2].get('handoff_state')=='FOLLOW'
        and normal.get('reason')=='local_follow' and normal.get('selector_mode')=='SOLO'
        and normal.get('selector_choice')=='baseline')
    current_result=None
    if supported:
        began=time.perf_counter()
        now=node.motion.last_meta['reference_stamp_s']
        speed=node.motion.history.wheels.values[-1] if node.motion.history.wheels.values else 0.
        current=snapshot_call(node.scan,node.motion.history,now,node.motion.counts['resets'],
            speed,node.side,node.last_steering,node.decision_output_dt_s,
            node.motion.verified,node.rear_blind_half_angle)
        try:
            scene=motion_scene(node.scan,node.motion.history,now,timing_verified=node.motion.verified,
                epoch=node.motion.counts['resets'],rear_half_angle_deg=node.rear_blind_half_angle)
            current_result=rebase(output,item['input'],current,scene,node.motion.history,node.follow.config)
        except (MotionUnavailable,ValueError) as error:
            current_result=dict(speed_mps=0.,steering_rad=node.last_steering,
                reference_s=now,reason='rebase_input_stop',input_error=str(error),
                independent_guard_required=True,runtime_authorized=False)
        if current_result is None:raise RuntimeError('supported rebase returned no result')
        detail={**output[2],'current_rebase':{k:v for k,v in current_result.items() if k!='path_points'}}
        if current_result['speed_mps']<=0:detail['reason']=current_result['reason']
        output=(current_result['speed_mps'],current_result['steering_rad'],detail)
        reference=current_result['reference_s']
        evidence['current_rebase']=dict(input=current,output=freeze(current_result),
            computation_wall_ms=1000*(time.perf_counter()-began))
    node.follow_trace.emit('call',evidence)
    node.decision_command_stamp_s=reference
    target,steering,detail=node.prepared_output(output)
    compact=status_detail(detail)
    if current_result is not None:compact['current_rebase']=detail.get('current_rebase',{})
    return target,steering,{**compact,'decision_delivery':{
        **node.decision_delivery.counts,'source_age_s':item['source_age_s'],
        'reference_s':item['reference_s'],'command_reference_s':reference}}
