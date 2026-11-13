"""0.15m/s 재검사도 가림 회전으로 거절된 SOLO에만0.10m/s를 한 번 검사한다."""
import math

from . import solo_gap as prior
CAP=.10


def rebase(output,submitted,current,scene,history,cfg,*,require_current_region=False, crop_current_scope=False):
    before=prior.rebase(output,submitted,current,scene,history,cfg,require_current_region=require_current_region,
                        crop_current_scope=crop_current_scope)
    if before is None or before['speed_mps']>0:return before
    normal=output[2].get('normal') or {}
    partial=normal.get('partial_observation') or {}
    region=(before.get('current_route_proof') or {}).get('region') or {}
    retry=before.get('current_solo_low_retry') or {}
    speed=current['speed_mps']
    if (not math.isfinite(speed) or not 0<=speed<=.15 or output[0]<=.15
            or output[2].get('handoff_state')!='FOLLOW' or normal.get('selector_mode')!='SOLO'
            or normal.get('boundary_from_memory') is not False
            or partial.get('allowed') is not True or partial.get('reason')!='fully_observed'
            or before.get('reason') not in ('route_current_large_motion_masked_toward_left',
                                           'route_current_large_motion_masked_toward_right')
            or retry.get('applied') is not False or retry.get('cap_mps')!=.15
            or region.get('unobserved')!=0 or region.get('masked_front_samples')!=0
            or region.get('queries',0)<=0):
        return before
    # 실제 측정 속도·현재 점유·전체 이동 영역·조향·시각 모두 원 검사에 그대로 전달한다.
    result=prior.prior.base.rebase((CAP,output[1],output[2]),submitted,current,scene,history,cfg,
                                  require_current_region=True)
    proof={} if result is None else result.get('current_route_proof') or {}
    accepted=(result is not None and 0<result['speed_mps']<=CAP and proof.get('accepted') is True
              and result['steering_rad']==before['steering_rad'] and result['reference_s']==before['reference_s'])
    return {**(result if accepted else before),'current_solo_crawl_retry':dict(applied=accepted,cap_mps=CAP,
        original_reason=before['reason'],previous_cap_mps=.15,measured_speed_mps=speed,
        retry_reason=None if result is None else result['reason'],full_current_check=True,
        unknown_filled=False,source_timestamp_renewed=False,changed_steering=False)}
