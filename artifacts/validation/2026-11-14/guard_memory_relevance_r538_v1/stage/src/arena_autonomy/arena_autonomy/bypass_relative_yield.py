"""명시 시뮬레이션 후보. 막힌 복귀 띠 옆에서 제한 감속을 제안한다.

차량 신원/속도를 확정하거나 점유를 없애지 않는다. 유효한 현재 면 관측 아래
최대5초, 실제 복귀 단계 변경까지 감속만 유지한다. 기본값은 off다.
"""
from collections import deque
from dataclasses import replace
import math


def finite(v):
    return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)


class _RelativeYieldWindow:
    window_s=.8
    minimum_scans=9
    scan_gap_s=.25
    maximum_rearward_displacement_m=.10
    trial_speed_cap_mps=.20

    def __init__(self):self.reset()

    def reset(self):
        self.rows=deque();self.key=None
        self.detail=dict(suggest_yield=False,reason='no_current_evidence',control_authorized=False,
                         moving_object_identity_proven=False)

    def update(self,row):
        if not isinstance(row,dict):
            self.reset();return self.detail
        required=('source','now','heading','speed','normal','along_low','along_high')
        if (not all(finite(row.get(k)) for k in required) or row.get('phase')!='entry'
                or row.get('side') not in ('left','right') or row.get('method')!='parallel_face_median'
                or not row.get('blocked_by_current_returns') or row.get('band_supported')
                or row.get('wall_confirmations',0)<2
                or not 0<=row['speed']<=1 or not 0<=row['now']-row['source']<=self.scan_gap_s
                or abs(row['heading'])>.08 or not .10<=row['along_high']-row['along_low']<=.50
                or row['along_high']<-.10 or row['along_low']>.50):
            self.reset();return self.detail
        key=(row.get('epoch'),row['source'])
        if key==self.key:return self.detail
        if self.key is not None and (key[0]!=self.key[0] or key[1]<=self.key[1]
                or key[1]-self.key[1]>self.scan_gap_s or row['side']!=self.rows[-1]['side']):
            self.reset()
        if self.rows:
            old=self.rows[-1]
            if (abs(row['normal']-old['normal'])>.06
                    or min(row['along_high'],old['along_high'])-max(row['along_low'],old['along_low'])<-.06):
                self.reset()
        self.key=key;self.rows.append(dict(row))
        while len(self.rows)>self.minimum_scans and row['source']-self.rows[1]['source']>=self.window_s-1e-9:
            self.rows.popleft()
        span=row['source']-self.rows[0]['source']
        centers=[(r['along_low']+r['along_high'])/2 for r in self.rows]
        widths=[r['along_high']-r['along_low'] for r in self.rows]
        normals=[r['normal'] for r in self.rows];headings=[r['heading'] for r in self.rows]
        displacement=centers[-1]-centers[0]
        stable=(max(widths)-min(widths)<=.06 and max(normals)-min(normals)<=.06
                and max(headings)-min(headings)<=.04)
        enough=len(self.rows)>=self.minimum_scans and span>=self.window_s-1e-9
        suggest=bool(enough and stable and displacement>=-self.maximum_rearward_displacement_m)
        self.detail=dict(suggest_yield=suggest,reason='persistent_blocked_return_band' if suggest else 'window_or_relative_clearance',
            scans=len(self.rows),span_s=span,relative_center_displacement_m=displacement,
            stable_face=stable,trial_speed_cap_mps=self.trial_speed_cap_mps if suggest else None,
            control_authorized=False,moving_object_identity_proven=False,
            unknown_filled=False,uses_motor_displacement_to_classify_object=False)
        return self.detail



class BoundedRelativeYield(_RelativeYieldWindow):
    maximum_continuation_s=5.
    maximum_observed_front_m=1.2

    def reset(self):
        super().reset()
        self.yield_since=None
        self.anchor=None

    def stop_advice(self,reason):
        self.reset()
        self.detail['reason']=reason
        return self.detail

    def update(self,row):
        if self.yield_since is None:
            result=super().update(row)
            if result.get('suggest_yield'):
                self.yield_since=row['source'];self.anchor=dict(row)
            return result
        return self.continue_observation(row)

    def continue_observation(self,row,*,motion_pause_gap=False):
        if not isinstance(row,dict):
            return self.stop_advice('continuation_observation_missing')
        required=('source','now','heading','speed','normal','along_low','along_high')
        if (not all(finite(row.get(k)) for k in required)
                or row.get('phase')!='entry' or row.get('side')!=self.anchor['side']
                or row.get('method')!='parallel_face_median' or row.get('wall_confirmations',0)<2
                or not 0<=row['speed']<=1 or not 0<=row['now']-row['source']<=self.scan_gap_s
                or abs(row['heading'])>.08 or not .10<=row['along_high']-row['along_low']<=.50
                or row['along_high']<-.10 or row['along_low']>self.maximum_observed_front_m):
            return self.stop_advice('continuation_current_evidence_invalid')
        if not 0<=row['now']-self.yield_since<=self.maximum_continuation_s:
            return self.stop_advice('continuation_budget_expired')
        key=(row.get('epoch'),row['source'])
        if (key[0]!=self.key[0] or key[1]<self.key[1]
                or (key[1]-self.key[1]>self.scan_gap_s and not motion_pause_gap)):
            return self.stop_advice('continuation_source_changed')
        old=self.rows[-1];anchor=self.anchor
        width=lambda r:r['along_high']-r['along_low']
        if (abs(row['normal']-anchor['normal'])>.06 or abs(width(row)-width(anchor))>.06
                or abs(row['heading']-anchor['heading'])>.04
                or min(row['along_high'],old['along_high'])-max(row['along_low'],old['along_low'])<-.06):
            return self.stop_advice('continuation_face_changed')
        if key!=self.key:
            self.rows.append(dict(row))
            while len(self.rows)>self.minimum_scans:self.rows.popleft()
            self.key=key
        self.detail=dict(suggest_yield=True,reason='bounded_current_face_yield_continuation',
            trial_speed_cap_mps=self.trial_speed_cap_mps,elapsed_s=row['now']-self.yield_since,
            scans=len(self.rows),current_band_occupied=bool(row.get('blocked_by_current_returns')),
            current_band_supported=bool(row.get('band_supported')),current_face_along_m=[row['along_low'],row['along_high']],
            control_authorized=False,moving_object_identity_proven=False,unknown_filled=False)
        return self.detail


def observation(output, reference_s):
    """센서 기반 현재 출력만 사용한다. 정답/actor 상태를 읽지 않는다."""
    d=output[2];b=d.get('bypass') or {};corridor=b.get('corridor') or {}
    if d.get('handoff_state')!='BYPASS' or b.get('entry_phase')!='entry' or len(corridor)!=1:return None
    side=next(iter(corridor));surface=corridor[side].get('surface') or {}
    wall=(b.get('reference') or {}).get('walls',{}).get(side) or {}
    band=b.get('rejoin_band') or {};along=surface.get('along_range_m')
    if not isinstance(along,list) or len(along)!=2 or not finite(wall.get('slope')):return None
    return dict(phase=b['entry_phase'],side=side,epoch=b.get('source_epoch'),source=b.get('source_first_s'),
        now=reference_s,heading=math.atan(wall['slope']),
        speed=(d.get('normal') or {}).get('measured_speed_mps'),normal=surface.get('normal_m'),
        along_low=along[0],along_high=along[1],method=surface.get('method'),
        blocked_by_current_returns=band.get('current_occupied_returns',0)>0,
        band_supported=band.get('supported',False),wall_confirmations=b.get('outer_wall_fresh_confirmations',0))


class RelativeYieldCommand:
    """다음 후보 계산의 상한만 조정한다. 실제 측정/최종 명령은 덮어쓰지 않는다."""

    def __init__(self):
        self.observer = BoundedRelativeYield()

    def reset(self):
        self.observer.reset()

    def command(self, owner, scene, speed, evaluate):
        observer = self.observer
        valid_prior = bool(observer.rows and observer.detail.get('suggest_yield')
            and observer.key[0] == getattr(scene, 'source_epoch', None)
            and finite(getattr(scene, 't_ref_s', None))
            and finite(getattr(scene, 'source_first_s', None))
            and 0 <= scene.t_ref_s-observer.rows[-1]['source'] <= observer.scan_gap_s
            and scene.source_first_s >= observer.rows[-1]['source']
            and finite(speed) and 0 <= speed <= owner.config.max_speed_mps
            and owner.mode == 'BYPASS' and owner.bypass.guide.phase == 'entry'
            and owner.bypass.guide.path is not None)
        guide = owner.bypass.guide
        original_entry, original_registered = guide.entry, guide.registered_entry
        try:
            if valid_prior:
                guide.entry = replace(original_entry,
                    speed_cap_mps=min(original_entry.speed_cap_mps, observer.trial_speed_cap_mps))
                guide.registered_entry = replace(original_registered,
                    speed_cap_mps=min(original_registered.speed_cap_mps, observer.trial_speed_cap_mps))
            # 같은 history/실측 speed/scene/직전 조향으로 원래 검사를 한 번 호출한다.
            output = evaluate()
        finally:
            guide.entry, guide.registered_entry = original_entry, original_registered
        current = observation(output, getattr(scene, 't_ref_s', None))
        advice = dict(observer.update(current))
        detail = dict(cap_requested=valid_prior,
            applied_cap_mps=observer.trial_speed_cap_mps if valid_prior else None,
            current_observer=advice, measurement_replaced=False,
            final_command_clipped=False, source='previous_call_advice_current_command_checks')
        return output[0], output[1], dict(output[2], relative_yield=detail)
