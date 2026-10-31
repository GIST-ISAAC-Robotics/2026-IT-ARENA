"""미연결 관측 후보: 복귀 띠의 물체가 뒤로 빠지지 않을 때 감속을 제안한다.

물체 ID/속도 추정이나 주행 허가가 아니다. 알려진 점유를 삭제하지 않는다.
"""
from collections import deque
import math


def finite(v):
    return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)


class RelativeYieldObserver:
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


def observation(call):
    """저장된 센서 기반 제어기 출력에서만 추출한다. actor/Gazebo 정답 없음."""
    d=call['output'][2];b=d.get('bypass') or {};corridor=b.get('corridor') or {}
    if d.get('handoff_state')!='BYPASS' or b.get('entry_phase')!='entry' or len(corridor)!=1:return None
    side=next(iter(corridor));surface=corridor[side].get('surface') or {}
    wall=(b.get('reference') or {}).get('walls',{}).get(side) or {}
    band=b.get('rejoin_band') or {};along=surface.get('along_range_m')
    if not isinstance(along,list) or len(along)!=2 or not finite(wall.get('slope')):return None
    return dict(phase=b['entry_phase'],side=side,epoch=b.get('source_epoch'),source=b.get('source_first_s'),
        now=call['input']['reference_s'],heading=math.atan(wall['slope']),
        speed=(d.get('normal') or {}).get('measured_speed_mps'),normal=surface.get('normal_m'),
        along_low=along[0],along_high=along[1],method=surface.get('method'),
        blocked_by_current_returns=band.get('current_occupied_returns',0)>0,
        band_supported=band.get('supported',False),wall_confirmations=b.get('outer_wall_fresh_confirmations',0))
