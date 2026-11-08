"""기존 경계 운동 관측을 실제 후방 외곽 조각에도 적용하는 저장 입력 비교. 주행 연결 없음."""
from collections import Counter
from dataclasses import replace
import hashlib,inspect,json,math,sys
from pathlib import Path
from unittest.mock import patch
import numpy as np
from arena_autonomy import wall_reference_support as witness_module
from arena_autonomy.bypass_corridor_trial import BoundaryMonitor,CorridorGuide,wall_frame
from arena_autonomy.bypass_handoff_corridor_trial import ForecastGuide
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.known_structure import classify
from arena_autonomy.lidar_motion import MotionConfig,MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import transport_points

HERE=Path(__file__).resolve().parent
SOURCES={'stationary':'stationary_rejoin_state_v1/persistent_block/run1/follow_decisions.jsonl',
         'restart':'stop_restart_newcourse_v1/moving_stop_restart/run1/follow_decisions.jsonl'}
original_padding=ForecastGuide._padding
original_reset=CorridorGuide.reset
# 현재 경로가 이미 후륜축까지 보여도 같은 후방 조각의 관측 근거를 수집한다.
# 유일한 변경은 이 조기 반환 제거다. 원 좌표·연결/크기/각도 검사는 그대로다.
source=inspect.getsource(witness_module.rear_wall_witness)
old='if lower <= -config.wheelbase_m/2 or upper < .30 or upper-lower < .65:'
assert source.count(old)==1
namespace=dict(vars(witness_module))
exec(compile(source.replace(old,'if upper < .30 or upper-lower < .65:'),'<shadow-current-rear-witness>','exec'),namespace)
collect=namespace['rear_wall_witness']


def run(name,enabled):
    path=HERE.parent/SOURCES[name]
    events=[json.loads(l) for l in path.read_text().splitlines()]
    h=events[0]['data'];mc=MotionConfig(**h['motion_config'])
    evidence=[];rows=[]
    def reset(self):
        original_reset(self)
        self._rear_face_monitors={side:BoundaryMonitor(replace(self.corridor_policy,window_s=.8,minimum_scans=9)) for side in ('left','right')}
    def padding(self,scene,config,side,wall,growth_s=0.):
        values,detail=original_padding(self,scene,config,side,wall,growth_s)
        if not enabled:return values,detail
        monitor=self._rear_face_monitors[side]
        face=None;ids=None;proof=None;points=None
        scan=self.raw_scan
        if (self.rear_wall_witness_enabled and self.path is not None and scan is not None
                and self.witness_sectors and len(scan.ranges)==len(scene.rays)
                and abs(scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9-scene.source_first_s)<1e-8
                and wall.get('reason')=='current_connected_straight_wall_fit'):
            _,_,sectors=classify(scan,self.witness_sectors)
            explained={frozenset(r['rays'].tolist()) for r in sectors if r['explained']}
            bounds=wall.get('rear_geometry_witness',{}).get('principal_x_m',wall['observed_x_m'])
            fit=(wall['slope'],wall['intercept_m'],*bounds)
            _,current=collect(scene,config,side,fit,wall,explained)
            proof=current.get('rear_geometry_witness')
            if proof is not None:
                rays=[r for r in scene.rays if r.status=='return']
                points=np.asarray([r.hit_xy_m for r in rays])
                along,normal=wall_frame(points,config,side,wall)
                lo,hi=proof['observed_rear_x_m']
                # 현재 반환이며 차체 뒤끝보다 뒤인 조각만. 앞차/전방 반환은 원래대로다.
                take=(points[:,0]>=lo-1e-9)&(points[:,0]<=min(hi,-config.length_m/2))&(abs(normal)<=.015)
                ids=np.flatnonzero(take)
                if len(ids)>=12 and np.ptp(along[ids])>=.20:
                    ends=ids[[np.argmin(along[ids]),np.argmax(along[ids])]]
                    # 외곽 면은 안으로 들어올수록 normal이 커진다. 기존 monitor의 접근 부호에 맞춘다.
                    face=dict(normal_m=-float(np.median(normal[ids])),along_range_m=[float(min(along[ids])),float(max(along[ids]))],
                              endpoints_xy_m=points[ends])
        def to_current(row):
            if self.motion_history is None or row['endpoints'] is None:return None
            try:moved=transport_points(row['endpoints'],self.motion_history,row['reference'],scene.t_ref_s)
            except (ValueError,MotionUnavailable):return None
            along=wall_frame(moved,config,side,wall)[0]
            return float(min(along)),float(max(along))
        state=monitor.update(scene,face,to_current)
        changed=0;before=values.copy()
        if face is not None and state.get('stable'):
            rays=[r for r in scene.rays if r.status=='return']
            ages=np.asarray([scene.t_ref_s-rays[i].acquired_at_s+growth_s for i in ids])
            reduced=.025+config.measurement_padding_m+state['observed_rate_term_mps']*ages+.5*self.corridor_policy.onset_acceleration_mps2*ages*ages
            values=values.copy();values[ids]=np.minimum(values[ids],reduced)
            changed=int(np.sum(values<before-1e-12))
        record=dict(t=scene.t_ref_s,side=side,face=face is not None,stable=state.get('stable'),changed=changed,
                    state=state,proof=proof,unknown_filled=False,static_identity_claimed=False,
                    rear_indices=[] if ids is None else [rays[i].source_index for i in ids])
        if changed or (scene.source_first_s,side) not in seen:
            evidence.append(freeze(record));seen.add((scene.source_first_s,side))
        return values,dict(detail,rear_face_motion=record)
    seen=set()
    with patch.object(CorridorGuide,'reset',reset),patch.object(ForecastGuide,'_padding',padding):
        ctl=BypassHandoffFollow(FollowConfig(**h['follow_config']),HandoffExecutionConfig.from_mapping(h['execution_config']),allow_simulation_trial=True)
        for e in events[1:]:
            if e['kind']=='reset':ctl.reset()
            elif e['kind']=='call':
                d=e['data'];out=freeze(replay_call(ctl,d['input'],mc));oldout=d['output']
                same=equal_output(comparable_decision(out),comparable_decision(oldout))
                if not enabled:assert same
                rows.append(dict(t=d['input']['reference_s'],before=oldout[:2],after=out[:2],
                                 numeric_same=equal_output(out[:2],oldout[:2]),reason=out[2].get('bypass',{}).get('reason')))
    return rows,evidence

name=sys.argv[1]
prior=json.loads((HERE.parent/'rear_face_motion_shadow_v1'/f'{name}.json').read_text())
assert prior['baseline_all_equal'] and prior['source_sha256']==hashlib.sha256((HERE.parent/SOURCES[name]).read_bytes()).hexdigest()
run_folder=(HERE.parent/SOURCES[name]).parent.parent
registered=json.loads((run_folder/'registered.json').read_text())
root=HERE.parents[3]
assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in registered['sources'].items())
candidate,evidence=run(name,True)
changed=[r for r in candidate if not r['numeric_same']]
result=dict(scope='fixed input only; no live implementation or driving proof',case=name,baseline_calls=prior['baseline_calls'],baseline_all_equal=True,baseline_reused_from_v1=True,rear_window_s=.8,rear_minimum_scans=9,
            changed=len(changed),positive_from_zero=sum(r['before'][0]==0 and r['after'][0]>0 for r in changed),
            changed_reasons=dict(Counter(r['reason'] for r in changed)),changes=changed,evidence=evidence,
            witness_source_sha256=hashlib.sha256(source.encode()).hexdigest(),
            source_sha256=hashlib.sha256((HERE.parent/SOURCES[name]).read_bytes()).hexdigest())
(HERE/f'{name}.json').write_text(json.dumps(freeze(result),ensure_ascii=False,indent=1)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k not in ('changes','evidence')},ensure_ascii=True))
