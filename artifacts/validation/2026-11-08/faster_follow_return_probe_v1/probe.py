"""저장 r32 입력에서 앞차 상대 간격 추세와 추종 출력의 동시 가용성. 경로 전환/새 주행 아님."""
from collections import Counter, deque
from dataclasses import replace
from hashlib import sha256
import json
import math
from pathlib import Path

import numpy as np
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import FollowConfig,MotionConfig,restore_call,freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionUnavailable

HERE=Path(__file__).resolve().parent
TRACE=HERE.parent/'moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl'

def replay():
    history_rows=deque(); rows=[]; base_mismatch=0; eligible=[]
    for line in TRACE.open(encoding='utf8'):
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            cfg=FollowConfig(**d['follow_config'])
            ex=HandoffExecutionConfig.from_mapping(d['execution_config'])
            mc=MotionConfig(**d['motion_config'])
            baseline=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True)
            # 일반 추종 비교에서도 faster_v1의 1.2 m/s 시차 예산을 그대로 사용한다.
            normal=BypassHandoffFollow(baseline.config,replace(ex,moving_pass_trial=False,
                moving_pass_profile='slow_v1'),allow_simulation_trial=True)
        if e['kind']=='reset':
            baseline.reset(); normal.reset(); history_rows.clear()
        if e['kind']!='call': continue
        x,scan,hist=restore_call(d['input'],mc); t=x['reference_s']
        try:
            scene=motion_scene(scan,hist,t,timing_verified=x['timing_verified'],epoch=x['epoch'],
                               rear_half_angle_deg=x['rear_half_angle_deg'])
            args=(scene,hist,x['speed_mps'],x['side'],x['previous_steering_rad'],x['dt_s'])
            old=freeze(baseline.command(*args,raw_scan=scan))
            alt=freeze(normal.command(*args,raw_scan=scan))
        except (MotionUnavailable,ValueError):
            baseline.reset();normal.reset();history_rows.clear()
            old=alt=[0.,0.,{}]
        base_mismatch+=not np.allclose(old[:2],d['output'][:2],rtol=0.,atol=1e-10)
        adapter=baseline.moving_pass;tracker=adapter.tracker
        ego_side=adapter.guide.side
        entry=old[2].get('moving_pass',{}).get('entry',{})
        wall=entry.get('reference',{}).get('walls',{}).get(ego_side,{})
        tgt=tracker.detail
        if not (adapter.active and tracker.current is not None and tgt.get('ahead_clear')
                and wall.get('reason')=='current_connected_straight_wall_fit'):
            history_rows.clear(); continue
        # 동일 실제 측정의 상대 간격 추세: 벽 접선으로 투영하여 자차 회전만으로 생기는 x축 변화를 줄인다.
        slope=wall['slope']; tangent=np.array((1.,slope))/math.hypot(1.,slope)
        distance=float(np.median(tracker.current@tangent))
        stamp=scene.source_first_s
        if not history_rows or stamp>history_rows[-1][0]:
            history_rows.append((stamp,distance))
            while len(history_rows)>1 and stamp-history_rows[0][0]>.50+1e-6:history_rows.popleft()
        if len(history_rows)>=4 and history_rows[-1][0]-history_rows[0][0]>=.30-1e-6:
            growth=history_rows[-1][1]-history_rows[0][1]
        else: growth=None
        nd=alt[2].get('normal',{})
        observed=bool(nd.get('fresh_boundary_accepted') or nd.get('path_points'))
        g1=bool(alt[2].get('handoff_state')=='FOLLOW' and alt[0]>0 and observed)
        rec=dict(t=t,source=stamp,old=old[:2],normal=alt[:2],normal_reason=nd.get('reason'),
            normal_state=alt[2].get('handoff_state'),normal_path_present=observed,
            target_ahead=True,distance_m=distance,rear_bound_m=tgt.get('rear_bound_m'),
            growth_m=growth,measured_speed=x['speed_mps'],normal_path_source_s=nd.get('path_source_s'),
            normal_selection=nd.get('selector_reason'))
        rows.append(rec)
        # gate for exploration ONLY: two independent range errors; original occupied/unknown checks remain.
        if old[0]==0 and growth is not None and growth>2*cfg.measurement_padding_m and g1:eligible.append(rec)
    (HERE/'rows.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows),encoding='utf8')
    out=dict(scope='saved r32 front separation and parallel ordinary FOLLOW availability only',
        baseline_command_mismatches=int(base_mismatch),tracked_observations=len(rows),
        stopped_with_increasing_gap=sum(r['old'][0]==0 and r['growth_m'] is not None
            and r['growth_m']>2*cfg.measurement_padding_m for r in rows),
        co_available=len(eligible),first=eligible[:1],last=eligible[-1:],trace_sha256=sha256(TRACE.read_bytes()).hexdigest(),
        alternatives_while_stopped=dict(Counter(str(r['normal_state'])+' / '+str(r['normal_reason'])+' / '+str(r['normal'][0]>0)
            for r in rows if r['old'][0]==0)),source_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
    (HERE/'summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False))
    if base_mismatch:raise SystemExit(2)

if __name__=='__main__':replay()
