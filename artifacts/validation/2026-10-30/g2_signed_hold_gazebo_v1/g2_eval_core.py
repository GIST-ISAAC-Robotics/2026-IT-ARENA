"""G2 판정의 핵심 계산(입출력 없음). 정답 자세는 평가에만 쓴다.

run1의 g2_eval.py(10/29 등록본)를 덮어쓰지 않고 판정 창을 분리했다.
- 주행 창: 초록 관측 ~ 하네스 case_window 끝(window_end). 통과·중앙 복귀/유지는 이 창에서만 판정한다.
- STOP 창: window_end ~ 정지 기록 끝(stop_end). 복귀 판정에 섞지 않고 따로 보고·판정한다.
- 접촉·노면·명령 상한은 초록~stop_end 전체에서 본다(더 보수적).
연속 구간은 표본 간격이 max_interval_s(현행 g1d 평가의 ego 자세 coverage 허용 0.05 s)를 넘으면 끊는다.
"""
import math

CORE_VERSION='g2_eval_core_v2'


def coverage(times,start,end,max_interval_s):
    """[start,end] 구간 표본 간격. 양 끝 경계와의 간격도 포함한다."""
    inside=[t for t in times if start<=t<=end]
    if not inside or end<start:
        return dict(valid=False,samples=len(inside),max_interval_s=None,maximum_allowed_interval_s=max_interval_s)
    edges=[start,*inside,end]
    worst=max(b-a for a,b in zip(edges,edges[1:]))
    return dict(valid=worst<=max_interval_s+1e-9,samples=len(inside),first_s=inside[0],last_s=inside[-1],
                max_interval_s=worst,maximum_allowed_interval_s=max_interval_s)


def target_pass_time(rows,target_front_s,window_end):
    """자차 뒤끝 s가 표적 앞끝 s를 처음 넘은 시각(주행 창 안)."""
    for r in rows:
        if r['t']>window_end:break
        if r['rear_s']>target_front_s:return r['t']
    return None


def centered_runs(rows,start,end,lateral_m,heading_rad,max_interval_s):
    """[start,end]에서 중앙 조건을 만족한 연속 구간 [(t0,t1)]. 표본 공백이 허용 간격을 넘으면 끊는다."""
    runs=[];t0=last=None
    for r in rows:
        if r['t']<start or r['t']>end:continue
        ok=abs(r['lateral'])<=lateral_m and abs(r['heading'])<=heading_rad
        if ok and t0 is not None and r['t']-last>max_interval_s:
            runs.append((t0,last));t0=None
        if ok:
            t0=r['t'] if t0 is None else t0;last=r['t']
        elif t0 is not None:
            runs.append((t0,last));t0=None
    if t0 is not None:runs.append((t0,last))
    return runs


def center_return(rows,pass_t,window_end,lateral_m,heading_rad,hold_s,max_interval_s):
    """통과 뒤 주행 창 안에서 hold_s 이상 연속 중앙 유지가 있는지."""
    if pass_t is None:
        return dict(returned=False,first_start_s=None,hold_satisfied_s=None,longest_s=0.,runs=0,centered_at_window_end=False)
    runs=centered_runs(rows,pass_t,window_end,lateral_m,heading_rad,max_interval_s)
    long=[(a,b) for a,b in runs if b-a>=hold_s-1e-9]
    inside=[r for r in rows if pass_t<=r['t']<=window_end]
    end_ok=bool(inside) and abs(inside[-1]['lateral'])<=lateral_m and abs(inside[-1]['heading'])<=heading_rad
    return dict(returned=bool(long),first_start_s=long[0][0] if long else None,
                hold_satisfied_s=long[0][0]+hold_s if long else None,
                longest_s=max((b-a for a,b in runs),default=0.),runs=len(runs),centered_at_window_end=end_ok)


def stop_window(rows,window_end,stop_end,lateral_m,heading_rad):
    """STOP 창(window_end 초과~stop_end)의 횡오차/방향. 복귀 판정과 별도."""
    inside=[r for r in rows if window_end<r['t']<=stop_end]
    if not inside:return dict(samples=0,within_center_tolerance=False,max_abs_lateral_m=None,max_abs_heading_rad=None)
    lat=max(abs(r['lateral']) for r in inside);head=max(abs(r['heading']) for r in inside)
    return dict(samples=len(inside),max_abs_lateral_m=lat,max_abs_heading_rad=head,
                within_center_tolerance=lat<=lateral_m and head<=heading_rad,
                progress_m=inside[-1]['s']-inside[0]['s'])


def bypass_side(rows,pass_t,target_lateral_m):
    """통과 시각의 자차 횡위치 부호가 표적 반대쪽인지(+왼쪽). 오른쪽 표적이면 왼쪽(+)으로 지나야 한다."""
    if pass_t is None:return dict(expected=None,observed=None,matches=False)
    if target_lateral_m==0:
        # 중앙 표적은 쪽을 미리 정하지 않는다. 실제 통과 쪽만 기록한다.
        sample=next(r for r in rows if r['t']>=pass_t)
        return dict(expected=None,observed='left' if sample['lateral']>0 else 'right',matches=True)
    expected='left' if target_lateral_m<0 else 'right'
    at=[r for r in rows if r['t']==pass_t]
    lateral=at[0]['lateral'] if at else None
    observed=None if lateral is None or lateral==0 else ('left' if lateral>0 else 'right')
    return dict(expected=expected,observed=observed,lateral_at_pass_m=lateral,matches=observed==expected)


def surface_ok(surface,max_off_wheels):
    """세 접지면 해석 모두 도로 밖 바퀴≤max이고 3바퀴 이상 사건0·벽 바퀴0·표본 공백 없음."""
    interp=surface.get('interpretations') or {}
    kinds_ok=set(interp)=={'any','center','full'} and all(
        v.get('max_off_wheels') is not None and v['max_off_wheels']<=max_off_wheels and v.get('penalty_events_merged')==0
        for v in interp.values())
    return dict(wheels_two_or_fewer_off_road=kinds_ok,no_wall_wheel=surface.get('wall_wheel_samples')==0,
                surface_samples_continuous=(surface.get('samples') or 0)>0 and surface.get('sample_gaps_over_limit')==0)


def command_cap(trace_rows,decision_speeds,cap):
    """/drive·/drive/safe 기록과 판단 출력 모두 상한 이하인지. 값이 없으면 실패."""
    drive=[r['drive_speed_mps'] for r in trace_rows if r.get('drive_speed_mps') is not None]
    safe=[r['safe_speed_mps'] for r in trace_rows if r.get('safe_speed_mps') is not None]
    values=dict(maximum_drive_mps=max(drive,default=None),maximum_safe_mps=max(safe,default=None),
                maximum_decision_mps=max(decision_speeds,default=None))
    ok=all(v is not None and math.isfinite(v) and v<=cap+1e-9 for v in values.values())
    return dict(ok=ok,cap_mps=cap,**values)


def trace_header_ok(header,expected):
    """schema4·G2 모드·실행 설정·등록 follow/motion 값이 현재 config와 일치하는지."""
    follow=header.get('follow_config') or {};motion=header.get('motion_config') or {}
    checks=dict(
        schema=header.get('schema')==expected['schema'],
        controller_mode=header.get('controller_mode')==expected['controller_mode'],
        execution_config=header.get('execution_config')==expected['execution_config'],
        follow_config={k:follow.get(k)==v for k,v in expected['follow_config'].items()},
        motion_config={k:motion.get(k)==v for k,v in expected.get('motion_config',{}).items()})
    ok=all(v if isinstance(v,bool) else all(v.values()) for v in checks.values())
    return dict(ok=ok,checks=checks)


def replay_ok(replay):
    return (replay.get('passed') is True and (replay.get('calls') or 0)>0 and replay.get('matched')==replay.get('calls')
            and replay.get('mismatches')==0)


def status_ok(status_rows):
    """behavior가 있는 상태 행이 모두 G2 시험 모드인지(빈 경우 실패)."""
    rows=[s for s in status_rows if s.get('behavior') is not None]
    return bool(rows) and all(s.get('mode')=='g2_entry_trial' and s.get('simulation_trial_only') is True
                              and s.get('automatic_race_mode_selection') is False for s in rows)


def static_target(actor_rows,reference,start,end,max_error_m,schedule):
    """고정 표적에서만 대표 자세 사용을 허용한다. 일정이 있으면(이동 표적) 무효."""
    inside=[a for a in actor_rows if start<=a[0]<=end]
    if schedule or not inside:return dict(valid=False,samples=len(inside),max_error_m=None,reason='moving schedule or no samples')
    err=max(math.hypot(a[1]-reference[0],a[2]-reference[1]) for a in inside)
    return dict(valid=err<=max_error_m,samples=len(inside),max_error_m=err,maximum_allowed_m=max_error_m)


def gates(*,rows,green,window_end,stop_end,target_front_s,target_lateral_m,rules,ego_times,contacts_in_run,
          overlap_samples,surface,trace_rows,decision_speeds,header,replay,status_rows,harness_valid,target_static):
    """등록 G2 판정. 각 항목은 bool. 상세값은 함께 반환한다."""
    gap=rules['max_sample_interval_s']
    window_cov=coverage(ego_times,green,window_end,gap)
    stop_cov=coverage(ego_times,window_end,stop_end,gap) if stop_end>window_end else dict(valid=False,samples=0)
    pass_t=target_pass_time(rows,target_front_s,window_end)
    center=center_return(rows,pass_t,window_end,rules['returned_to_center_lateral_m'],rules['returned_to_center_heading_rad'],
                         rules['returned_to_center_hold_s'],gap)
    stop=stop_window(rows,window_end,stop_end,rules['returned_to_center_lateral_m'],rules['returned_to_center_heading_rad'])
    side=bypass_side(rows,pass_t,target_lateral_m)
    surf=surface_ok(surface,rules['max_off_road_wheels'])
    cap=command_cap(trace_rows,decision_speeds,rules['command_cap_mps'])
    head=trace_header_ok(header,rules['trace_header_expected'])
    result=dict(
        ego_window_coverage=window_cov['valid'],ego_stop_coverage=stop_cov['valid'],
        target_passed=pass_t is not None,bypass_side_matches=side['matches'],
        returned_to_center_in_window=center['returned'],
        stop_window_within_center_tolerance=stop['within_center_tolerance'],
        no_actor_contact=contacts_in_run<=rules['actor_contact_messages_with_ego'],
        no_body_overlap=overlap_samples<=rules['overlap_samples'],
        **surf,command_cap_drive_safe_decision=cap['ok'],
        g2_controller_rows=status_ok(status_rows),trace_header_matches_config=head['ok'],
        decision_replay_identical=replay_ok(replay),harness_valid=harness_valid is True,
        static_target_pose_valid=target_static.get('valid') is True)
    detail=dict(window_coverage=window_cov,stop_coverage=stop_cov,pass_time_s=pass_t,
                pass_time_since_green_s=None if pass_t is None else pass_t-green,center=center,stop_window=stop,
                bypass_side=side,command_cap=cap,trace_header=head,target_static=target_static)
    return result,detail
