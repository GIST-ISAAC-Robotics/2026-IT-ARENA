"""원천/발행/콜백/제어 참조시각을 분리한다. DDS 내부 도착시각은 추정하지 않는다."""
from collections import Counter
import gzip
import json
from pathlib import Path
import sys
from analyze import quantiles


def analyze(root):
    read=lambda name:json.loads((root/name).read_bytes())
    mapping=read('clock_mapping.json')
    wall0,sim0=mapping['monotonic_origin_s'],mapping['simulation_origin_s']
    active=lambda t:7.62<=t<27.52
    with gzip.open(root/'publication_lateness.jsonl.gz','rt',encoding='utf8') as stream:
        published=[json.loads(line) for line in stream]
    publication={(r['kind'],round(r['at']*1e9)):wall0+r['at']-sim0+r['late_ms']/1000 for r in published}
    result=dict(interval_s=[7.62,27.52],mapping=mapping,nodes={},
        boundary='publication time is before publish call; callback entry is after rclpy conversion; DDS arrival unavailable')
    evidence={}
    for role,name in (('controller','local_pursuit'),('guard','lidar_safety')):
        timing=[json.loads(line) for line in (root/(name+'_timing.jsonl')).read_text().splitlines()]
        controls=[r for r in read(role+'_controls.json') if active(r['control_reference_s'])]
        stats={}
        for callback in ('imu','wheels','scan','image','control','command'):
            rows=[r for r in timing if r['callback']==callback and active(sim0+r['begin_monotonic_ns']/1e9-wall0)]
            if not rows:continue
            stats[callback]=dict(count=len(rows),duration_ms=quantiles(r['duration_ms'] for r in rows),
                handler_gap_ms=quantiles(r['wall_interval_ms'] for r in rows if r['wall_interval_ms'] is not None),
                source_age_ros_ms=quantiles(r['source_age_ms'] for r in rows if r['source_age_ms'] is not None),
                clock_lag_ms=quantiles((sim0+r['begin_monotonic_ns']/1e9-wall0-r['receive_ros_ns']/1e9)*1000 for r in rows))
            if callback in ('imu','wheels'):
                kind='gyro' if callback=='imu' else 'wheels'
                delivered=[r for r in rows if (kind,r['source_ns']) in publication]
                stats[callback]['publication_to_callback_ms']=quantiles(
                    (r['begin_monotonic_ns']/1e9-publication[(kind,r['source_ns'])])*1000 for r in delivered)
                stats[callback]['publication_to_callback_over30ms']=sum(
                    r['begin_monotonic_ns']/1e9-publication[(kind,r['source_ns'])]>.03 for r in delivered)
        labels=Counter();failure_rows=[]
        guard_status={}
        if role=='guard':
            with gzip.open(root/'guard_status.jsonl.gz','rt',encoding='utf8') as stream:
                for line in stream:
                    s=json.loads(line)['value'];guard_status[round(s['reference_s']*1e9)]=s
        for control in controls:
            ref=control['control_reference_s'];row=dict(control)
            for label in ('gyro','wheel'):
                source=control.get(label+'_available_source_s')
                row[label+'_age_ms']=None if source is None else (ref-source)*1000
            if role=='controller':
                reason=control.get('motion_meta',{}).get('reason') if control.get('state')=='MOTION_STOP' else control.get('state')
            else:
                state=guard_status.get(round(ref*1e9),{})
                reason=state.get('reason','unmatched_guard_status');row['guard_status']=state
            row['reason']=reason;labels[reason]+=1
            if reason=='motion_stale':
                row['expired_motion_channels']=[k for k in ('gyro','wheel') if row[k+'_age_ms'] is not None and row[k+'_age_ms']>30.000001]
                row['clock_now_minus_entry_ms']=(ref-control['begin_clock_s'])*1000
                failure_rows.append(row)
        result['nodes'][role]=dict(callbacks=stats,control_count=len(controls),reasons=dict(labels),
            motion_stale_count=len(failure_rows),
            stale_gyro=sum('gyro' in r['expired_motion_channels'] for r in failure_rows),
            stale_wheel=sum('wheel' in r['expired_motion_channels'] for r in failure_rows),
            stale_gyro_age_ms=quantiles(r['gyro_age_ms'] for r in failure_rows if r['gyro_age_ms'] is not None),
            stale_wheel_age_ms=quantiles(r['wheel_age_ms'] for r in failure_rows if r['wheel_age_ms'] is not None),
            stale_control_duration_ms=quantiles((r['end_monotonic_ns']-r['begin_monotonic_ns'])/1e6 for r in failure_rows))
        evidence[role]=failure_rows
    (root/'causal_summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    (root/'failure_controls.json').write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf8')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':analyze(Path(sys.argv[1]))
