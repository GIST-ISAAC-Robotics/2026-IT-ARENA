"""실제 수신·비구동 출력과 별도 계산 부하를 구분해 집계한다."""
from pathlib import Path
import bisect,json,statistics
A=Path(__file__).resolve().parent
R=A/'remote_result'/'actual';S=R/'live'
def read(name,root=S):return json.loads((root/name).read_bytes())
def lines(name):return [json.loads(x) for x in (S/name).read_text(encoding='utf-8').splitlines()]
def stats(values):
    v=sorted(values)
    return dict(n=len(v),median=statistics.median(v),p95=v[int((len(v)-1)*.95)],maximum=v[-1]) if v else None

def main():
    result=read('result.json',R);sup=read('supervisor.json')
    node,imu,c1,camera=[read(n+'/summary.json') for n in ('node','imu','c1','camera')]
    published=lines('imu/clock_estimates.jsonl');pubstamps=[r['stamp_ns']/1e9 for r in published]
    gyro=[r for r in read('node/motion.json') if r.get('kind')=='gyro']
    matched=set();unmatched=[]
    for row in gyro:
        index=bisect.bisect_left(pubstamps,row['stamp_s'])
        candidates=[i for i in (index-1,index) if 0<=i<len(pubstamps)]
        near=min(candidates,key=lambda i:abs(pubstamps[i]-row['stamp_s'])) if candidates else None
        if near is None or abs(pubstamps[near]-row['stamp_s'])>1e-6 or near in matched:unmatched.append(row)
        else:matched.add(near)
    images=read('node/images.json');sent=read('camera/frames.json')
    sent_ids={r['stamp_ns'] for r in sent};received_ids=[r['stamp_ns'] for r in images]
    vision=read('node/vision.json');timeline=read('timeline.json',R)
    calc_start=next(r['wall_ns'] for r in timeline if r['event']=='calculation_started')/1e9
    calc_end=next(r['wall_ns'] for r in timeline if r['event']=='calculation_ended')/1e9
    live_span=[min(r['received_s'] for r in gyro),max(r['received_s'] for r in gyro)] if gyro else None
    overlap=max(0,min(calc_end,live_span[1])-max(calc_start,live_span[0])) if live_span else 0
    before=read('before.json',R);after=read('after.json',R)
    cleanup=(not result['fault'] and not result['after_error'] and not result['forced']
        and result['ports_free'] and not result['changed_sources']
        and all(code==0 for code in result['children'].values())
        and not sup['fault'] and not sup['forced'] and sup['ports_free'] and sup['source_unchanged']
        and all(code==0 for code in sup['children'].values())
        and imu['inactive_status_observed'] and c1['sensor_stop_ack'] and camera['pipeline_null']
        and before['status_last'][-1]['active'] is False and after['status_last'][-1]['active'] is False)
    zero=(node['nonzero_private_commands']==0 and not node['actuator_node_started']
          and node['motion']['drive_motor']==node['motion']['wheels']==0
          and not result['actuator_started'] and not result['firmware_written'])
    answer=dict(scope='live sensor shadow plus isolated recorded planner load; no full live control or independent guard proof',
        cleanup_passed=cleanup,zero_output_passed=zero,
        gyro=dict(published=len(published),callbacks=len(gyro),accepted=node['motion']['gyro'],
            rejected=node['motion']['rejected'],missing=len(published)-len(matched),unmatched_callbacks=len(unmatched),
            estimated_stamp_to_callback_ms=stats([(r['received_s']-r['stamp_s'])*1000 for r in gyro]),
            callback_gap_ms=stats([(b['received_s']-a['received_s'])*1000 for a,b in zip(gyro,gyro[1:])]),
            decoder=imu['decoder'],boot_count=imu['boot_count']),
        c1=dict(published=c1['counts']['published'],received=node['scans'],rate_hz=c1['observed_rate_hz'],
            point_count=[c1['point_count_min'],c1['point_count_max']]),
        camera=dict(published=len(sent),received=len(images),missing=len(sent_ids-set(received_ids)),
            unexpected=len(set(received_ids)-sent_ids),duplicate_callbacks=len(received_ids)-len(set(received_ids)),
            producer_fps=camera['producer_fps']),
        vision=dict(results=len(vision),processing_ms=stats([r['processing_ms'] for r in vision])),
        workload_process_and_live_callback_overlap_s=overlap,
        workload_overlap_basis='calculation process start/end vs live gyro receipt, includes startup',
        live_commands=node['commands'],live_states=node['states'],live_geometry_profile='existing shadow 135mm, not final 128mm',
        live_plan_uses_encoder_substitute=False,sensor_timestamp_verified=False,
        calculation=read('calculation/report.json',R))
    answer['motion_input_delivery_passed']=(not answer['gyro']['missing'] and not unmatched
         and c1['counts']['published']==node['scans'])
    (A/'audit.json').write_text(json.dumps(answer,indent=2)+'\n')
    print(json.dumps(answer,ensure_ascii=False))
    assert cleanup and zero and overlap>10,'cleanup/output/workload evidence incomplete'

if __name__=='__main__':main()
