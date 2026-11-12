"""수신 콜백과 원 발행 표본을 대조한다. 시작/종료 누락도 별도로 숨기지 않는다."""
from pathlib import Path
import bisect,json,statistics

A=Path(__file__).resolve().parent
S=A/'actual/stack'
read=lambda name:json.loads((S/name).read_bytes())
lines=lambda name:[json.loads(s) for s in (S/name).read_text().splitlines()]
def stats(rows):
    rows=sorted(rows)
    return dict(n=len(rows),median=statistics.median(rows),p95=rows[int((len(rows)-1)*.95)],maximum=max(rows)) if rows else None
def groups(indices):
    result=[]
    for i in indices:
        if not result or i!=result[-1][-1]+1:result.append([])
        result[-1].append(i)
    return result

node,imu=read('node/summary.json'),read('imu/summary.json')
motion=read('node/motion.json');gyro=[r for r in motion if r['kind']=='gyro']
published=lines('imu/clock_estimates.jsonl')
stamps=[r['stamp_ns']/1e9 for r in published]
assert all(b>a for a,b in zip(stamps,stamps[1:]))
matched={}
for row in gyro:
    i=bisect.bisect_left(stamps,row['stamp_s'])
    near=min((j for j in (i-1,i) if 0<=j<len(stamps)),key=lambda j:abs(stamps[j]-row['stamp_s']))
    assert abs(stamps[near]-row['stamp_s'])<=1e-6 and near not in matched
    matched[near]=row
assert len(gyro)==node['motion']['gyro']+node['motion']['rejected']
missing=[i for i in range(len(published)) if i not in matched]
missing_groups=[]
for block in groups(missing):
    a,b=block[0],block[-1]
    before=matched.get(a-1);after=matched.get(b+1)
    missing_groups.append(dict(count=len(block),indices=[a,b],sequence=[published[a]['sequence'],published[b]['sequence']],
        source_from_first_s=[stamps[a]-stamps[0],stamps[b]-stamps[0]],before=before,after=after,
        callback_gap_s=after['received_s']-before['received_s'] if before and after else None))
ops=lines('imu/control.jsonl')
status=[r['payload'] for r in ops if r['direction']=='status_published']
status_rx=read('node/mcu_states.json')
index={r['host_receipt_ns']:r for r in status}
assert len(index)==len(status)
for row in status_rx:assert row['payload']==index[row['payload']['host_receipt_ns']]
assert len({r['payload']['host_receipt_ns'] for r in status_rx})==len(status_rx)
images=read('node/images.json');vision=read('node/vision.json')
tx=[r['payload']['command']['kind'] for r in ops if r['direction']=='tx_queued']
assert tx==['STOP'] and imu['counts']['rx_ack']==1
observations=[r['payload'] for r in ops if r['direction']=='rx' and r['payload'].get('type')=='runtime_observation_v1']
assert all(not r['output_ready'] and not r['encoder_motion_verified'] and r['pwm_gpio_level']==r['dir_gpio_level']==0 for r in observations)
assert node['nonzero_private_commands']==0 and node['motion']['drive_motor']==node['motion']['wheels']==0
sup=read('supervisor.json');bench=json.loads((A/'actual/result.json').read_bytes())
assert sup['ports_free'] and sup['source_unchanged'] and not sup['forced'] and not sup['fault']
assert all(v==0 for v in sup['children'].values())
assert bench['restored'] and bench['port_unowned'] and all(not r['active'] for r in bench['after_restore']['imu_status_last'])
assert not json.loads((A/'source_check.json').read_bytes())['changed']
latency=[(r['received_s']-r['stamp_s'])*1000 for r in gyro]
gaps=[dict(gap_ms=(b['received_s']-a['received_s'])*1000,before=a,after=b) for a,b in zip(gyro,gyro[1:])]
result=dict(scope='unmounted hardware, output disabled; no valid encoder or moving vehicle',
    passed=not missing and len(status)==len(status_rx),
    gyro=dict(published=len(published),callbacks=len(gyro),accepted=node['motion']['gyro'],rejected=node['motion']['rejected'],
        missing=len(missing),missing_groups=missing_groups,source_to_callback_ms=stats(latency),
        callback_gap_ms=stats([r['gap_ms'] for r in gaps]),largest_callback_gaps=sorted(gaps,key=lambda r:r['gap_ms'],reverse=True)[:8]),
    camera=dict(published=read('camera/summary.json')['frames'],received=len(images),first=images[:2],last=images[-2:]),
    vision=dict(accepted=len(vision),first=vision[:2],processing_ms=stats([r['processing_ms'] for r in vision])),
    mcu_status=dict(published=len(status),received=len(status_rx),payloads_exact=True),
    uart=dict(frames=imu['counts']['imu_frames'],decoder=imu['imu_decoder'],control_errors=imu['control_errors'],
        queue_overflow=imu['serial_io']['queue_overflow']),
    c1=dict(published=read('c1/summary.json')['counts']['published'],received=node['scans']),
    positive_commands=0,final_stop_ack=True,app_restored=True,ports_free=True,normal_shutdown=True,source_unchanged=True)
(A/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('gyro','camera','vision')},ensure_ascii=False))
print('gyro',len(published),len(gyro),'missing',len(missing),'groups',[(r['count'],r['source_from_first_s'],r['callback_gap_s']) for r in missing_groups])
print('latency_ms',stats(latency))
