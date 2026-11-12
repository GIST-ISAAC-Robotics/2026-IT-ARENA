from pathlib import Path
import hashlib,json,statistics,sys
A=Path(__file__).resolve().parent;R=A.parents[3];out=A/'actual/stack'
read=lambda p:json.loads(p.read_bytes())
def stats(v):
    v=sorted(v)
    return dict(count=len(v),median=statistics.median(v),p95=v[int((len(v)-1)*.95)],max=max(v)) if v else None
node,imu,c1,camera=[read(out/n/'summary.json') for n in ('node','imu','c1','camera')]
source=[r['payload'] for r in map(json.loads,(out/'c1/converted.jsonl').read_text().splitlines()) if r['payload'] is not None]
received=read(out/'node/scans.json')
assert len(source)==len(received)==node['scans']==c1['counts']['published']
for a,b in zip(source,received):
    assert (a['source_frame_id'],a['source_epoch'],a['time_basis'],len(a['ranges_m']),a['acquired_at_s'][0],a['acquired_at_s'][-1]) == (b['frame'],b['epoch'],b['time_basis'],b['points'],b['first_s'],b['last_s'])
assert node['motion']['drive_motor']==node['motion']['wheels']==node['motion']['rejected']==0
assert node['nonzero_private_commands']==0 and node['drive_output'] is False
assert c1['sensor_stop_ack'] and camera['pipeline_null'] and imu['serial_io']['port_closed']
sent=read(out/'camera/frames.json');images=read(out/'node/images.json');vision=read(out/'node/vision.json')
index={r['stamp_ns']:r for r in sent};actual={r['stamp_ns'] for r in images}
assert len(index)==len(sent)==camera['frames'] and len(actual)==len(images)==node['images']
for row in images:
    assert row['stamp_ns'] in index and row['bytes']==index[row['stamp_ns']]['bytes']
    assert (row['width'],row['height'],row['step'],row['encoding'],row['frame'])==(1280,720,3840,'bgr8','b0390_receive_time_unverified')
ops=[json.loads(l) for l in (out/'imu/control.jsonl').open()]
rx=[r['payload'] for r in ops if r['direction']=='rx'];offers=[r for r in rx if r.get('type')=='offer']
observations=[r for r in rx if r.get('type')=='runtime_observation_v1']
assert all(x['output_ready'] is False for x in offers+observations)
assert all(x['encoder_motion_verified'] is False and x['pwm_gpio_level']==x['dir_gpio_level']==0 for x in observations)
tx=[r['payload']['command']['kind'] for r in ops if r['direction']=='tx_queued']
assert tx==['STOP'] and imu['counts']['rx_ack']==1
supervisor=read(out/'supervisor.json');bench=read(A/'actual/result.json')
assert supervisor['ports_free'] and supervisor['source_unchanged'] and not supervisor['forced'] and not supervisor['fault']
assert all(v==0 for v in supervisor['children'].values())
assert bench['restored'] and bench['port_unowned'] and all(not x['active'] for x in bench['after_restore']['imu_status_last'])
assert not read(A/'source_check.json')['changed']
delta=imu['clock']['published']-node['motion']['gyro']
result=dict(passed=delta==0,scope='actual unmounted sensor ingestion / no drive or valid encoder',
    c1_arrays_metadata_unchanged=len(source),c1_points=sum(len(r['ranges_m']) for r in source),
    gyro=dict(published=imu['clock']['published'],accepted=node['motion']['gyro'],unaccounted=delta,rejected=node['motion']['rejected'],
        loss_location='ROS publish to common callback; no per-callback capture in r311'),
    uart=dict(imu_frames=imu['counts']['imu_frames'],crc_or_decode_errors=imu['control_errors'],
        sequence_decoder=imu['imu_decoder'],queue_overflow=imu['serial_io']['queue_overflow']),
    camera=dict(sent=len(sent),received=len(images),missing=len(sent)-len(images),format_stamp_mismatch=0,pixels_verified=False),
    vision=dict(accepted=len(vision),processing_ms=stats([v['processing_ms'] for v in vision]),
        source_to_acceptance_ms=stats([1000*(v['accepted_ros_s']-v['stamp']) for v in vision])),
    raw_mcu_observations=len(observations),mcu_status_published=imu['counts']['mcu_status_published'],
    mcu_status_common_receipt_verified=False,output_ready=False,positive_commands=0,final_stop_ack=True,
    app_restored=True,ports_free=True,normal_shutdown=True,source_unchanged=True)
(A/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
if delta:print('RESULT: receive accounting incomplete; preserve failure and add callback evidence')
