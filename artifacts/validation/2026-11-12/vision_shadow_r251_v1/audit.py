from pathlib import Path
import hashlib,json,zipfile,shutil,statistics
R=Path(__file__).resolve().parents[4];A=Path(__file__).resolve().parent
archive=R/'build/r251_results.zip';digest=hashlib.sha256(archive.read_bytes()).hexdigest()
assert digest=='c304d6063fb189cffc8b686cefcb7125a22c69829333422b315abc60f7217cea'
out=R/'build/r251_recovered';out.mkdir(exist_ok=False)
with zipfile.ZipFile(archive) as z:
    for n in z.namelist():assert not Path(n).is_absolute() and '..' not in Path(n).parts
    z.extractall(out)
read=lambda p:json.loads(p.read_bytes())
def stats(values):
    q=sorted(values)
    return dict(count=len(q),median=statistics.median(q),p95=q[int((len(q)-1)*.95)],maximum=max(q)) if q else None
node,imu,c1,camera=[read(out/n/'summary.json') for n in ('node','imu','c1','camera')]
source=[json.loads(s)['payload'] for s in (out/'c1/converted.jsonl').read_text().splitlines()]
received=read(out/'node/scans.json')
assert len(source)==len(received)==node['scans']==c1['counts']['published']
for a,b in zip(source,received):
    assert (a['source_frame_id'],a['source_epoch'],a['time_basis'],len(a['ranges_m']),a['acquired_at_s'][0],a['acquired_at_s'][-1]) == (
        b['frame'],b['epoch'],b['time_basis'],b['points'],b['first_s'],b['last_s'])
assert node['motion']['gyro']==imu['clock']['published']
assert node['motion']['drive_motor']==node['motion']['wheels']==node['motion']['rejected']==0
assert node['nonzero_private_commands']==0 and node['drive_output'] is False
assert imu['inactive_status_observed'] and c1['sensor_stop_ack'] and camera['pipeline_null']
sent=read(out/'camera/frames.json');images=read(out/'node/images.json');vision=read(out/'node/vision.json')
index={r['stamp_ns']:r for r in sent};actual={r['stamp_ns'] for r in images}
assert len(index)==len(sent)==camera['frames'] and len(actual)==len(images)==node['images']
for row in images:
    assert row['stamp_ns'] in index and row['bytes']==index[row['stamp_ns']]['bytes']
    assert (row['width'],row['height'],row['step'],row['encoding'],row['frame'])==(1280,720,3840,'bgr8','b0390_receive_time_unverified')
missing=[i for i,r in enumerate(sent) if r['stamp_ns'] not in actual]
assert len(vision)==node['vision']['stats']['accepted'] and not node['vision']['fault']
assert not node['worker_alive'] and not node['vision_alive'] and node['worker_exitcode']==node['vision_exitcode']==0
supervisor=read(out/'supervisor.json')
assert supervisor['ports_free'] and supervisor['source_unchanged'] and not supervisor['forced']
assert all(v==0 for v in supervisor['children'].values())
for n in ('node','imu','c1','camera'):shutil.copy2(out/n/'summary.json',A/(n+'_summary.json'))
shutil.copy2(out/'supervisor.json',A/'supervisor.json')
result=dict(archive_sha256=digest,c1_array_metadata_unchanged=len(source),gyro_published_and_common_accepted=node['motion']['gyro'],
    camera=dict(sent=len(sent),received=len(images),missing_indices=missing,duplicate=0,format_or_stamp_mismatch=0,
        receive_age_ms=stats([(r['received_ns']-r['stamp_ns'])/1e6 for r in images]),
        callback_gap_ms=stats([(b['received_ns']-a['received_ns'])/1e6 for a,b in zip(images,images[1:])]),
        payload_bytes_verified=False,exposure_time_verified=False),
    vision=dict(accepted=len(vision),processing_ms=stats([r['processing_ms'] for r in vision]),
        completion_wait_ms=stats([r['completion_wait_ms'] for r in vision]),
        acceptance_source_age_ms=stats([1000*(r['accepted_ros_s']-r['stamp']) for r in vision]),
        decoded_marker_observations=sum(len(r['ids']) for r in vision),accuracy_evaluated=False),
    encoder_count=0,positive_private_commands=0,time_contract_verified=False,mounting_calibrated=False,
    children_normal=True,ports_free=True,scope='actual image recognition plus C1/IMU ingestion; no planning load or actuation proof')
(A/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
