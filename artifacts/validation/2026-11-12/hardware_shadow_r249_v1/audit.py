from pathlib import Path
import hashlib,json,zipfile,shutil
R=Path(__file__).resolve().parents[4];A=Path(__file__).resolve().parent
archive=R/'build/r249_results.zip';digest=hashlib.sha256(archive.read_bytes()).hexdigest()
assert digest=='cc1656cbd1fc0d8f3f8d8689ae6a4e80c0d39356c04bfce5b488b4f43d2ec579'
out=R/'build/r249_recovered';out.mkdir(exist_ok=False)
with zipfile.ZipFile(archive) as z:
    for n in z.namelist():assert not Path(n).is_absolute() and '..' not in Path(n).parts
    z.extractall(out)
read=lambda p:json.loads(p.read_bytes())
node,imu,c1=[read(out/n/'summary.json') for n in ('node','imu','c1')]
source=[json.loads(s)['payload'] for s in (out/'c1/converted.jsonl').read_text().splitlines()]
received=read(out/'node/scans.json')
assert len(source)==len(received)==node['scans']==c1['counts']['published']==203
for a,b in zip(source,received):
    assert (a['source_frame_id'],a['source_epoch'],a['time_basis'],len(a['ranges_m']),a['acquired_at_s'][0],a['acquired_at_s'][-1]) == (
        b['frame'],b['epoch'],b['time_basis'],b['points'],b['first_s'],b['last_s'])
assert node['motion']['gyro']==imu['clock']['published']==4032
assert node['motion']['drive_motor']==node['motion']['wheels']==node['motion']['rejected']==0
assert node['nonzero_private_commands']==0 and node['drive_output'] is False
assert imu['inactive_status_observed'] and c1['sensor_stop_ack']
supervisor=read(out/'supervisor.json')
assert supervisor['ports_free'] and supervisor['source_unchanged'] and not supervisor['forced']
assert all(v==0 for v in supervisor['children'].values())
for n in ('node','imu','c1'):shutil.copy2(out/n/'summary.json',A/(n+'_summary.json'))
shutil.copy2(out/'supervisor.json',A/'supervisor.json')
result=dict(archive_sha256=digest,c1_array_metadata_unchanged=len(source),gyro_published_and_common_accepted=node['motion']['gyro'],
    encoder_count=0,positive_private_commands=0,time_contract_verified=False,mounting_calibrated=False,
    no_camera_or_autonomous_drive=True,children_normal=True,ports_free=True)
(A/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
