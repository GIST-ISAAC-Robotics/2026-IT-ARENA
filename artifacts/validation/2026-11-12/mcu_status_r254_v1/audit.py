from collections import Counter
import hashlib,json,shutil,statistics,struct,zipfile,zlib
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
z=R/'build/r254_results_private.zip';sha=hashlib.sha256(z.read_bytes()).hexdigest()
assert sha=='802b72d2291e2ed8780df7ced686ee1fe410c5e78b26c0451445096e05715899'
out=R/'build/r254_recovered';out.mkdir(exist_ok=False)
with zipfile.ZipFile(z) as archive:
    assert all(not Path(n).is_absolute() and '..' not in Path(n).parts for n in archive.namelist())
    archive.extractall(out)
read=lambda p:json.loads(p.read_bytes())
lines=lambda p:[json.loads(s) for s in p.read_text().splitlines()]
bridge=read(out/'run/capture/summary.json');receiver=read(out/'run/receiver_summary.json');supervisor=read(out/'run/supervisor.json')
raw=(out/'run/capture/uart.bin').read_bytes();i=0;control=[];imus=[];nonframe=0
fmt=struct.Struct('<IHBBIIQQIHBB16sIII')
while i<len(raw):
    points=[p for p in (raw.find(b'IA',i),raw.find(b'IMU1',i)) if p>=0]
    if not points:nonframe+=len(raw)-i;break
    start=min(points);nonframe+=start-i;i=start
    imu=raw[i:i+4]==b'IMU1'
    if i+(7 if imu else 4)>len(raw):break
    size=struct.unpack_from('<H',raw,i+(4 if imu else 2))[0]
    assert (size==68 and raw[i+6]==1) if imu else 0<size<=1536
    length=68 if imu else size+8
    if i+length>len(raw):break
    chunk=raw[i:i+length]
    assert zlib.crc32(chunk[:-4] if imu else chunk[2:-4])==struct.unpack_from('<I',chunk,length-4)[0]
    if imu:imus.append(fmt.unpack(chunk))
    else:control.append(json.loads(chunk[4:-4]))
    i+=length
assert len(imus)==11973==bridge['counts']['imu_frames']
assert all(b[5]==a[5]+1 and b[4]==a[4] for a,b in zip(imus,imus[1:]))
assert all(row[13]==row[14]==0 for row in imus)
source=lines(out/'run/capture/control.jsonl')
assert [r['payload'] for r in source if r['direction']=='rx']==control
offers={(r['boot'],r['status_now_us']):r for r in control if r['type']=='offer'}
sent=[r['payload'] for r in source if r['direction']=='status_published']
seen=lines(out/'run/mcu_status_rows.jsonl')
assert sent==[r['payload'] for r in seen]
assert len(sent)==976==receiver['mcu_observer']['counts']['accepted']
assert all(r['outcome']=='accepted' and r['fresh'] for r in seen)
for item in sent:
    original=offers[item['boot'],item['mcu_now_us']]
    assert all(item[k]==original[k] for k in ('v','boot','generation','state','reason','estop_held','output_ready'))
    assert not item['time_contract_verified'] and not item['output_ready']
    assert 0<=item['host_receipt_ns']-item['receipt_age_ns']-item['now_us']*1000<1000
assert receiver['counts']['gyro']==bridge['counts']['gyro_published']==3901
assert receiver['motor_samples']==receiver['counts']['drive_motor']==receiver['counts']['rejected']==0
assert bridge['serial_io']['port_closed'] and not bridge['serial_io']['error']
assert supervisor['children_exit']==[0,0,0] and supervisor['original_imu_restored']
assert [r['success'] for r in supervisor['controls']]==[False,True,False,True]
for label in ('flash','restore'):
    data=read(out/label/'restore_summary.json')
    assert data['complete'] and data['readback_sha256']==data['target_sha256']
    shutil.copyfile(out/label/'restore_summary.json',A/(label+'_summary.json'))
for name,source_path in [('bridge_summary.json',out/'run/capture/summary.json'),
                         ('receiver_summary.json',out/'run/receiver_summary.json'),
                         ('supervisor.json',out/'run/supervisor.json')]:shutil.copyfile(source_path,A/name)
ages=sorted((r['received_s']-r['payload']['now_us']*1e-6)*1000 for r in seen)
result=dict(archive_sha256=sha,raw_imu_frames=len(imus),raw_sequence_gaps=0,raw_crc_errors=0,
    control_frames=len(control),offers=len(offers),status_published_received_equal=len(sent),
    status_fields_equal_to_actual_mcu=True,state_counts=dict(Counter(r['state'] for r in sent)),
    generations=sorted({r['generation'] for r in sent}),
    status_callback_age_ms=dict(p50=statistics.median(ages),p95=ages[int((len(ages)-1)*.95)],maximum=max(ages)),
    common_gyro_accepted=receiver['counts']['gyro'],drive_frames=bridge['counts'].get('tx_DRIVE',0),
    encoder_samples=0,output_ready=False,firmware_restored=True,time_contract_verified=False,
    scope='physical MCU state and IMU reception, common observer, output-disabled app; no operational permission or motion')
(A/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
