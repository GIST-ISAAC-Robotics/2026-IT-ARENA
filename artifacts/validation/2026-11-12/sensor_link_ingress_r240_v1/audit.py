"""별도 struct/CRC 순회로 UART 원문·저장 행·공통 입력·제어 응답을 대조한다."""
from pathlib import Path
import hashlib,json,math,struct,sys,zlib

A=Path(__file__).resolve().parent
r=Path(sys.argv[1]);out=Path(sys.argv[2]) if len(sys.argv)>2 else A/'audit.json'
raw=(r/'run/capture/uart.bin').read_bytes()
rows=[json.loads(x) for x in (r/'run/capture/imu.jsonl').read_text().splitlines()]
rx=[json.loads(x) for x in (r/'run/receiver_rows.jsonl').read_text().splitlines()]
ops=[json.loads(x) for x in (r/'run/capture/control.jsonl').read_text().splitlines()]
imu=[];control=[];discarded=crc_errors=0;offset=0
while offset<len(raw):
    if raw[offset:offset+4]==b'IMU1' and offset+68<=len(raw):
        p=raw[offset:offset+68]
        if int.from_bytes(p[4:6],'little')==68 and p[6]==1 and zlib.crc32(p[:64])==int.from_bytes(p[64:],'little'):
            imu.append(struct.unpack('<IHBBIIQQIHBB16sIII',p));offset+=68;continue
        crc_errors+=1
    elif raw[offset:offset+2]==b'IA' and offset+8<=len(raw):
        n=int.from_bytes(raw[offset+2:offset+4],'little')
        if 0<n<=1536 and offset+n+8<=len(raw):
            p=raw[offset:offset+n+8]
            if zlib.crc32(p[2:-4])==int.from_bytes(p[-4:],'little'):
                control.append(json.loads(p[4:-4]));offset+=n+8;continue
            crc_errors+=1
    discarded+=1;offset+=1
assert len(imu)==len(rows)
for f,row in zip(imu,rows):
    expected=dict(type=f[3],boot=f[4],sequence=f[5],read_begin_us=f[6],read_end_us=f[7],irq_count=f[8],
        fifo_level=f[9],fifo_status=f[10],tag_raw=f[11],payload_hex=f[12].hex(),spi_errors=f[13],tx_errors=f[14])
    assert all(row[k]==v for k,v in expected.items())
    if f[3]==1 and f[11]>>3==1:
        assert row['gyro_rad_s']==[v*.0175*math.pi/180 for v in struct.unpack('<hhh',f[12][:6])]
assert control==[o['payload'] for o in ops if o['direction']=='rx']
gaps=[dict(before=a['sequence'],after=b['sequence'],missing=b['sequence']-a['sequence']-1)
      for a,b in zip(rows,rows[1:]) if b['sequence']!=a['sequence']+1]
summary=json.loads((r/'run/capture/summary.json').read_text())
receiver=json.loads((r/'run/receiver_summary.json').read_text())
supervisor=json.loads((r/'run/supervisor.json').read_text())
restore=json.loads((r/'restore/restore_summary.json').read_text())
real_rx=[v for v in rx if v['kind']=='gyro']
assert len(real_rx)==summary['counts']['gyro_published']==receiver['counts']['gyro']
gyros=[row for row in rows if 'gyro_rad_s' in row]
# 미교정 읽기 시각 변환은 같은 함수로 재확인하고 raw XYZ는 위에서 독립 확인한다.
sys.path.insert(0,str(A.parents[3]/'src/arena_vehicle_interface'))
from arena_vehicle_interface.imu_read_clock import ImuReadClock
clock=ImuReadClock();converted=[];group=[];key=None
for row in rows:
    current=(row['host_monotonic_ns'],row['host_ros_ns'])
    if key is not None and current!=key:
        converted.extend(clock.convert_batch(group,host_monotonic_ns=key[0],ros_receipt_ns=key[1]));group=[]
    key=current
    if 'gyro_rad_s' in row:group.append(row)
if key is not None:converted.extend(clock.convert_batch(group,host_monotonic_ns=key[0],ros_receipt_ns=key[1]))
assert len(converted)==len(real_rx)
for stamp,row in zip(converted,real_rx):
    assert row['stamp_s']==stamp.stamp_ns//10**9+(stamp.stamp_ns%10**9)*1e-9
    assert row['accepted'] and row['values']==stamp.sample['gyro_rad_s']
errors={k:max(row.get(k,0) for row in rows) for k in ('spi_errors','tx_errors','overrun_observations')}
stop=[o['payload'] for o in ops if o['direction']=='rx' and o['payload'].get('type')=='ack'
      and o['payload']['reason']=='software_stop']
offers=[p for p in control if p.get('type')=='offer']
diags=[p for p in control if p.get('type')=='diag']
result=dict(raw_sha256=hashlib.sha256(raw).hexdigest(),raw_bytes=len(raw),imu_frames=len(imu),control_frames=len(control),
    independently_matching_raw_rows=True,common_motion_values_and_stamps_match=True,common_motion_samples=len(real_rx),
    sequence_gaps=gaps,crc_errors=crc_errors,discarded_nonframe_bytes=discarded,sensor_errors=errors,
    all_output_disabled=bool(offers) and all(p['output_ready'] is False for p in offers),
    no_invented_encoder_feedback=bool(diags) and all(p['feedback_valid'] is False and p['motor_counts'] is None for p in diags),
    deadline_miss_max=max((p['deadline_miss_count'] for p in diags),default=None),stop_acks=len(stop),
    original_imu_restored=restore['complete'] and restore['readback_sha256']==restore['target_sha256']==
      '8c9fb78850dcdb56d6c9e14e11e91e8e852dcdf78718038cc85dd6690f81ff27',
    uart_and_ros_lossless=not gaps and not crc_errors and not any(errors.values()) and receiver['counts']['rejected']==0,
    ready_to_drive=False,time_contract_verified=False)
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result))
