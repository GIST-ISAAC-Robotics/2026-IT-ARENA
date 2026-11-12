import hashlib,itertools,json,math,statistics,sys
from pathlib import Path
A=Path(__file__).resolve().parent;root=A.parents[3]
sys.path[:0]=[str(root/'src/arena_vehicle_interface'),str(root/'scripts')]
from arena_vehicle_interface.imu_fifo_clock import ImuFifoClock
from probe_esp32_imu import Decoder,summarize
run=root/'build/r268_recovered/run'
records=[json.loads(x) for x in (run/'capture/frames.jsonl').read_text().splitlines()]
received=[json.loads(x) for x in (run/'receiver_rows.jsonl').read_text().splitlines()]
bridge=json.loads((run/'capture/summary.json').read_text());decoder=Decoder()
wire=decoder.feed((run/'capture/uart.bin').read_bytes())
assert len(wire)==len(records)
assert all(all(record[k]==v for k,v in raw.items()) for raw,record in zip(wire,records))
clock=ImuFifoClock();sent=[]
for key,batch in itertools.groupby(records,key=lambda r:(r['host_monotonic_ns'],r['host_ros_ns'],r['phase'])):
 rows=list(batch)
 if key[2] in ('warmup','measure'):
  sent.extend(clock.convert_batch(rows,host_monotonic_ns=key[0],ros_receipt_ns=key[1]))
assert len(sent)==len(received)==bridge['clock']['published']
max_stamp_error=max(abs((r['stamp_s']-(s.stamp_ns//10**9+(s.stamp_ns%10**9)*1e-9))) for s,r in zip(sent,received))
assert max_stamp_error==0
assert all(r['kind']=='gyro' and r['accepted'] and r['values']==s.sample['gyro_rad_s'] for s,r in zip(sent,received))
summary=summarize(records,decoder,bridge['duration_s'])
assert not summary['sequence_gaps'] and not decoder.counters.get('crc_errors')
assert summary['spi_errors_max']==summary['tx_errors_max']==summary['overrun_observations_max']==0
result=dict(wire_frames=len(wire),ros_gyro=len(received),measured_gyro=bridge['counts']['gyro_received_measure'],
            measured_rate_hz=bridge['counts']['gyro_received_measure']/bridge['duration_s'],
            raw_ros_values_identical=True,stamp_reconstruction_max_error_s=max_stamp_error,
            sequence_gap_count=len(summary['sequence_gaps']),crc_errors=decoder.counters.get('crc_errors',0),
            spi_errors=summary['spi_errors_max'],tx_errors=summary['tx_errors_max'],fifo_overrun=summary['overrun_observations_max'],
            nonframe_bytes=decoder.counters.get('nonframe_bytes',0),raw_preserved=True,
            time_contract_verified=False,installation_calibrated=False,operational_drive_proven=False)

clock_records=[json.loads(x) for x in (run/'capture/clock_estimates.jsonl').read_text().splitlines()]
assert len(clock_records)==len(sent)
for saved,stamp in zip(clock_records,sent):
 assert saved['sequence']==stamp.sample['sequence']
 assert saved['stamp_ns']==stamp.stamp_ns
 assert saved['mcu_lower_ns']==stamp.mcu_lower_ns and saved['mcu_upper_ns']==stamp.mcu_upper_ns
 assert saved['register_anchor_sequence']==stamp.register_anchor_sequence < stamp.sample['sequence']
 assert not saved['time_contract_verified']
result.update(causal_clock_records_identical=True,read_to_capture_shift_us=dict(
 min=min((s.read_based_stamp_ns-s.stamp_ns)/1000 for s in sent),
 median=statistics.median((s.read_based_stamp_ns-s.stamp_ns)/1000 for s in sent),
 max=max((s.read_based_stamp_ns-s.stamp_ns)/1000 for s in sent)))

(A/'live_audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
