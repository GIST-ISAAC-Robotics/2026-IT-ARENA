"""기존 실물 원문만 분석. 보드/ROS/센서 값/타임스탬프를 수정하지 않는다."""
import hashlib
import json
from pathlib import Path
import statistics
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_vehicle_interface')]
from probe_esp32_imu import Decoder
from fifo_time import FifoTimeSlots,fit_sensor_to_mcu


def spread(values):
    x=sorted(values)
    return dict(count=len(x),min=x[0],p50=statistics.median(x),
                p95=x[int((len(x)-1)*.95)],max=x[-1]) if x else None


def main():
    raw=ROOT/'build/r254_recovered/run/capture/uart.bin'
    digest=hashlib.sha256(raw.read_bytes()).hexdigest()
    decoder=Decoder();frames=decoder.feed(raw.read_bytes())
    slots=FifoTimeSlots();samples=[]
    for row in frames:
        result=slots.feed(row)
        if result:samples.append(result)
    assert not decoder.counters['crc_errors'] and not decoder.counters['invalid_header']
    gyro=[r for r in samples if r['sample']['sensor_tag']==1]
    accel=[r for r in samples if r['sample']['sensor_tag']==2]
    original_gyro=[r for r in frames if r['type']==1 and r['sensor_tag']==1]
    assert [r['sample'] for r in gyro]==original_gyro
    assert len({r['generation'] for r in samples})==1
    anchors=[r for r in frames if r['type']==2 and r.get('active')]
    fit=fit_sensor_to_mcu(anchors,boot=gyro[0]['boot'])
    brackets=[]
    for r in gyro:
        bracket=fit.map_interval(r['sensor_tick']*25,boot=str(r['boot']))
        # 여기서 host는 Jetson이 아니라 MCU esp_timer 시간 영역이다.
        begin=r['sample']['read_begin_us']*1000;end=r['sample']['read_end_us']*1000
        brackets.append(dict(sequence=r['sample']['sequence'],sensor_tick=r['sensor_tick'],
            mcu_capture_lower_ns=bracket.host_lower_ns,mcu_capture_upper_ns=bracket.host_upper_ns,
            interval_width_ns=bracket.width_ns,read_lag_lower_ns=begin-bracket.host_upper_ns,
            read_lag_upper_ns=end-bracket.host_lower_ns))
    assert hashlib.sha256(raw.read_bytes()).hexdigest()==digest
    report=dict(source=str(raw.relative_to(ROOT)).replace('\\','/'),source_sha256=digest,
        raw_frames=len(frames),gyro_paired=len(gyro),accel_paired=len(accel),counts=dict(slots.counts),
        sensor_tick_deltas=spread([b['sensor_tick']-a['sensor_tick'] for a,b in zip(gyro,gyro[1:])]),
        mcu_read_interval_us=spread([b['sample']['read_end_us']-a['sample']['read_end_us'] for a,b in zip(gyro,gyro[1:])]),
        paired_values_and_order_unchanged=True,register_brackets=len(anchors),
        tick_ns_interval=[min(float(v[0])*25000 for v in fit.vertices),max(float(v[0])*25000 for v in fit.vertices)],
        mapped_interval_width_us=spread([r['interval_width_ns']/1000 for r in brackets]),
        read_lag_lower_us=spread([r['read_lag_lower_ns']/1000 for r in brackets]),
        read_lag_upper_us=spread([r['read_lag_upper_ns']/1000 for r in brackets]),
        sensor_time_after_read_count=sum(r['mcu_capture_lower_ns']>g['sample']['read_end_us']*1000 for r,g in zip(brackets,gyro)),
        assumptions=dict(sensor_rate_from_nominal_25us_max_error_ppm=100000,mcu_bracket_error_ns=1000,sensor_tick_quantization_us=25),
        scope='sensor FIFO tick to MCU conditional affine bracket only; no Jetson/ROS mapping, installed calibration or runtime adoption')
    (A/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    (A/'mapped_gyro.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in brackets))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
