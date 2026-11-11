#!/usr/bin/env python3
"""진단 UART 원문/원시 FIFO 보존. 취득 시각 확정·교정값 적용은 하지 않는다."""
import argparse
import collections
import gc
import json
import math
from pathlib import Path
import statistics
import struct
import time
import zlib

WIRE = struct.Struct('<IHBBIIQQIHBB16sIII')
MAGIC = b'IMU1'


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.counters = collections.Counter()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 4:
            index = self.buffer.find(MAGIC)
            if index < 0:
                n = max(0, len(self.buffer)-3)
                self.counters['nonframe_bytes'] += n
                del self.buffer[:n]
                break
            if index:
                self.counters['nonframe_bytes'] += index
                del self.buffer[:index]
            if len(self.buffer) < WIRE.size:
                break
            packet = bytes(self.buffer[:WIRE.size])
            f = WIRE.unpack(packet)
            if f[1] != WIRE.size or f[2] != 1:
                self.counters['invalid_header'] += 1
                del self.buffer[0]
                continue
            if zlib.crc32(packet[:-4]) != f[-1]:
                self.counters['crc_errors'] += 1
                del self.buffer[0]
                continue
            del self.buffer[:WIRE.size]
            item = dict(type=f[3], boot=f[4], sequence=f[5], read_begin_us=f[6],
                        read_end_us=f[7], irq_count=f[8], fifo_level=f[9],
                        fifo_status=f[10], tag_raw=f[11], payload_hex=f[12].hex(),
                        spi_errors=f[13], tx_errors=f[14])
            p = f[12]
            if item['type'] == 1:
                item['sensor_tag'] = f[11] >> 3
                item['raw_xyz'] = list(struct.unpack('<hhh', p[:6]))
                if item['sensor_tag'] == 1:
                    item['gyro_rad_s'] = [v*.0175*math.pi/180 for v in item['raw_xyz']]
                elif item['sensor_tag'] == 2:
                    item['accel_mps2'] = [v*.000244*9.80665 for v in item['raw_xyz']]
                elif item['sensor_tag'] == 4:
                    item['sensor_timestamp_raw'] = struct.unpack('<I', p[:4])[0]
            elif item['type'] == 2:
                ts, ng, na, temp, active, _ = struct.unpack('<IIIhBB', p)
                item.update(sensor_timestamp_raw=ts, gyro_counter=ng, accel_counter=na,
                            temperature_raw=temp, temperature_c=25+temp/256, active=bool(active))
            elif item['type'] == 3:
                flash, psram, errors, reset = struct.unpack('<IIII', p)
                item.update(who_am_i=f[11], flash_bytes=flash, psram_bytes=psram,
                            memory_errors=errors, reset_reason=reset)
            elif item['type'] == 4:
                item['registers'] = {str(f[11]+i): v for i, v in enumerate(p)}
            elif item['type'] == 5:
                item['overrun_observations'] = struct.unpack('<I', p[:4])[0]
            result.append(item)
        return result


def spread(values):
    if not values:
        return None
    a = sorted(values)
    return dict(count=len(a), min=a[0], mean=statistics.fmean(a),
                median=statistics.median(a), p95=a[int((len(a)-1)*.95)],
                p99=a[int((len(a)-1)*.99)], max=a[-1],
                stddev=statistics.pstdev(a))


def summarize(records, decoder, duration):
    measured = [r for r in records if r['phase'] == 'measure']
    raw = [r for r in measured if r['type'] == 1]
    out = dict(measured_host_seconds=duration, wire_bytes=WIRE.size,
               decoder=dict(decoder.counters), phase_counts=dict(collections.Counter(r['phase'] for r in records)),
               measured_frames=len(measured), boot_count=len({r['boot'] for r in records}),
               spi_errors_max=max((r['spi_errors'] for r in records), default=0),
               tx_errors_max=max((r['tx_errors'] for r in records), default=0),
               fifo_level_max=max((r['fifo_level'] for r in measured), default=0),
               irq_count_max=max((r['irq_count'] for r in records), default=0),
               overrun_observations_max=max((r.get('overrun_observations',0) for r in records), default=0),
               raw_tags=dict(collections.Counter(r['sensor_tag'] for r in raw)),
               time_contract_verified=False, calibration_adopted=False,
               installation='user_reported_tilted_unmounted_jumper_wires', streams={})
    gaps = []; prev = None
    for r in records:
        if prev and prev['boot'] == r['boot']:
            jump=(r['sequence']-prev['sequence']) & 0xffffffff
            if jump != 1: gaps.append(dict(before=prev['sequence'],after=r['sequence'],jump=jump))
        prev=r
    out['sequence_gaps']=gaps
    out['metadata']=[r for r in records if r['type']==3][:1]
    regs = {}
    for r in measured:
        if r['type']==4: regs.update(r['registers'])
    out['registers_last']=regs
    if '99' in regs:
        fine=regs['99'] if regs['99']<128 else regs['99']-256
        factor=1+.0015*fine
        out['factory_clock_readout']=dict(internal_freq_fine_raw=regs['99'],
            signed=fine, nominal_fractional_frequency_difference=.0015*fine,
            nominal_timestamp_tick_us=25, candidate_tick_us=25/factor,
            candidate_odr_hz=(1/(192*25e-6))*factor,
            applied_to_recorded_timestamps=False, time_contract_verified=False)
    out['temperature_c']=spread([r['temperature_c'] for r in measured if 'temperature_c' in r])
    for tag, key in [(1,'gyro_rad_s'),(2,'accel_mps2')]:
        a=[r for r in raw if r['sensor_tag']==tag]
        if not a: continue
        span=(a[-1]['read_begin_us']-a[0]['read_begin_us'])/1e6
        s=dict(count=len(a), host_window_rate_hz=len(a)/duration,
               mcu_read_span_rate_hz=(len(a)-1)/span if span else None,
               read_interval_ms=spread([(q['read_begin_us']-p['read_begin_us'])/1000 for p,q in zip(a,a[1:])]),
               read_duration_us=spread([r['read_end_us']-r['read_begin_us'] for r in a]),
               axes=[spread([r[key][i] for r in a]) for i in range(3)],
               saturated_axis_values=sum(abs(v)>=32760 for r in a for v in r['raw_xyz']))
        if tag==2: s['norm_mps2']=spread([math.sqrt(sum(v*v for v in r[key])) for r in a])
        if tag==1:
            split=a[0]['read_begin_us']+60000000
            first=[r for r in a if r['read_begin_us']<split]
            later=[r for r in a if r['read_begin_us']>=split]
            bias=[statistics.fmean(r[key][i] for r in first) for i in range(3)]
            s['provisional_only']=dict(estimated_first_seconds=60,
                apparent_bias_rad_s=bias, stationarity_independently_verified=False,
                later_residual_mean_rad_s=[statistics.fmean(r[key][i]-bias[i] for r in later) for i in range(3)] if later else None,
                coefficients_written_or_applied=False, recheck_after_mounting=True)
        out['streams'][key]=s
    ts=[r['sensor_timestamp_raw'] for r in raw if r['sensor_tag']==4]
    out['fifo_timestamp_delta_ticks']=spread([(b-a)&0xffffffff for a,b in zip(ts,ts[1:])])
    return out


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=300)
    parser.add_argument('--warmup', type=float, default=10)
    parser.add_argument('--keep-gc', action='store_true', help='비교용. 기본은 수신 중 순환 GC를 중지한다.')
    args=parser.parse_args()
    import serial
    args.output.mkdir(parents=True,exist_ok=False)
    decoder=Decoder(); records=[]
    ser=serial.Serial(port=None,baudrate=921600,timeout=.2,write_timeout=2)
    ser.dtr=False; ser.rts=False; ser.port=args.port; ser.open()
    start=time.monotonic(); first_data=None; deadline=start+20
    received_start=None; stop_sent=False; measurement_end=None
    gc_was_enabled=gc.isenabled()
    with (args.output/'uart.bin').open('wb') as rawlog, (args.output/'frames.jsonl').open('w') as lines:
        def receive(phase):
            nonlocal first_data, received_start
            data=ser.read(max(1,min(ser.in_waiting,8192)))
            mono=time.monotonic_ns(); wall=time.time_ns()
            rawlog.write(data)
            decoded=decoder.feed(data)
            for r in decoded:
                r.update(host_monotonic_ns=mono,host_wall_ns=wall,phase=phase)
                records.append(r); lines.write(json.dumps(r,separators=(',',':'))+'\n')
                if r['type']==1 and first_data is None: first_data=time.monotonic()
                if received_start is None: received_start=time.monotonic()
            return decoded
        try:
            ser.write(b'X'); time.sleep(.1)
            while time.monotonic()-start<2: receive('setup')
            metas=[r for r in records if r['type']==3]
            if not metas: raise RuntimeError('No diagnostic metadata: firmware/USB path not confirmed')
            m=metas[-1]
            if m['who_am_i']!=0x6c or m['memory_errors'] or m['spi_errors']:
                raise RuntimeError('Identity/memory/SPI initial check failed: '+json.dumps(m))
            first_data=None
            if not args.keep_gc:
                gc.disable()  # 비순환 표본 객체는 참조 계수로 해제한다. 수신 중 전체 세대 순회 지연을 피한다.
            ser.write(b'S')
            while first_data is None:
                if time.monotonic()>deadline: raise TimeoutError('No FIFO samples')
                receive('startup')
            warm_end=first_data+args.warmup
            while time.monotonic()<warm_end: receive('warmup')
            measure_start=time.monotonic()
            while time.monotonic()-measure_start<args.seconds: receive('measure')
            measurement_end=time.monotonic()
        finally:
            ser.write(b'X'); stop_sent=True
            until=time.monotonic()+2
            while time.monotonic()<until: receive('stop')
            ser.close()
            if gc_was_enabled:
                gc.enable()
            duration=(measurement_end-measure_start) if measurement_end is not None else 0
            summary=summarize(records,decoder,duration) if duration else dict(complete=False,decoder=dict(decoder.counters))
            summary['stop_command_sent']=stop_sent
            summary['inactive_status_observed']=any(r.get('active') is False for r in records if r['phase']=='stop')
            summary['cyclic_gc_disabled_during_capture']=not args.keep_gc
            (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(complete=True,output=str(args.output),frames=len(records),
                         counts=summary.get('raw_tags'),stop_confirmed=summary['inactive_status_observed'])))


if __name__=='__main__': main()
