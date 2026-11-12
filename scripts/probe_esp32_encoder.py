#!/usr/bin/env python3
"""수동 엔코더 CRC 프레임 기록. 모터·서보 명령과 PPR 보정 적용 없음."""
import argparse
import collections
import json
from pathlib import Path
import time
import zlib


class Decoder:
    def __init__(self):
        self.buffer=bytearray();self.errors=collections.Counter()

    def feed(self,data):
        self.buffer.extend(data);frames=[]
        while b'\n' in self.buffer:
            raw,_,self.buffer=self.buffer.partition(b'\n')
            if len(raw)<10 or raw[8:9]!=b' ':
                self.errors['nonframe_lines']+=1;continue
            try:checksum=int(raw[:8],16)
            except ValueError:self.errors['nonframe_lines']+=1;continue
            body=raw[9:]
            if zlib.crc32(body)!=checksum:
                self.errors['crc_errors']+=1;continue
            try:item=json.loads(body)
            except (ValueError,UnicodeDecodeError):self.errors['json_errors']+=1;continue
            if item.get('event') not in ('info','status','edge'):
                self.errors['unknown_event']+=1;continue
            frames.append(item)
        if len(self.buffer)>16384:
            self.errors['oversize_line']+=1;self.buffer.clear()
        return frames


def summarize(frames,errors):
    statuses=[r for r in frames if r['event']=='status']
    edges=[r for r in frames if r['event']=='edge']
    phases={}
    for name in dict.fromkeys(r['phase'] for r in statuses):
        part=[r for r in statuses if r['phase']==name]
        e=[r for r in edges if r['phase']==name]
        first,last=part[0],part[-1]
        fields=('pcnt_count','software_count','edges','a_rise','a_fall','b_rise','b_fall',
                'invalid','same_state_irq','irqs','ring_dropped')
        phases[name]=dict(status_samples=len(part),edge_records=len(e),
            mcu_span_seconds=(last['begin_us']-first['begin_us'])/1e6,
            first={k:first[k] for k in fields},last={k:last[k] for k in fields},
            delta={k:last[k]-first[k] for k in fields},
            observed_states=sorted({r['ab'] for r in part}|{r['before'] for r in e}|{r['after'] for r in e}),
            edge_direction_counts=dict(collections.Counter(str(r['delta']) for r in e)),
            final_pcnt_minus_software=last['pcnt_count']-last['software_count'])
    sequence_gaps=[]
    for a,b in zip(frames,frames[1:]):
        if a['boot']==b['boot'] and (b['seq']-a['seq'])&0xffffffff!=1:
            sequence_gaps.append(dict(before=a['seq'],after=b['seq']))
    return dict(status_samples=len(statuses),edge_records=len(edges),phases=phases,
        decoder_errors=dict(errors),sequence_gaps=sequence_gaps,
        boot_count=len({r['boot'] for r in frames}),
        ring_dropped_max=max((r['ring_dropped'] for r in statuses),default=0),
        invalid_transitions_max=max((r['invalid'] for r in statuses),default=0),
        uart_tx_errors_max=max((r['tx_errors'] for r in statuses),default=0),
        counts_per_motor_revolution_adopted=None,absolute_accuracy_verified=False,
        analog_voltage_or_edge_quality_measured=False,motor_energized=False)


def run(args):
    import serial
    output=args.output;output.mkdir(parents=True,exist_ok=False)
    phase_path=output/'phase.json';phase_path.write_text(json.dumps({'phase':'baseline'}))
    decoder=Decoder();frames=[];phase='startup';last_print=0;started=False;stopped=False
    port=serial.Serial(args.port,921600,timeout=.05,write_timeout=1,exclusive=True)
    raw=(output/'uart-private.bin').open('wb');log=(output/'frames.jsonl').open('w',encoding='utf-8')
    boot=None;firmware_info=None;measure_begin=None;markers=[]

    def receive():
        nonlocal boot,firmware_info
        data=port.read(max(1,min(port.in_waiting,8192)))
        raw.write(data);now=time.monotonic_ns();batch=decoder.feed(data)
        for r in batch:
            if boot is None:boot=r['boot']
            elif boot!=r['boot']:raise RuntimeError('MCU reboot during capture')
            r.update(host_monotonic_ns=now,phase=phase)
            frames.append(r);log.write(json.dumps(r)+'\n')
            if r['event']=='info':
                firmware_info=r
                if r['firmware'] not in ('encoder_probe_v1','encoder_probe_v2') or r['motor_pwm'] or r['motor_dir']:
                    raise RuntimeError('firmware/output mismatch')
        log.flush();raw.flush()
        return batch

    try:
        port.write(b'?');deadline=time.monotonic()+4
        while time.monotonic()<deadline and firmware_info is None:receive()
        if firmware_info is None:raise RuntimeError('no encoder firmware identity')
        phase='baseline';port.write(b'S');port.flush();started=True;measure_begin=time.monotonic()
        markers.append(dict(phase=phase,host_monotonic_ns=time.monotonic_ns()))
        print('CAPTURE_READY motor_power_must_remain_disconnected',flush=True)
        while time.monotonic()-measure_begin<args.seconds:
            if (output/'STOP').exists():break
            try:new=json.loads(phase_path.read_text()).get('phase',phase)
            except (OSError,ValueError):new=phase
            if new!=phase:
                phase=new;markers.append(dict(phase=phase,host_monotonic_ns=time.monotonic_ns()))
            batch=receive()
            status=[r for r in batch if r['event']=='status']
            if status and time.monotonic()-last_print>3:
                r=status[-1];print(json.dumps({k:r[k] for k in ['phase','pcnt_count','software_count',
                    'a_rise','a_fall','b_rise','b_fall','invalid','ring_dropped','ab']}),flush=True)
                last_print=time.monotonic()
    finally:
        try:
            if started:
                phase='stopping';port.write(b'X');port.flush();deadline=time.monotonic()+2
                while time.monotonic()<deadline:
                    batch=receive()
                    if any(r['event']=='status' and not r['streaming'] for r in batch):
                        stopped=True;break
        finally:
            port.close();log.close();raw.close()
            summary=summarize(frames,decoder.errors)
            summary.update(stop_acknowledged=stopped,host_port_closed=True,
                phase_markers=markers,firmware_info={k:v for k,v in (firmware_info or {}).items()
                    if k not in ('boot','seq','host_monotonic_ns','phase')})
            (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
            print('CAPTURE_FINISHED '+json.dumps({k:v for k,v in summary.items() if k!='phases'}),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--seconds',type=float,default=300)
    run(parser.parse_args())
