#!/usr/bin/env python3
"""격리 UART 릴레이를 통한 ST 서보 상태 읽기와 명시 선택 ±10도 무부하 시험."""
import argparse
import json
from pathlib import Path
import statistics
import struct
import time

BAUDS = (1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400)


def packet(servo_id, instruction, parameters=b''):
    body = bytes((servo_id, len(parameters)+2, instruction)) + bytes(parameters)
    return b'\xff\xff' + body + bytes((~sum(body) & 255,))


def responses(raw, sent):
    frames = []; errors = []; offset = 0
    while offset < len(raw):
        i = raw.find(b'\xff\xff', offset)
        if i < 0:
            errors.append('trailing_bytes'); break
        if i != offset: errors.append('prefix_bytes')
        if i+4 > len(raw):
            errors.append('partial_header'); break
        size = raw[i+3]+4
        if size < 6 or i+size > len(raw):
            errors.append('partial_or_invalid_length'); offset=i+1; continue
        p = raw[i:i+size]; offset=i+size
        if p == sent: continue  # Adapter local echo is not a servo reply.
        if sum(p[2:]) & 255 != 255:
            errors.append('checksum'); continue
        frames.append(dict(id=p[2], status=p[4], data=p[5:-1]))
    return frames, errors


def signed_magnitude(value, bit):
    return -(value & ~(1 << bit)) if value & (1 << bit) else value


def feedback(data):
    if len(data) != 15: raise ValueError('feedback length')
    pos, speed, load = struct.unpack('<HHH', data[:6])
    current = struct.unpack('<H', data[13:15])[0]
    return dict(position_raw=pos, position_counts=signed_magnitude(pos,15),
                position_degrees=signed_magnitude(pos,15)*360/4096,
                speed_raw=speed, speed_signed=signed_magnitude(speed,15),
                load_raw=load, load_signed=signed_magnitude(load,10),
                voltage_v=data[6]/10, temperature_c=data[7],
                registered=data[8], servo_status=data[9], moving=data[10],
                target_feedback_raw=int.from_bytes(data[11:13],'little'),
                current_raw=current, current_signed=signed_magnitude(current,15),
                current_a_candidate=signed_magnitude(current,15)*.0065)


class Probe:
    def __init__(self, port, output):
        import serial
        self.output=output; output.mkdir(parents=True,exist_ok=False)
        self.log=(output/'transactions.jsonl').open('w',encoding='utf-8')
        self.serial=serial.Serial(port,921600,timeout=.15,write_timeout=1,exclusive=True)
        self.bad=0; self.timeouts=0; self.requests=0; self.boot=None
        self.servo_id=None; self.active=False

    def command(self, line, event=None, timeout=1.2):
        begin=time.monotonic_ns(); self.serial.write((line+'\n').encode('ascii'));self.serial.flush()
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            raw=self.serial.readline()
            try: reply=json.loads(raw)
            except (ValueError,UnicodeDecodeError): continue
            entry=dict(host_monotonic_ns=time.monotonic_ns(),request=line,reply=reply)
            self.log.write(json.dumps(entry)+'\n');self.log.flush()
            if reply.get('event')=='lease_expired':
                self.active=False; raise RuntimeError('firmware lease expired')
            if reply.get('event')=='rejected': raise RuntimeError('firmware rejected '+line)
            if event is None or reply.get('event')==event:
                reply['host_roundtrip_ms']=(time.monotonic_ns()-begin)/1e6
                if 'boot' in reply:
                    if self.boot is not None and self.boot!=reply['boot']:
                        raise RuntimeError('MCU reboot during session')
                    self.boot=reply['boot']
                return reply
        raise TimeoutError('relay response '+line)

    def connect(self):
        time.sleep(1.2)
        for _ in range(5):
            try:
                info=self.command('INFO','info')
                if info['firmware']!='servo_probe_v1' or info['motor_pwm'] or info['motor_dir']:
                    raise RuntimeError('unexpected firmware / motor output')
                return info
            except TimeoutError: pass
        raise RuntimeError('servo probe firmware not responding')

    def transaction(self, servo_id, instruction, parameters=b'', required=True):
        sent=packet(servo_id,instruction,parameters);self.requests+=1
        r=self.command('Q '+sent.hex(),'reply');raw=bytes.fromhex(r['rx'])
        frames, errors=responses(raw,sent); self.bad+=len(errors)
        expected=len(parameters)==2 and instruction==2 and parameters[1] or 0
        matches=[f for f in frames if f['id']==servo_id and len(f['data'])==expected]
        if errors or r['io_error']:raise RuntimeError('UART/packet framing error '+repr(errors))
        if len(matches)!=1:
            self.timeouts+=1
            if required: raise RuntimeError('servo response missing/ambiguous')
            return None
        f=matches[0]
        if f['status'] and required:raise RuntimeError('servo error status '+str(f['status']))
        return f,r

    def read(self, address, count):
        return self.transaction(self.servo_id,2,bytes((address,count)))

    def write(self, address, data):
        return self.transaction(self.servo_id,3,bytes((address,))+bytes(data))

    def scan(self, broad=False):
        for baud in BAUDS:
            self.command('B '+str(baud),'baud')
            ids=range(254) if broad else (1,)
            for ident in ids:
                r=self.transaction(ident,1,required=False)
                if r is not None:
                    self.servo_id=ident
                    return dict(id=ident,baud=baud,ping_status=r[0]['status'])
        raise RuntimeError('No servo response; broad scan available with --scan-all-ids')

    def capture(self, seconds, label, target=None):
        rows=[];deadline=time.monotonic()+seconds;next_sample=time.monotonic()
        while time.monotonic()<deadline:
            f,r=self.read(56,15);row=feedback(f['data'])
            row.update(label=label,host_monotonic_ns=time.monotonic_ns(),
                       mcu_begin_us=r['begin_us'],mcu_end_us=r['end_us'],
                       host_roundtrip_ms=r['host_roundtrip_ms'],target=target)
            rows.append(row)
            with (self.output/'feedback.jsonl').open('a',encoding='utf-8') as log:
                log.write(json.dumps(row)+'\n')
            if self.active and (row['servo_status'] or not 9.0<=row['voltage_v']<=12.6 or
                                row['temperature_c']>=50 or abs(row['current_signed'])>=150):
                raise RuntimeError('motion feedback outside bench bounds '+repr(row))
            next_sample+=.05;time.sleep(max(0,next_sample-time.monotonic()))
        return rows

    def close(self):
        try:
            if self.active:self.command('X','disarmed');self.active=False
        finally:self.serial.close();self.log.close()


def run(args):
    probe=Probe(args.port,args.output);summary=dict(mode=args.mode,scope='unmounted_no_horn_no_linkage',
        eeprom_modified=False,physical_angle_independently_measured=False,
        motor_power_user_reported_disconnected=True)
    rows=[]
    try:
        summary['relay']=probe.connect();summary['servo']=probe.scan(args.scan_all_ids)
        cfg=probe.read(0,40)[0]['data'];ctrl=probe.read(40,16)[0]['data']
        summary['config_before_hex']=cfg.hex();summary['control_before_hex']=ctrl.hex()
        summary['identity']=dict(firmware=list(cfg[:2]),endian=cfg[2],model_raw_le=int.from_bytes(cfg[3:5],'little'),
            id=cfg[5],baud_code=cfg[6],return_level=cfg[8],angle_min=int.from_bytes(cfg[9:11],'little'),
            angle_max=int.from_bytes(cfg[11:13],'little'),phase=cfg[18],resolution=cfg[30],
            offset_raw=int.from_bytes(cfg[31:33],'little'),mode=cfg[33])
        rows+=probe.capture(args.seconds,'read_only')
        if args.mode in ('motion','watchdog'):
            ident=summary['identity'];sample=rows[-1];centre=sample['position_counts']
            if ident['mode']!=0 or ident['endian']!=0 or ident['resolution']!=1:
                raise RuntimeError('unexpected position-mode configuration')
            if sample['servo_status'] or not 9<=sample['voltage_v']<=12.6 or sample['temperature_c']>=50:
                raise RuntimeError('initial servo feedback outside bounds')
            speed=128 if ident['phase'] & 4 else 2
            probe.command(f'ARM {probe.servo_id} {centre} {speed}','armed');probe.active=True
            probe.write(40,b'\0')
            goal=lambda position: bytes((10,))+struct.pack('<HHH',position,0,speed)
            probe.write(41,goal(centre))
            probe.write(40,b'\1')
            rows+=probe.capture(.7,'hold_initial',centre)
            if args.mode=='motion':
                targets=[min(ident['angle_max'],centre+114),centre,max(ident['angle_min'],centre-114),centre]
                summary['motion']=dict(centre=centre,targets=targets,cycles=args.cycles,speed_raw=speed,acceleration_raw=10,
                    max_offset_counts=114,max_offset_degrees=114*360/4096)
                for cycle in range(args.cycles):
                    for index,target in enumerate(targets):
                        if not 0<=target<=4095 or abs(target-centre)>114:raise RuntimeError('target bound')
                        probe.write(41,goal(target));block=probe.capture(2.5,f'motion_{cycle}_{index}',target);rows+=block
                        if abs(block[-1]['position_counts']-target)>8:raise RuntimeError('target not reached')
                probe.command('X','disarmed');probe.active=False
            else:
                before_torque=probe.read(40,1)[0]['data'][0]
                if before_torque!=1:raise RuntimeError('watchdog precondition: torque was not enabled')
                silence_begin=time.monotonic();time.sleep(1.5)
                event=None;deadline=time.monotonic()+1
                while time.monotonic()<deadline:
                    raw=probe.serial.readline()
                    try: candidate=json.loads(raw)
                    except (ValueError,UnicodeDecodeError):continue
                    probe.log.write(json.dumps(dict(request=None,reply=candidate,
                        host_monotonic_ns=time.monotonic_ns()))+'\n');probe.log.flush()
                    if candidate.get('event')=='lease_expired':event=candidate;break
                if event is None:raise RuntimeError('watchdog expiry event missing')
                probe.active=False
                info=probe.command('INFO','info')
                if info['armed']:raise RuntimeError('watchdog remained armed')
                summary['watchdog']=dict(torque_before=before_torque,lease_ms=1000,
                    host_silence_seconds=time.monotonic()-silence_begin,
                    expiry_event_observed=True,same_mcu_boot=info['boot']==probe.boot,
                    physical_bus_disconnect_tested=False)
            summary['torque_after']=probe.read(40,1)[0]['data'][0]
            if summary['torque_after']!=0:raise RuntimeError('torque release unconfirmed')
            rows+=probe.capture(.5,'released')
        summary['config_after_hex']=probe.read(0,40)[0]['data'].hex()
        if summary['config_after_hex']!=summary['config_before_hex']:raise RuntimeError('configuration changed')
        summary['completed']=True
    except Exception as exc:
        summary['completed']=False;summary['error']=repr(exc)
        raise
    finally:
        try:probe.close()
        finally:
            summary.update(requests=probe.requests,response_timeouts=probe.timeouts,packet_errors=probe.bad,
                           feedback_samples=len(rows))
            if rows:
                summary['observed']={k:dict(min=min(r[k] for r in rows),max=max(r[k] for r in rows),
                    mean=statistics.fmean(r[k] for r in rows)) for k in ['position_counts','voltage_v',
                    'temperature_c','current_signed','host_roundtrip_ms']}
                summary['servo_status_values']=sorted({r['servo_status'] for r in rows})
            (args.output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
            print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--mode',choices=('read','motion','watchdog'),default='read')
    parser.add_argument('--seconds',type=float,default=10)
    parser.add_argument('--cycles',type=int,choices=range(1,6),default=1)
    parser.add_argument('--scan-all-ids',action='store_true')
    run(parser.parse_args())
