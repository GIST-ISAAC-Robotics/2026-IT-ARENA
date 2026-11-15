#!/usr/bin/env python3
"""격리 모터 벤치: 포트 하나를 유지하며 명시한 단발 구동만 실행한다."""
import argparse
import collections
import json
import re
from pathlib import Path
import time
import zlib


def packet(body):
    b=body.encode('ascii')
    return f'{zlib.crc32(b):08x} '.encode()+b+b'\n'


def decode_line(line, allow_boot_prefix=False):
    prefix=0
    if allow_boot_prefix:
        match=re.search(rb'[0-9a-fA-F]{8} \{',line)
        if match:prefix=match.start();line=line[prefix:]
    if line[8:9]!=b' ':raise ValueError('framing')
    if int(line[:8],16)!=zlib.crc32(line[9:]):raise ValueError('CRC')
    return json.loads(line[9:]),prefix


def validate_pulse(firmware, duty, ms, observe_guard=False):
    if type(duty) is not int or type(ms) is not int:
        raise ValueError('integer pulse bounds required')
    if observe_guard:
        limit={'motor_probe_guarded_5s_v1':50,'motor_probe_start15_5s_v1':150}.get(firmware,0)
        if not 1<=duty<=limit or not 501<=ms<=5000:
            raise ValueError('guarded host pulse bounds')
    elif not 1<=duty<=100 or not 1<=ms<=500:
        raise ValueError('host pulse bounds')


def telemetry_fresh(motor, encoder, now_ns=None, max_age_s=1.0):
    now_ns=time.monotonic_ns() if now_ns is None else now_ns
    return all(isinstance(frame,dict) and 'host_ns' in frame and
               0<=now_ns-frame['host_ns']<=max_age_s*1e9 for frame in (motor,encoder))


class Bench:
    def __init__(self, port, output, firmware='motor_probe_v1'):
        import serial
        self.output=output;output.mkdir(parents=True,exist_ok=False)
        self.raw=(output/'uart-private.bin').open('wb')
        self.log=(output/'frames.jsonl').open('w',encoding='utf-8')
        self.commands=(output/'commands.jsonl').open('w',encoding='utf-8')
        self.port=serial.Serial(port=None,baudrate=921600,timeout=.02,write_timeout=.2,exclusive=True)
        self.port.dtr=False;self.port.rts=False;self.port.port=port;self.port.open()
        self.buffer=bytearray();self.frames=[];self.errors=collections.Counter()
        self.boot=None;self.seq=None;self.info=None;self.motor=None;self.encoder=None
        self.id=0;self.phase='startup';self.replies={};self.last_state=0
        self.firmware=firmware

    def send(self, data):
        self.commands.write(json.dumps({'host_ns':time.monotonic_ns(),'phase':self.phase,
                                       'wire':data.decode('ascii')})+'\n');self.commands.flush()
        if self.port.write(data)!=len(data):raise RuntimeError('short UART write; no retry')
        self.port.flush()

    def receive(self):
        data=self.port.read(max(1,min(self.port.in_waiting,16384)))
        self.raw.write(data);self.buffer.extend(data)
        while b'\n' in self.buffer:
            line,_,self.buffer=self.buffer.partition(b'\n')
            try:
                r,prefix=decode_line(line,allow_boot_prefix=self.info is None)
                if prefix:self.errors['boot_prefix_bytes']+=prefix
            except (ValueError,UnicodeDecodeError) as exc:
                self.errors['unparsed_line']+=1
                if self.info:raise RuntimeError('bad frame after identity') from exc
                continue
            if self.boot is None:self.boot=r['boot']
            elif self.boot!=r['boot']:raise RuntimeError('MCU boot changed; stop, no retry')
            if self.seq is not None and (r['seq']-self.seq)&0xffffffff!=1:
                self.errors['sequence_gap']+=1;raise RuntimeError('sequence gap')
            self.seq=r['seq'];r.update(host_ns=time.monotonic_ns(),phase=self.phase)
            self.frames.append(r);self.log.write(json.dumps(r)+'\n')
            event=r['event']
            if event=='info':
                if r['firmware']!=self.firmware:raise RuntimeError('wrong firmware')
                self.info=r
            elif event=='motor':self.motor=r;self.id=max(self.id,r['last_id'])
            elif event=='status':self.encoder=r
            elif event=='reply':self.replies[r['id']]=r
        if len(self.buffer)>16384:raise RuntimeError('oversize UART line')
        self.log.flush();self.raw.flush()
        if time.monotonic()-self.last_state>.2:
            self.save('state.json',{'phase':self.phase,'motor':self.motor,'encoder':self.encoder,
                                    'errors':dict(self.errors),'port_open':self.port.is_open,
                                    'telemetry_fresh':telemetry_fresh(self.motor,self.encoder)})
            self.last_state=time.monotonic()

    def save(self,name,data):
        p=self.output/name;t=p.with_suffix(p.suffix+'.tmp')
        t.write_text(json.dumps(data,indent=2),encoding='utf-8');t.replace(p)

    def wait(self,predicate,seconds=2):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            self.receive()
            if predicate():return
        raise RuntimeError('response timeout; no automatic retry')

    def collect(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.receive()

    def request(self,verb,tail='',boot=None):
        self.id+=1;rid=self.id
        b=self.boot if boot is None else boot
        self.send(packet(f'{verb} {b} {rid}'+(' '+tail if tail else '')))
        self.wait(lambda:rid in self.replies)
        return self.replies[rid]

    def arm(self):
        r=self.request('ARM')
        if r['result']!='armed':raise RuntimeError('arm rejected: '+r['result'])
        return r['token']

    def off(self):
        before=self.seq;self.send(b'X\n')
        self.wait(lambda:self.motor is not None and self.motor['seq']>before
                  and not self.motor['active'] and not self.motor['armed']
                  and self.motor['reason']=='host_stop' and self.motor['pwm_pin']==0)

    def pulse(self,duty,ms,*,observe_guard=False):
        validate_pulse(self.firmware,duty,ms,observe_guard)
        self.collect(.1)
        if not telemetry_fresh(self.motor,self.encoder):
            raise RuntimeError('stale MCU telemetry; no ARM or PULSE sent')
        before=dict(self.encoder);count=self.motor['pulses']
        token=self.arm();r=self.request('PULSE',f'{token} {duty} {ms}')
        if r['result']!='started':raise RuntimeError('pulse rejected: '+r['result'])
        self.wait(lambda:self.motor['pulses']==count+1 and not self.motor['active']
                  and (observe_guard or self.motor['reason']=='deadline'),seconds=ms/1000+1)
        ended=dict(self.motor);self.collect(1)
        after=dict(self.encoder)
        late=ended['ended_us']-ended['deadline_us']
        if ended['reason']=='deadline' and (late<0 or late>20000):raise RuntimeError('cutoff time outside bench bound')
        if self.motor['pwm_pin']!=0 or self.motor['duty_permille']!=0:
            raise RuntimeError('PWM idle readback not LOW')
        fields=('pcnt_count','software_count','edges','a_rise','a_fall','b_rise','b_fall','invalid','ring_dropped')
        return dict(duty_permille=duty,duration_ms=ms,cutoff_lateness_us=late,
                    output_end=ended,encoder_delta={k:after[k]-before[k] for k in fields},
                    before=before,after=after)

    def preflight(self):
        self.phase='motor_power_disconnected_preflight';checks=[]
        def expect(r,result):
            if r['result']!=result:raise RuntimeError(f'expected {result}, got {r}')
            checks.append(result)
        expect(self.request('PULSE','1 50 250'),'not_armed')
        expect(self.request('ARM',boot=self.boot^1),'wrong_boot')
        token=self.arm();expect(self.request('PULSE',f'{token} 101 250'),'out_of_range')
        token=self.arm();expect(self.request('PULSE',f'{token} 50 501'),'out_of_range')
        token=self.arm();self.collect(2.1)
        expect(self.request('PULSE',f'{token} 50 250'),'not_armed')
        token=self.arm();self.replies.pop(0,None)
        self.send(b'00000000 PULSE 0 0 0 50 250\n')
        self.wait(lambda:0 in self.replies);expect(self.replies[0],'bad_crc')
        self.replies.pop(0,None);self.send(b'1234');self.wait(lambda:0 in self.replies)
        expect(self.replies[0],'partial_timeout')
        self.send(b'S\n');self.collect(.1)
        trial=self.pulse(50,250);checks.append('deadline_cutoff')
        # 이미 소비한 허가와 요청 ID를 다시 보내도 출력을 재개하지 않는다.
        last=self.id;self.replies.pop(last,None)
        self.send(packet(f'PULSE {self.boot} {last} {token} 50 250'))
        self.wait(lambda:last in self.replies);expect(self.replies[last],'old_id')
        self.collect(3.1)
        token=self.arm();expect(self.request('PULSE',f'{token} 50 500'),'started')
        self.collect(.08);self.off();checks.append('explicit_stop')
        self.collect(3.1)
        self.send(b'S\n');self.collect(.1)
        self.phase='ready_motor_power_disconnected'
        result={'passed':True,'checks':checks,'deadline_trial':trial,
                'physical_motor_power':'disconnected_user_confirmed','physical_braking_verified':False}
        self.save('preflight.json',result);print('PREFLIGHT_PASSED_PORT_REMAINS_OPEN',flush=True)
        return result

    def close(self):
        stop_confirmed=False
        try:
            self.off()
            stop_confirmed=True
        finally:
            self.port.close();self.log.close();self.raw.close();self.commands.close()
            self.save('closed.json',{'port_closed':True,'stop_confirmed':stop_confirmed,
                                    'last_motor':self.motor,'errors':dict(self.errors)})


def main(args):
    if not args.motor_power_disconnected:raise ValueError('startup requires power disconnected')
    b=Bench(args.port,args.output);seen=set();start=time.monotonic()
    try:
        b.send(b'?\n');b.wait(lambda:b.info is not None and b.motor is not None and b.encoder is not None,4)
        b.off();b.preflight()
        while time.monotonic()-start<args.seconds:
            b.receive()
            if (args.output/'STOP').exists():break
            p=args.output/'request.json'
            if not p.exists():continue
            try:r=json.loads(p.read_text())
            except (OSError,ValueError):continue
            rid=r['request_id']
            if rid in seen:continue
            seen.add(rid);action=r['action']
            b.phase=r.get('phase','manual_request')
            if action=='stop':b.off();break
            if action=='pulse':
                result=b.pulse(int(r['duty_permille']),int(r['duration_ms']))
                b.save('result-'+str(rid)+'.json',result)
                print('PULSE_FINISHED '+json.dumps({k:result[k] for k in
                      ('duty_permille','duration_ms','cutoff_lateness_us','encoder_delta')}),flush=True)
                b.phase='idle_after_pulse'
            else:raise ValueError('unsupported action')
    except BaseException as exc:
        b.save('error.json',{'error':repr(exc),'phase':b.phase});raise
    finally:b.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--seconds',type=float,default=1200)
    p.add_argument('--motor-power-disconnected',action='store_true')
    main(p.parse_args())
