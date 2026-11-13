"""실센서 관측과 저장 입력 계산을 분리한 비구동 동시 부하. firmware 쓰기 없음."""
from pathlib import Path
import hashlib
import json
import os
import signal
import subprocess
import sys
import time

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
LOAD_ROOT=Path('/home/q/it-arena-r358-20261113')
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
IMU='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5031468-if00'
C1='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_362acd64b231f11190a78f5f30d20014-if00-port0'


def passive(out,name):
    import serial
    from probe_esp32_imu import Decoder
    decoder=Decoder(); rows=[]
    ser=serial.Serial(port=None,baudrate=921600,timeout=.05,exclusive=True)
    ser.dtr=False;ser.rts=False;ser.port=IMU
    with (out/(name+'-uart-private.bin')).open('xb') as log:
        try:
            ser.open();begin=time.monotonic()
            while time.monotonic()-begin<3.:
                data=ser.read(max(1,min(8192,ser.in_waiting)));log.write(data)
                rows.extend(decoder.feed(data))
        finally:ser.close()
    metadata=[r for r in rows if r['type']==3]
    status=[r for r in rows if r['type']==2]
    result=dict(frames=len(rows),metadata_last=metadata[-1:] if metadata else [],
                status_last=status[-1:] if status else [],decoder=dict(decoder.counters),
                uart_commands_written=0,closed=not ser.is_open)
    (out/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    assert metadata and metadata[-1]['who_am_i']==0x6c
    assert metadata[-1]['memory_errors']==metadata[-1]['spi_errors']==0
    assert status and status[-1]['active'] is False
    return result


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='216' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    old=json.loads((LOAD_ROOT/'artifacts/validation/2026-11-13/shared_ray_timed_r358_v1/source_manifest.json').read_bytes())
    extra=json.loads((ROOT/'source_manifest.json').read_bytes())
    def changed():
        return [str(base/n) for base,manifest in ((LOAD_ROOT,old),(ROOT,extra)) for n,h in manifest.items()
            if not (base/n).is_file() or hashlib.sha256((base/n).read_bytes()).hexdigest()!=h]
    assert not changed()
    assert subprocess.run(['fuser',IMU,C1],capture_output=True).returncode==1
    out=A/'actual';out.mkdir(exist_ok=False)
    events=[];children={};logs=[];forced=[];fault=None;interrupted=False
    def record(event,**kw):
        row=dict(event=event,monotonic_s=time.monotonic(),wall_ns=time.time_ns(),**kw)
        events.append(row);(out/'timeline.json').write_text(json.dumps(events,indent=2)+'\n')
    def stop(*_):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def start(name,cmd,cwd=ROOT):
        stream=(out/(name+'.log')).open('x');logs.append(stream)
        env=dict(os.environ)
        inherited=[p for p in os.environ.get('PYTHONPATH','').split(':') if p and not p.startswith((str(ROOT)+'/',str(LOAD_ROOT)+'/'))]
        env['PYTHONPATH']=':'.join([str(cwd/'src/arena_autonomy'),str(cwd/'src/arena_vehicle_interface'),str(cwd/'scripts'),*inherited])
        children[name]=subprocess.Popen(cmd,cwd=cwd,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        record(name+'_started',pid=children[name].pid,argv=cmd)
        return children[name]
    try:
        passive(out,'before');record('identity_checked')
        live=start('live',[sys.executable,'-B',str(ROOT/'scripts/run_hardware_sensor_preview.py'),
            '--output',str(out/'live'),'--seconds','20','--camera','--camera-publish-hz','20','--imu-source','probe',
            '--imu-port',IMU,'--c1-port',C1,
            '--c1-executable','/home/q/it_arena_c1_preview_r204_20261112/stream_c1_native_preview',
            '--c1-sha256','d8f7d2a02ef47786022206e2a0278c6e7edc2d3ddb40bed2254706b590748e2d'])
        ready=out/'live/imu/clock_estimates.jsonl';deadline=time.monotonic()+35
        while not (ready.exists() and ready.stat().st_size):
            if interrupted:raise InterruptedError('requested stop')
            if live.poll() is not None or time.monotonic()>deadline:raise RuntimeError('live input not ready')
            time.sleep(.1)
        record('live_imu_data_observed')
        load=start('calculation',[sys.executable,'-B',str(LOAD_ROOT/'artifacts/validation/2026-11-13/shared_ray_timed_r358_v1/timed_replay.py'),
            '--classification','shared','--policy','deadline','--mode','delay20','--submission','latest',
            '--duration','31','--plan-ms','0','--current-ms','0','--out',str(out/'calculation'),
            '--trace','artifacts/validation/2026-11-13/final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl'],cwd=LOAD_ROOT)
        deadline=time.monotonic()+70;ended=set()
        while len(ended)<len(children):
            if interrupted:raise InterruptedError('requested stop')
            for name,p in children.items():
                code=p.poll()
                if code is not None and name not in ended:
                    ended.add(name);record(name+'_ended',exitcode=code)
                    if code:raise RuntimeError(name+' failed: '+str(code))
            if time.monotonic()>deadline:raise TimeoutError('bounded overlap deadline')
            time.sleep(.1)
    except BaseException as exc:
        fault=type(exc).__name__+': '+str(exc)
        raise
    finally:
        for name,p in reversed(list(children.items())):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=18.)
                except subprocess.TimeoutExpired:
                    forced.append(name);os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5.)
            record(name+'_reaped',exitcode=p.returncode)
        for log in logs:log.close()
        free=subprocess.run(['fuser',IMU,C1],capture_output=True).returncode==1
        after_error=None
        if free:
            try:passive(out,'after')
            except Exception as exc:after_error=type(exc).__name__+': '+str(exc)
        else:after_error='ports not free'
        report=dict(fault=fault,after_error=after_error,forced=forced,
                    children={n:p.returncode for n,p in children.items()},ports_free=free,
                    changed_sources=changed(),source_count=len(old)+len(extra),
                    actuator_started=False,firmware_written=False,power_changed=False,
                    live_inputs_used_for_file_plan=False,synthetic_live_encoder=False)
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    assert not fault and not after_error and free and not forced and not changed()


if __name__=='__main__':main()
