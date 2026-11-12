"""스택 수정 r305 비구동 앱을 확인한 뒤 시험 직전 app 영역을 복원한다."""
from pathlib import Path
import collections,gc,hashlib,json,struct,subprocess,sys,time
import serial
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_vehicle_interface'),str(R/'scripts')]
from arena_vehicle_interface.sensor_link_wire import SensorLinkDecoder
from arena_vehicle_interface.actuation_wire import Decoder
from probe_esp32_imu import Decoder as ImuDecoder

def main():
    out=A/'actual';out.mkdir(exist_ok=False)
    port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5031468-if00'
    firmware=A/'arena_runtime_preview.bin';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert sha(firmware)=='a63e2bc7d1e5f6f9774974f505820d0f027632e45b44c6c8fef5644ea56c9530'
    assert str(Path(port).resolve())=='/dev/ttyACM0'
    assert subprocess.run(['fuser','/dev/ttyACM0'],capture_output=True).returncode==1
    report=dict(actuation_commands=0,host_uart_transmit_bytes=0,flash_started=False,restored=False,scope='temporary r305 output_ready=false; raw PCNT not valid motion; UART1 read requests only')
    def command(label,*args):
        with (out/(label+'-private.log')).open('xb') as log:
            subprocess.run([sys.executable,'-m','esptool','--chip','esp32s3','--port',port,'--baud','921600',*args],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=90)
    def capture(path,seconds,mixed):
        ser=serial.Serial(port=None,baudrate=921600,timeout=.05,exclusive=True)
        ser.dtr=False;ser.rts=False;ser.port=port
        chunks=[];decoder=SensorLinkDecoder();ia=Decoder();imu=ImuDecoder();controls=[];imus=[]
        enabled=gc.isenabled()
        try:
            ser.open();start=time.monotonic();gc.disable()
            while time.monotonic()-start<seconds:
                chunk=ser.read(max(1,min(ser.in_waiting,8192)));chunks.append(chunk)
                for kind,packet in decoder.feed(chunk):
                    if kind=='imu':imus.extend(imu.feed(packet))
                    else:controls.extend(ia.feed(packet))
        finally:
            ser.close()
            if enabled:gc.enable()
        raw=b''.join(chunks);path.write_bytes(raw)
        observations=[x for x in controls if x.get('type')=='runtime_observation_v1']
        tags=collections.Counter(x.get('sensor_tag') for x in imus if x.get('type')==1)
        status=[x for x in imus if x.get('type')==2]
        result=dict(bytes=len(raw),raw_sha256=hashlib.sha256(raw).hexdigest(),seconds=time.monotonic()-start,
            mux=dict(decoder.counters),ia_errors=ia.errors,imu_counters=dict(imu.counters),imu_frames=len(imus),raw_tags=dict(tags),
            control_types=dict(collections.Counter(x.get('type') for x in controls)),raw_observations=len(observations),
            observations_first=observations[:2],observations_last=observations[-2:],imu_status_last=status[-2:],
            boot_ids=sorted(set(x['boot'] for x in imus)),imu_sequence_gaps=sum(b['sequence']!=a['sequence']+1 for a,b in zip(imus,imus[1:])),
            port_closed=not ser.is_open)
        if observations:
            stamps=[x['pcnt_end_us'] for x in observations]
            result['pcnt_report_gap_max_us']=max(b-a for a,b in zip(stamps,stamps[1:]))
            result['all_output_disabled']=all(x['output_ready'] is False and x['encoder_motion_verified'] is False for x in observations)
            result['main_stack_free_min_bytes']=min(x['main_stack_free_min_bytes'] for x in observations)
            result['maximum_tick_gap_us']=max(x['maximum_tick_gap_us'] for x in observations)
            result['gpio_levels_zero']=all(x['pwm_gpio_level']==0 and x['dir_gpio_level']==0 for x in observations)
            result['pcnt_minmax']=[min(x['pcnt_count'] for x in observations),max(x['pcnt_count'] for x in observations)]
        (path.with_suffix('.summary.json')).write_text(json.dumps(result,indent=2)+'\n')
        return result
    try:
        command('partition','read-flash','0x8000','0x1000',str(out/'partition-private.bin'))
        table=(out/'partition-private.bin').read_bytes();apps=[]
        for i in range(0,len(table),32):
            magic,kind,subtype,start,size,label,flags=struct.unpack('<HBBII16sI',table[i:i+32])
            if magic==0xffff:break
            if magic==0xebeb:continue
            assert magic==0x50aa
            if kind==0:apps.append((subtype,start,size,label.rstrip(b'\0').decode()))
        assert apps==[(0,0x10000,0x100000,'factory')],apps
        command('backup','read-flash','0x10000','0x100000',str(out/'before-app-private.bin'))
        report['before_app_sha256']=sha(out/'before-app-private.bin');report['partition_sha256']=sha(out/'partition-private.bin')
        report['flash_started']=True
        command('preview','write-flash','0x10000',str(firmware))
        command('preview_readback','read-flash','0x10000',str(firmware.stat().st_size),str(out/'preview-readback-private.bin'))
        assert sha(out/'preview-readback-private.bin')==sha(firmware)
        report['preview_sha256']=sha(firmware)
        report['capture']=capture(out/'runtime-private.bin',30,True)
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        raise
    finally:
        try:
            if report['flash_started']:
                command('restore','write-flash','0x10000',str(out/'before-app-private.bin'))
                command('restore_readback','read-flash','0x10000','0x100000',str(out/'restored-app-private.bin'))
                report['restored_app_sha256']=sha(out/'restored-app-private.bin')
                report['restored']=report['restored_app_sha256']==report['before_app_sha256']
                assert report['restored']
                report['after_restore']=capture(out/'restored-passive-private.bin',3,False)
            report['port_unowned']=subprocess.run(['fuser','/dev/ttyACM0'],capture_output=True).returncode==1
        finally:
            (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps({k:v for k,v in report.items() if k not in ('capture','after_restore')}),flush=True)

if __name__=='__main__':main()
