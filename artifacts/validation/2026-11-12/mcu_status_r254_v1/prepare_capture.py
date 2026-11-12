from pathlib import Path
import shutil

A=Path(__file__).resolve().parent;R=A.parents[3]
B=A.parent/'sensor_link_r239_v1'
bridge=(R/'scripts/bridge_sensor_link_preview.py').read_text('utf-8')
bridge=bridge.replace('from collections import Counter','from collections import Counter\nfrom status_clock import McuStatusClock')
bridge=bridge.replace("    clock=ImuReadClock();", "    mcu_clock=McuStatusClock()\n    mcu_status_pub=node.create_publisher(String,'/hardware_preview/actuation_status',10)\n    clock=ImuReadClock();")
bridge=bridge.replace('raw.write(data); gyros=[]','raw.write(data); gyros=[];mcu_states=[]')
needle="                            offer_boots.add(row['boot'])"
assert needle in bridge
bridge=bridge.replace(needle,needle+"\n                            mcu_states.append(row)")
needle="            measured('decode_record',begin);begin=time.perf_counter_ns()"
assert needle in bridge
bridge=bridge.replace(needle,"""            for item in mcu_clock.convert_batch(mcu_states,monotonic_ns=mono,realtime_ns=ros):
                mcu_status_pub.publish(String(data=json.dumps(item)))
                operations.write(json.dumps(dict(direction='status_published',host_ns=mono,payload=item))+'\\n')
                counts['mcu_status_published']+=1
"""+needle)
bridge=bridge.replace("tx_decode_errors=tx.errors,clock=dict(clock.counts)","tx_decode_errors=tx.errors,mcu_clock=dict(mcu_clock.counts),clock=dict(clock.counts)")
(A/'bridge.py').write_text(bridge,encoding='utf-8')

receiver=(B/'receiver.py').read_text('utf-8')
receiver=receiver.replace('from arena_autonomy.lidar_motion_ros import MotionInput','from arena_autonomy.lidar_motion_ros import MotionInput\nfrom arena_autonomy.race_session import McuObserver')
receiver=receiver.replace("(out/'receiver.ready').write_text('ready')","""observer=McuObserver();mcu_rows=[]
def mcu_status(message):
    payload=json.loads(message.data);now=node.get_clock().now().nanoseconds*1e-9;wall=time.monotonic()
    outcome=observer.update(payload,now,wall)
    mcu_rows.append(dict(payload=payload,received_s=now,outcome=outcome,fresh=observer.fresh(now,wall)))
node.create_subscription(String,'/hardware_preview/actuation_status',mcu_status,10)
(out/'receiver.ready').write_text('ready')""")
receiver=receiver.replace("    (out/'receiver_summary.json')", "    result['mcu_observer']=dict(counts=observer.counts,rows=len(mcu_rows),fresh=sum(r['fresh'] for r in mcu_rows),last=observer.latest)\n    with (out/'mcu_status_rows.jsonl').open('x') as stream:\n        for row in mcu_rows:stream.write(json.dumps(row)+'\\n')\n    (out/'receiver_summary.json')")
(A/'receiver.py').write_text(receiver,encoding='utf-8')
run=(B/'run.py').read_text('utf-8').replace("=='189'","=='218'").replace('sensor_link_r239','mcu_status_r254')
run=run.replace("str(root/'scripts/bridge_sensor_link_preview.py')","str(A/'bridge.py')")
run=run.replace("str(target),expected]", "str(target),expected,original]")
old="""        with (out/'restore.log').open('x') as log:
            subprocess.run([python,str(A/'flash.py'),str(remote/'restore'),original,old_sha],check=True,timeout=240,stdout=log,stderr=subprocess.STDOUT)
        result['original_imu_restored']=True"""
new="""        flash_summary=remote/'flash/restore_summary.json'
        wrote=flash_summary.exists() and json.loads(flash_summary.read_text()).get('app_write_started') is True
        if wrote:
            with (out/'restore.log').open('x') as log:
                subprocess.run([python,str(A/'flash.py'),str(remote/'restore'),original,old_sha],check=True,timeout=240,stdout=log,stderr=subprocess.STDOUT)
            result['original_imu_restored']=True
        else:
            result['restore_not_needed_no_write']=True"""
assert old in run
run=run.replace(old,new)
(A/'run.py').write_text(run,encoding='utf-8')
flash=(B/'flash.py').read_text('utf-8')
old="        command('flash', 'write-flash', '0x10000', str(TARGET))"
assert old in flash
flash=flash.replace(old,"""        if len(sys.argv)>4:
            current=Path(sys.argv[4]);before=(out/'before-app-private.bin').read_bytes()
            assert hashlib.sha256(before[:current.stat().st_size]).hexdigest()==sha(current), 'current app changed; preserve backup and do not flash'
        report['app_write_started']=True
"""+old)
(A/'flash.py').write_text(flash,encoding='utf-8')
(A/'.gitignore').write_text('run/\nbuild/*.bin\nbuild/symbols.txt\n',encoding='utf-8')
for p in A.glob('*.py'):compile(p.read_text('utf-8'),str(p),'exec')
print('bounded output-disabled status capture prepared')
