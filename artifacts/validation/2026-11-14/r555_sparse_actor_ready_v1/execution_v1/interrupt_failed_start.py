import datetime,json,os,signal
from pathlib import Path
A=Path(__file__).resolve().parent
pid=int((A/'run.pid').read_text())
proc=Path('/proc')/str(pid)
cmd=(proc/'cmdline').read_bytes().split(b'\0')
expected=[b'python3',b'artifacts/validation/2026-11-14/r555_sparse_actor_ready_v1/run.py']
assert cmd[:2]==expected,cmd[:2]
assert (proc/'cwd').resolve()==A.parents[4]
receipt=dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),pid=pid,verified_command=[x.decode() for x in cmd[:2]],signal='SIGINT',reason='OFF empty parameter parse failure before ROS/controller initialization; request existing validator finally cleanup',stop_service_proven=False)
os.kill(pid,signal.SIGINT)
(A/'interrupt_failed_start.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt))
