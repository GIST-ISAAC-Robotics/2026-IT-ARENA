"""기존 비구동 센서/저장 계산을 한 번 복제해 CPU·메모리·스왑을 측정한다."""
from pathlib import Path
import hashlib,json,os,signal,subprocess,time,tarfile

OLD=Path('/home/q/it-arena-r363-20261113')
ROOT=Path('/home/q/it-arena-r373b-20261113')
REL=Path('artifacts/validation/2026-11-13/camera_publish_r363_v1')

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((OLD/'source_manifest.json').read_bytes())
assert not ROOT.exists(),'one new bounded run only'
assert not [p for p,h in manifest.items() if digest(OLD/p)!=h]
for name in manifest:
    p=ROOT/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((OLD/name).read_bytes())
(ROOT/'source_manifest.json').write_bytes((OLD/'source_manifest.json').read_bytes())
out=ROOT/'resource_result';out.mkdir()

def metrics():
    mem={k:int(v.split()[0]) for line in Path('/proc/meminfo').read_text().splitlines() for k,v in [line.split(':',1)]}
    vm={s[0]:int(s[1]) for line in Path('/proc/vmstat').read_text().splitlines() if (s:=line.split())}
    cores={s[0]:[int(x) for x in s[1:9]] for line in Path('/proc/stat').read_text().splitlines()
           if (s:=line.split()) and s[0].startswith('cpu')}
    return dict(monotonic_s=time.monotonic(),mem_available_kib=mem['MemAvailable'],
                swap_used_kib=mem['SwapTotal']-mem['SwapFree'],pswpin=vm['pswpin'],pswpout=vm['pswpout'],
                cpu_ticks=cores)

begin=metrics();samples=[begin];child=None;stats=None;forced=[];cancel=False
def stop(*_):
    global cancel
    cancel=True
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
try:
    with (out/'bench.log').open('x') as log,(out/'tegrastats.log').open('x') as tlog:
        stats=subprocess.Popen(['tegrastats','--interval','1000'],stdout=tlog,stderr=subprocess.STDOUT)
        child=subprocess.Popen(['python3','-B',str(ROOT/REL/'bench.py')],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+125
        while child.poll() is None:
            if cancel or time.monotonic()>deadline:raise TimeoutError('cancel or bounded resource deadline')
            time.sleep(1);samples.append(metrics())
        assert child.returncode==0,'existing sensor/compute benchmark failed; retain raw result'
finally:
    if child is not None and child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=25)
        except subprocess.TimeoutExpired:
            forced.append('bench_parent');child.terminate();child.wait(timeout=5)
    if stats is not None:
        stats.terminate();stats.wait(timeout=5)
    samples.append(metrics())
    (out/'resources.json').write_text(json.dumps(samples,indent=2)+'\n')
    changed=[p for p,h in manifest.items() if digest(OLD/p)!=h or digest(ROOT/p)!=h]
    (out/'integrity.json').write_text(json.dumps(dict(changed_sources=changed,source_count=len(manifest),
        child_exit=child.returncode if child else None,forced=forced,
        old_root_unchanged=True if not changed else False,
        firmware_written=False,actuator_started=False,gui_or_power_changed=False),indent=2)+'\n')
    with tarfile.open('/home/q/r373b-resources.tar.gz','x:gz') as archive:
        archive.add(out,arcname='resources')
        actual=ROOT/REL/'actual'
        if actual.exists():archive.add(actual,arcname='actual')
print(json.dumps(dict(completed=True,samples=len(samples),
    min_available_mib=min(r['mem_available_kib'] for r in samples)/1024,
    max_swap_mib=max(r['swap_used_kib'] for r in samples)/1024,
    pswpin_delta=samples[-1]['pswpin']-samples[0]['pswpin'],pswpout_delta=samples[-1]['pswpout']-samples[0]['pswpout'])))
