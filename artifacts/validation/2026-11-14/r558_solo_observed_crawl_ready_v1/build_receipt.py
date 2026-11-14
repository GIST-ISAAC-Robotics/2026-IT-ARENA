"""승인된 단일 패키지 빌드 전후 입력과 경쟁 상태만 기록한다."""
import datetime,hashlib,json,os,sys
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_bytes())
def main():
    selected=read(A/'source_selection.json');assert selected['native_rebuild_authorized'] and selected['wsl_exclusive_owner']
    if sys.argv[1]=='before':
        competing=[]
        for p in Path('/proc').glob('[0-9]*'):
            if int(p.name) in (os.getpid(),os.getppid()):continue
            try:
                exe=(p/'exe').resolve().name;env=(p/'environ').read_bytes().split(b'\0');args=(p/'cmdline').read_bytes().split(b'\0')
                if exe in ('bash','sh','dash'):continue
                if any(v.startswith(b'GZ_PARTITION=arena_autonomy_test_') for v in env) or exe in ('gzserver','gzclient','gz','ruby') or any(v in (b'pytest',b'colcon') for v in args):competing.append(dict(pid=int(p.name),exe=exe))
            except (FileNotFoundError,PermissionError,ProcessLookupError):pass
        assert not competing,competing
        changed=[p for p,h in selected['required_sources'].items() if sha(ROOT/p)!=h];assert not changed,changed
        result=dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),competing_work=competing,changed_inputs=changed,
            native_sha256=sha(ROOT/'src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'))
        name='build_before.json'
    else:
        changed=[p for p,h in selected['required_sources'].items() if sha(ROOT/p)!=h]
        result=dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),exit_code=int(sys.argv[2]),changed_inputs=changed,
            native_sha256=sha(ROOT/'src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'),
            build_scope='colcon arena_autonomy only, symlink-install',passed=int(sys.argv[2])==0 and not changed)
        name='build_result.json'
    with (A/name).open('x',encoding='utf-8') as f:json.dump(result,f,indent=2);f.write('\n')
    if sys.argv[1]=='after':assert result['passed'],result
    print(json.dumps(result))
if __name__=='__main__':main()
