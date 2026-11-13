"""r452 + 승인된 r459 정확한6파일 차이와 actor0 adapter를 동결한다. 실행 없음."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import zipfile
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'guard_crawl_drive_r452_v1'
POLICY=A.parent/'bounded_guard_approach_r459_v1'
SOURCE_SUFFIXES={'.py','.yaml','.xml','.xacro','.sdf','.cpp','.hpp','.stl','.so'}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_bytes())
def write_new(p,v):
    p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x',encoding='utf8') as f:json.dump(v,f,indent=2,ensure_ascii=False);f.write('\n')

def arguments():
    args=list(read(BASE/'registered.json')['args'])
    i=args.index('--actor-overlay');del args[i:i+2]
    for flag,value in {'--case':'solo_course_lap','--config':(A/'config.json').relative_to(ROOT).as_posix(),
                       '--output':(A/'run1').relative_to(ROOT).as_posix()}.items():args[args.index(flag)+1]=value
    args[0]=(A/'solo_runner.py').relative_to(ROOT).as_posix()
    return args

def verify_common():
    old=read(BASE/'prepared.json');assert sha(BASE/'sources.zip')==old['source_archive_sha256']
    baseline={p:h for p,h in old['frozen_inputs'].items() if p.startswith(('src/','scripts/','tests/','config/'))}
    changes=read(POLICY/'changes.json');integration=read(POLICY/'integration.json')
    assert integration['integrated'] is True and len(changes)==6
    expected={r['path']:r for r in changes}
    assert {r['path']:(r['before'],r['after']) for r in integration['files']}=={p:(r['original_sha256'],r['stage_sha256']) for p,r in expected.items()}
    assert sum(p.startswith('src/') for p in expected)==5 and sum(p.startswith('tests/') for p in expected)==1
    for p,r in expected.items():assert baseline.get(p)==r['original_sha256'],p
    paths={ROOT/p for p in baseline}
    paths.update(p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in SOURCE_SUFFIXES)
    paths.update((ROOT/'scripts').glob('*.py'));paths.update((ROOT/'tests').rglob('*.py'))
    current={p.relative_to(ROOT).as_posix():sha(p) for p in sorted(paths) if p.is_file()}
    delta={p:(baseline.get(p),current.get(p)) for p in baseline.keys()|current.keys() if baseline.get(p)!=current.get(p)}
    assert delta=={p:(r['original_sha256'],r['stage_sha256']) for p,r in expected.items()},delta
    return current

def freeze():
    assert not any((A/n).exists() for n in ('sources.zip','prepared.json','registered.json','run1'))
    common=verify_common();test=read(A/'preflight_v1/result.json');build=read(A/'build_v1/receipt.json')
    assert test['passed'] and build['build_executed'] and build['packages']==['arena_autonomy']
    assert build['build_exitcode']==build['import_cli_exitcode']==0 and build['native_import_verified']
    assert build['common_sha256']==common
    for p in [*A.glob('*.py'),*(A/'launch').glob('*.py'),*(A/'execution_v1').glob('*.py')]:ast.parse(p.read_bytes())
    selected=dict(common)
    old=read(BASE/'prepared.json')
    for p,h in old['frozen_inputs'].items():
        if 'old_signal_610_r326_v1/' in p or p.startswith('config/'):
            assert sha(ROOT/p)==h;selected[p]=h
    selected.update({p.relative_to(ROOT).as_posix():sha(p) for p in A.rglob('*') if p.is_file()
                    and p.suffix in ('.py','.sh','.json','.md','.sdf','.patch') and 'original' not in p.parts
                    and p.name not in ('session_cleanup.json','freeze.json','check_after_freeze.json')})
    for name in ('changes.json','integration.json'):selected[(POLICY/name).relative_to(ROOT).as_posix()]=sha(POLICY/name)
    selected[(BASE/'sources.zip').relative_to(ROOT).as_posix()]=sha(BASE/'sources.zip')
    # The old archive is referenced and hash frozen, not nested again in the new archive.
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in selected:
            if not p.endswith('/sources.zip'):z.write(ROOT/p,p)
    value=dict(args=arguments(),frozen_inputs=selected,source_archive_sha256=sha(A/'sources.zip'),
        baseline_archive_sha256=sha(BASE/'sources.zip'),common_delta=read(POLICY/'changes.json'),common_files=len(common),
        actor_count=0,facilities=3,real_time_factor=.25,max_sim_seconds=80,no_progress_stop_sim_seconds=8,
        source_policy='r452_plus_r459_exact_six_files',new_policy_candidate=True,root_run_confirmation_required=True,
        header_selection_unchanged=True,r455_integrated=False,
        one_execution_only=True,repeat_on_failure=False,simulator_started=False,build_executed=True)
    write_new(A/'prepared.json',value)
    return dict(prepared=True,prepared_sha256=sha(A/'prepared.json'),source_archive_sha256=value['source_archive_sha256'],
                files=len(selected),common_files=len(common),common_delta=6,simulation_started=False)

def verify_prepared():
    p=read(A/'prepared.json');changed=[k for k,h in p['frozen_inputs'].items() if not (ROOT/k).is_file() or sha(ROOT/k)!=h]
    assert not changed,changed
    assert sha(A/'sources.zip')==p['source_archive_sha256'];return p

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--freeze',action='store_true');p.add_argument('--check',action='store_true');o=p.parse_args()
    print(json.dumps(freeze() if o.freeze else dict(checked=True,files=len(verify_prepared()['frozen_inputs'])) if o.check else dict(common_files=len(verify_common()),common_delta=6)))
