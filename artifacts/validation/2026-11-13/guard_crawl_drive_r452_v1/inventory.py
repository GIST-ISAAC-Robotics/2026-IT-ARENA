"""r441 archive 대비 실제 공통 delta를 전부 보존한다. 고정 파일 수를 가정하지 않는다."""
from pathlib import Path
import argparse,difflib,importlib.util,json,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
spec=importlib.util.spec_from_file_location('r452_prepare',A/'prepare.py');p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--phase',choices=('prebuild','postbuild'),required=True)
    parser.add_argument('--version',type=int,default=1);args=parser.parse_args()
    refs={
        'r439':'installed_limit_chain_r439_v1/integration.json',
        'r444':'estimated_observation_r444_v1/integration.json',
        'r446':'estimated_entry_contract_r446_v1/changes.json',
        'r449':'complete_plan_trace_r449_v1/integration.json',
        'r451':'guard_local_cap_r451_v1/integration.json'}
    hashes={};groups={};integrations={}
    for group,name in refs.items():
        path=A.parent/name;body=p.read(path);integrations[group]=dict(path=path.relative_to(ROOT).as_posix(),sha256=p.sha(path))
        raw=body.get('files',body.get('source_sha256'))
        if isinstance(raw,list):mapping={row['path']:row['sha256'] for row in raw}
        else:mapping={name:value if isinstance(value,str) else value['stage_sha256'] for name,value in raw.items()}
        for name,h in mapping.items():hashes[name]=h;groups.setdefault(name,[]).append(group)
    sim='src/arena_bringup/launch/simulation.launch.py';hashes[sim]=p.sha(ROOT/sim);groups[sim]=['r449_launch']
    current=p.common_sources();delta=p.source_delta(current)
    reasons={
        'r439':'설치 선택 상한 생성기/시험 추가. 이번 simulation은 실행하지 않음.',
        'r444':'명시 estimated observation 계약 추가. 이번 simulation의 원 verified 계약은 유지.',
        'r446':'estimated scope의 일반 base 입구 거절. 등록 G1 runtime 경로 유지.',
        'r449':'기본OFF 완전 plan 기록. 이번 진단만 명시 true.',
        'r451':'GUARD 원 상태/점유/1cm 한계에서 단일 .1 재검사와 연결/지원 회귀.',
        'r449_launch':'기본 false launch 인자 및 controller bool 전달 추가. 이번 진단 true만 선택.'}
    unknown=[];files={}
    for row in delta:
        name=row['path'];group=groups.get(name,[])
        if not group or current.get(name)!=hashes.get(name):unknown.append(dict(row,receipt_sha256=hashes.get(name),groups=group))
        files[name]={k:v for k,v in row.items() if k!='path'}
        files[name].update(groups=group,reason=' '.join(reasons[g] for g in group))
    review=dict(phase=args.phase,baseline=p.BASE.relative_to(ROOT).as_posix(),baseline_archive_sha256=p.sha(p.BASE/'sources.zip'),
        files=files,integrations=integrations,delta_count=len(delta),current_common_files=len(current),
        unclassified_or_hash_mismatched=unknown,selection_difference=['g1_complete_plan_trace:=true'],
        original_config_geometry_thresholds_preserved=p.static_contract(),execution_not_started=True)
    suffix='_v'+str(args.version)
    destination=A/('source_review_'+args.phase+suffix+'.json');p.write_new(destination,review)
    patch=[]
    with zipfile.ZipFile(p.BASE/'sources.zip') as z:
        for row in delta:
            name=row['path'];old=z.read(name) if name in z.namelist() else b'';new=(ROOT/name).read_bytes() if (ROOT/name).is_file() else b''
            try:
                old=old.decode('utf-8-sig');new=new.decode('utf-8-sig')
            except UnicodeDecodeError:patch.append('Binary source delta: '+name+'\n');continue
            patch.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='r441/'+name,tofile='r452/'+name))
    (A/('source_delta_'+args.phase+suffix+'.patch')).write_text(''.join(patch),encoding='utf8',newline='\n')
    print(json.dumps(dict(phase=args.phase,delta_count=len(delta),common_files=len(current),unknown=unknown,
        files=[dict(path=name,groups=v['groups']) for name,v in files.items()]),ensure_ascii=False))
    assert not unknown,'unknown delta requires root review; do not widen automatically'
    if args.phase=='postbuild':
        scope={k:review[k] for k in ('baseline','baseline_archive_sha256','files','integrations','selection_difference')}
        scope.update(status='preparer_exact_integration_hashes_verified_root_freeze_review_pending',execution_not_authorized=True)
        p.write_new(A/'source_scope.json',scope)
if __name__=='__main__':main()
