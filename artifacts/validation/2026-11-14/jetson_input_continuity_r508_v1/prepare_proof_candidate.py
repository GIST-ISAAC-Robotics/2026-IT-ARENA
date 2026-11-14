import hashlib,json,shutil,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent
C=A/'candidate_shm16_drain_proof'
C.mkdir(exist_ok=False)
for p in (A/'candidate_shm16_drain').iterdir():
    if p.is_file() and p.name!='manifest.json':shutil.copyfile(p,C/p.name)
original=A.parents[3]/'src/arena_autonomy/arena_autonomy/race_runtime_policy/current_route_priority.py'
assert hashlib.sha256(original.read_bytes()).hexdigest()=='23adb7767de1f64b4892a19956b33adae0dbb08a36cb9de86e8ff51891992651'
s=original.read_text(encoding='utf8')
old="    if result['speed_mps'] > 0 and not _front_observed(scene):\n        result.update(speed_mps=0., reason='current_route_front_observation_lost')\n    return result\n"
new="""    if result['speed_mps'] > 0 and not _front_observed(scene):
        result.update(speed_mps=0., reason='current_route_front_observation_lost')
    if result['speed_mps'] > 0:
        # 현재 기하/전체 점유 sweep과 전방 관측이 통과한 결과의 인계 계약.
        # 기존 원 경로 시각을 유지하며, 과거 점유 허가를 복사하지 않는다.
        result['current_route_proof'] = dict(policy=POLICY, accepted=True,
            original_path_source_s=result['path_source_s'],
            original_path_age_s=result['path_age_s'],
            current_evidence_first_s=result['source_first_s'],
            current_reference_s=result['reference_s'],
            source_timestamp_renewed=False, old_occupancy_permission_reused=False,
            unknown_filled=False, route_geometry_age_limit_s=.5,
            scope='current path and occupied sweep; no full-sweep visibility veto')
    return result
"""
assert s.count(old)==1
(C/'current_route_priority.py').write_text(s.replace(old,new),encoding='utf8')
bootstrap='''"""r500 파일 불변; 이 시험의 프로세스 안에서만 명시 후보를 읽는다."""
import hashlib
from pathlib import Path
from arena_autonomy.race_runtime_policy import current_route_priority as module
path=Path(__file__).with_name('current_route_priority.py')
assert hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()=='23adb7767de1f64b4892a19956b33adae0dbb08a36cb9de86e8ff51891992651'
exec(compile(path.read_bytes(),str(path),'exec'),module.__dict__)
CANDIDATE_SHA=hashlib.sha256(path.read_bytes()).hexdigest()
'''
(C/'proof_candidate_bootstrap.py').write_text(bootstrap,encoding='utf8')
p=C/'node_runner.py';s=p.read_text(encoding='utf8')
s=s.replace("PREFIX = '/replay_r508'", "from proof_candidate_bootstrap import CANDIDATE_SHA\n\nPREFIX = '/replay_r508'",1)
s=s.replace('summary.update(role=args.role,', 'summary.update(route_proof_candidate_sha256=CANDIDATE_SHA,role=args.role,',1)
p.write_text(s,encoding='utf8')
shutil.copyfile(A/'test_current_route_proof_candidate.py',C/'test_current_route_proof_candidate.py')
manifest=json.loads((A/'candidate_shm16_drain/manifest.json').read_bytes())
manifest.update(harness_revision=5,candidate='shm_16m + bounded motion drain + current priority proof handoff; no TTL change',
    original_priority_sha256='23adb7767de1f64b4892a19956b33adae0dbb08a36cb9de86e8ff51891992651')
manifest['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(C.iterdir()) if p.is_file()}
(C/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
for p in C.glob('*.py'):compile(p.read_bytes(),str(p),'exec')
with zipfile.ZipFile(A/'candidate_shm16_drain_proof.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in C.iterdir():z.write(p,p.name)
print(json.dumps(dict(files=len(manifest['files']),zip_sha256=hashlib.sha256((A/'candidate_shm16_drain_proof.zip').read_bytes()).hexdigest())))
