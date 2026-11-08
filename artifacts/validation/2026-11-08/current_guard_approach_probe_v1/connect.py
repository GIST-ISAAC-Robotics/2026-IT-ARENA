"""Connect reviewed r48 candidates after preserved detached evidence passes."""
from pathlib import Path
import hashlib,json,zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3];P=A.parent/'traffic_corner_prefix_probe_v1'
OUT=A.parent/'observed_corner_continuity_r48_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert not OUT.exists()
assert (A/'tests_v3.exit').read_text().strip()=='0'
assert (A/'replay_v1.exit').read_text().strip()=='0'
assert (P/'tests_v2.exit').read_text().strip()=='0'
r=json.loads((A/'replay_summary.json').read_bytes())
assert r['calls']==1891 and r['source_unchanged'] and r['first_difference']['t']>=35.52
mapping={
 'src/arena_autonomy/arena_autonomy/local_follow.py':(P/'before_local_follow.py',P/'candidate_local_follow.py'),
 'src/arena_autonomy/arena_autonomy/pass_release_trial.py':(A/'before_selector.py',A/'candidate_selector.py')}
for name,(before,after) in mapping.items():assert (ROOT/name).read_bytes()==before.read_bytes(),name
OUT.mkdir()
with zipfile.ZipFile(OUT/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
    for name in mapping:z.write(ROOT/name,name)
    z.write(ROOT/'tests/test_source_provenance_r0_plus_b3.py','tests/test_source_provenance_r0_plus_b3.py')
changes={}
for name,(before,after) in mapping.items():
    (ROOT/name).write_bytes(after.read_bytes())
    changes[name]=dict(before=sha(before),after=sha(after),reason='Current observed corner prefix and independently observed guarded-side motion; no occupancy/TTL/default changes')
parent='config/tests/current_corner_fallback_r47_sha256.json';contract='config/tests/observed_corner_continuity_r48_sha256.json'
c=dict(schema_version=1,scope='짧은 현재 코너 인계와 현재 관측된 측방 이동. 초기 확보/TTL/점유/독립 보호/기본 OFF 유지.',
    parent_contract=parent,parent_contract_sha256=sha(ROOT/parent),
    before_snapshot=(OUT/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(OUT/'before.zip'),
    approved_changes=changes,approved_evaluation_scripts={},new_dependencies={})
(ROOT/contract).write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n',encoding='utf8',newline='\n')
p=ROOT/'tests/test_source_provenance_r0_plus_b3.py';s=p.read_text(encoding='utf8')
needle=f"M3_SUCCESSORS += ('{parent}',)";assert s.count(needle)==1
p.write_text(s.replace(needle,needle+f"\nM3_SUCCESSORS += ('{contract}',)"),encoding='utf8',newline='\n')
for folder,name,mod in ((P,'test_traffic_corner_prefix.py','local_follow'),(A,'test_observed_guard_approach.py','pass_release_trial')):
    s=(folder/'test_candidate.py').read_text(encoding='utf8').replace('from loader import load',f'import arena_autonomy.{mod} as m').replace('m=load()','')
    s=s.replace('A=Path(__file__).resolve().parent',f"A=Path(__file__).resolve().parents[1]/'{folder.relative_to(ROOT).as_posix()}'")
    (ROOT/'tests'/name).write_text(s,encoding='utf8',newline='\n')
OLD=A.parent/'corner_slowing_actor_v1';REG=A.parent/'current_corner_fallback_r47_v1'
for f in ('check_runtime.py','assess.py','new_track_course.json','evaluation_source.json','geometry.json','run.py','review.py'):
    (OUT/f).write_bytes((OLD/f).read_bytes())
s=(OLD/'run.sh').read_text(encoding='utf8').replace(OLD.name,OUT.name)
(OUT/'run.sh').write_text(s,encoding='utf8',newline='\n')
s=(REG/'tests.sh').read_text(encoding='utf8').replace(REG.name,OUT.name)
s=s.replace('tests/test_current_corner_fallback.py','tests/test_traffic_corner_prefix.py tests/test_observed_guard_approach.py tests/test_guard_curve_trial.py tests/test_boundary_bridge.py tests/test_current_corner_fallback.py')
(OUT/'tests.sh').write_text(s,encoding='utf8',newline='\n')
s=(REG/'replay.py').read_text(encoding='utf8').replace('current_corner_fallback_probe_v2/replay_rows.jsonl','current_guard_approach_probe_v1/replay_rows.jsonl').replace('retained_corner_agreement_r46_v1/run1/follow_decisions.jsonl','corner_slowing_actor_v1/run1/follow_decisions.jsonl')
(OUT/'replay.py').write_text(s,encoding='utf8',newline='\n')
s=(REG/'replay.sh').read_text(encoding='utf8').replace(REG.name,OUT.name)
(OUT/'replay.sh').write_text(s,encoding='utf8',newline='\n')
(OUT/'register.py').write_bytes((A/'register_template.py').read_bytes())
print('r48 connected. Run related tests and replay before unchanged slow-corner scene.')
