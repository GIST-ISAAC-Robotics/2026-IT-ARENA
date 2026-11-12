"""보존한 r149의 대표 호출 비용 분해. 판단 로직과 상태 갱신은 변경하지 않는다."""
import cProfile,importlib.util,io,json,pstats,sys,hashlib
from pathlib import Path
root=Path(sys.argv[1]);a=root/'artifacts/validation/2026-11-12/adjacent_cost_r150_v1'
spec=importlib.util.spec_from_file_location('measure',a/'measure.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
events=[json.loads(line) for line in (a.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl').open()]
expected={r['seq']:r['output'] for r in map(json.loads,(a/'candidate_expected.jsonl').open())}
m.candidate.install();owner,motion=m.make_controller(events[0]['data']);prof=cProfile.Profile();chosen={402,416,605,606,642};calls=0;rows=[];bad=[]
for row in events[1:-1]:
    if row['kind']=='reset':owner.reset()
    if row['kind']!='call':continue
    calls+=1
    if row['seq'] in chosen:prof.enable()
    output,cost=m.timed(owner,row['data']['input'],motion)
    if row['seq'] in chosen:
        prof.disable();rows.append({'seq':row['seq'],'cost_profiled':cost})
    output=m.freeze(output)
    if not m.equal_output(output[:2],expected[row['seq']][:2]):bad.append(row['seq'])
    if len(rows)==len(chosen):break
stream=io.StringIO();stats=pstats.Stats(prof,stream=stream).sort_stats('cumulative');stats.print_stats(45)
(a/'jetson_guard_pipeline_profile.txt').write_text(stream.getvalue())
manifest=json.loads((a/'bundle_manifest.json').read_bytes())
result={'calls':calls,'profiled':rows,'command_mismatches':bad,'source_unchanged':all(hashlib.sha256((root/name).read_bytes()).hexdigest()==v['sha256'] for name,v in manifest['files'].items()),'scope':'5 representative calls with cProfile overhead; not a timing benchmark or full trace replay'}
(a/'jetson_guard_pipeline_profile.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
