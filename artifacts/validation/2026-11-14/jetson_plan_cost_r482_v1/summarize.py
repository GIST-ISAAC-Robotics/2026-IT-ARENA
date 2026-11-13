"""회수된 원 결과에서 요약/불변 receipt 생성. 장치 실행 없음."""
from pathlib import Path
import hashlib,json,math,statistics,shutil
A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def stats(values):
    ordered=sorted(values);where=(len(ordered)-1)*.95;lo=math.floor(where);hi=math.ceil(where)
    return dict(count=len(values),median_ms=statistics.median(values),
                p95_ms=ordered[lo]+(ordered[hi]-ordered[lo])*(where-lo),max_ms=max(values),
                over_100ms=sum(x>100 for x in values))
M=json.loads((A/'compare_manifest.json').read_bytes())
O=A/'compare_result/compare_results'
all_data={v:[json.loads((O/f'{v}_{i}.json').read_bytes()) for i in range(3)] for v in ('base','stage')}
keys=[x['key'] for x in all_data['base'][0]['rows']]
cases=[]
for index,key in enumerate(keys):
    row=dict(key=key)
    for version,records in all_data.items():
        values=[r['rows'][index]['wall_ms'] for r in records]
        row[version]=dict(**stats(values),samples_ms=values)
    row['median_reduction_pct']=(1-row['stage']['median_ms']/row['base']['median_ms'])*100
    cases.append(row)
aggregate={v:stats([r['wall_ms'] for d in records for r in d['rows']]) for v,records in all_data.items()}
summary=dict(cases=cases,selected_eight_aggregate=aggregate,
    percentile='linear interpolation, not nearest rank; n=3 per case and n=24 selected calls per version',
    representative_selection='six r476 static_side_current_lane slow rows plus before/after normal rows',
    original_trace_plan_count=217,full_trace_replayed=False,
    timing_excludes=['constructor/import','state restoration','profile warmup','interprocess startup'],
    timing_includes='full Evaluator.__call__ including original freeze/replay work; first timed call per fresh process may include lazy state',
    matrix_calls=48,profile_warmup_calls=2,
    all_48_raw_exact=all(r['raw_exact'] for records in all_data.values() for d in records for r in d['rows']),
    all_2_warmup_raw_exact=all(records[0]['warmup']['raw_exact'] for records in all_data.values()),
    equality_scope='whole nested JSON output exact, including speed/steering/region counts/padding/occupancy details; no tolerance relaxation',
    peak_rss_kib={v:[r['peak_rss_kib'] for r in records] for v,records in all_data.items()},
    pytest=dict(passed=20,deselected=1,excluded='existing launch-argument/header-file test; no launch/config/scripts change'),
    previous_failed_profile=dict(prefix_calls=88,profile_calls=0,reason='dynamic class standard pickle failure',preserved='profile_result_v1'),
    profile_v2=dict(prefix_calls=98,profiled_in_prefix=8,restored_unprofiled_check=8,all_raw_exact=True),
    diagnostic_calls=2,normal_execution=True,owned_pid_remaining=0,jetson_returned=True,
    common_integration=False,hardware_timing_test=False,ros_test=False)
(A/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
source_paths=[p for p in M['files'] if p.startswith('base/src/')]
local_changes=[p for p,h in M['files'].items() if sha(A/p)!=h]
common_delta=[p[5:] for p in source_paths if not (ROOT/p[5:]).is_file() or sha(ROOT/p[5:])!=M['files'][p]]
candidate='src/arena_autonomy/arena_autonomy/current_lane_preference.py'
receipt=dict(base_source_files=len(source_paths),local_manifest_changes=local_changes,
    current_common_vs_frozen_source_delta=common_delta,
    candidate_files=[dict(path=candidate,before_sha256=sha(A/'base'/candidate),stage_sha256=sha(A/'stage'/candidate),
        common_sha256_at_review=sha(ROOT/candidate))],
    native_reuse=M['native_reuse'],
    archives={p.name:sha(p) for p in A.glob('*results*.zip')},
    comparison_execution_sha256=sha(A/'compare_result/compare_execution.json'),
    common_tools_outside_frozen_scope='root independently integrated Wi-Fi helpers and optional jetson_access stdin_payload; this worker changed no common file')
(A/'source_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
(A/'tests').mkdir(exist_ok=True)
shutil.copy2(A/'stage/tests/test_current_lane_region_reuse.py',A/'tests/test_current_lane_region_reuse.py')
print(json.dumps(dict(aggregate=aggregate,source_delta=common_delta,local_changes=local_changes),indent=2))
