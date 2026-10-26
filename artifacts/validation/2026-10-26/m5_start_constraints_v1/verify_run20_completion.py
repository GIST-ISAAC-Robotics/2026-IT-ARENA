"""사용자 중단 직전 산출물의 계산 완료 여부만 확인. 제어기/운동 재실행 없음."""
import hashlib
import json
from pathlib import Path
import zipfile

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
RUN=BASE/'run20_straight_dynamics'
paths=[RUN/'data/report.json',RUN/'data/summary.json',RUN/'data/sources.zip',RUN/'exit_code.txt',RUN/'end.txt',
       BASE/'run13_observed_speed/data/report.json']
raw={p:p.read_bytes() for p in paths}
report=json.loads(raw[paths[0]]); summary=json.loads(raw[paths[1]])
prior=json.loads(raw[paths[-1]])['cases'][0]
keys=['t_s','frame','fault','explicit_stop','measured_mps','request_mps','request_steering',
      'safe_mps','safe_steering','reason','safety_reason','metadata','truth_pose','truth_travel_m']
same=len(prior['calls'])==len(report['cases'][0]['calls']) and all(
    all(a[k]==b[k] for k in keys) for a,b in zip(prior['calls'],report['cases'][0]['calls']))
with zipfile.ZipFile(paths[2]) as archive:
    source_ok=all(hashlib.sha256(archive.read(n)).hexdigest()==h for n,h in report['source_sha256'].items())
checks=dict(report_summary_equal=[c['summary'] for c in report['cases']]==summary['cases'],
    three_cases=len(report['cases'])==3,all_270_calls=all(len(c['calls'])==270 for c in report['cases']),
    all_last_call_at_5_9818=all(abs(c['calls'][-1]['t_s']-5.9818)<1e-10 for c in report['cases']),
    each_20_explicit_zero_calls=all(sum(r['explicit_stop'] for r in c['calls'])==20 and
        all(r['request_mps']==0 and r['safe_mps']==0 for r in c['calls'] if r['explicit_stop']) for c in report['cases']),
    captured_sources_match_archive=source_ok,nominal_off_matches_run13_actions_and_trajectory=same,
    recorded_python_exit_zero=raw[paths[3]].decode().strip()=='0',summary_integrity=summary['integrity_pass'],
    scan_schedule_valid=summary['scan_schedule_valid'])
result=dict(checks=checks,calculation_complete=all(checks.values()),
    wrapper_end_timestamp_missing=not raw[paths[4]].strip(),shell_session_interrupted=True,
    interpretation='Python result complete; wrapper final date/exit interrupted. Not a new run or service shutdown proof.',
    evidence_sha256={str(p.relative_to(ROOT)):hashlib.sha256(b).hexdigest() for p,b in raw.items()})
with (RUN/'completion_check.json').open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['calculation_complete'] else 1)
