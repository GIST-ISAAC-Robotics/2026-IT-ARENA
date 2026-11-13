"""완료된 세 arm을 읽어 원 재현/기하효과/후보효과/반례/비용을 구분한다."""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    root=args.output.resolve();reports={name:json.loads((root/name/'report.json').read_bytes()) for name in ('frozen','legacy','final')}
    frozen,legacy,final=(reports[name] for name in ('frozen','legacy','final'))
    checks=dict(each_arm_passed=all(r['passed'] for r in reports.values()),
        frozen795=frozen['completed_stored']==795,legacy795_exact=legacy['completed_stored']==legacy['legacy_frozen_exact']==795,
        original_r377223_exact=frozen['original_r377_recorded_exact']==legacy['original_r377_recorded_exact']==223,
        legacy572_preserved=legacy['preservation_count']==572 and legacy['preservation_applied']==0,
        final223=final['completed_stored']==223,negative15_stopped=final['completed_counterexamples']==15 and all(r['counterexample_passed'] for r in final['counterexamples']),
        same_native_binary=len({r['imports']['native']['sha256'] for r in reports.values()})==1,
        same_candidate=len({r['candidate_sha256'] for r in reports.values()})==1,
        same_budget=len({r['budget_sha256'] for r in reports.values()})==1)
    result=dict(schema='r386_final_geometry_summary',passed=all(checks.values()),checks=checks,
        original_r377_run_passed=False,original_r386_archive_executed=False,algorithm_modified=False,
        final_geometry=dict(baseline_changed=final['geometry_baseline_changed'],geometry_only_positive_recovery=final['geometry_only_positive_recovery']),
        candidate_effect={k:final[k] for k in ('r377_attempted','r377_applied','baseline_positive','candidate_positive','rejection_reasons')},
        last=final['last'],counterexamples=final['counterexamples'],costs=final['groups'],
        automatic_adoption=False,new_simulation=False,full_lap=False,command_expiry_evaluated=False,
        scope='저장795 legacy재현과 명시기하223 후보/15반례의 단일직렬 비교. 실차/완주/10Hz/안전허가 증명 아님.')
    (root/'comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:result[k] for k in ('passed','checks','final_geometry','candidate_effect')},ensure_ascii=False))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
