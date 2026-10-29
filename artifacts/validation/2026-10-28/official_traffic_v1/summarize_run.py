"""공식 신호 교통 통합의 관측 결과만 회수한다. 원시 판정은 바꾸지 않는다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]

def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()

if __name__=='__main__':
    run=BASE/(sys.argv[1] if len(sys.argv)>1 else 'run1_course')
    out=BASE/(run.name+'_summary.json')
    if out.exists():raise FileExistsError(out)
    summary=load_module('traffic_summary',ROOT/'artifacts/validation/2026-10-27/g1_traffic_lap_v1/summarize_run.py')
    markers=load_module('marker_summary',BASE/'summarize_markers.py')
    watched=[run/name for name in ('report.json','trial_overlay.json','follow_trace_replay.json',
        'trajectory.jsonl','autonomy_status.jsonl','follow_decisions.jsonl','signal_sequence.jsonl')]
    before={str(p.relative_to(ROOT)):sha(p) for p in watched}
    report=json.loads((run/'report.json').read_text(encoding='utf-8'))
    overlay=json.loads((run/'trial_overlay.json').read_text(encoding='utf-8'))
    replay=json.loads((run/'follow_trace_replay.json').read_text(encoding='utf-8'))
    result=summary.summarize(run)
    result.update(markers=markers.summarize(run), runtime_contract=overlay.get('runtime_contract'),
        source_unchanged=overlay.get('source_unchanged'),
        snapshot_current_unchanged=all((ROOT/p).is_file() and sha(ROOT/p)==digest
            for p,digest in overlay['source_sha256'].items()),
        original_evaluator_exit=overlay.get('evaluator_exit'),
        stop=report.get('stop'), shutdown=report.get('shutdown_audit'),
        remaining_test_pids=report.get('remaining_test_pids'),
        minimum_gap=report['evaluation'].get('gap'),
        original_case_metrics=report['evaluation'].get('case_metrics'),
        false_start=report.get('false_start'), signal_states=report.get('signal_states'),
        replay={k:v for k,v in replay.items() if k != 'outcomes'},
        raw_sha256=before, raw_unchanged=all(sha(ROOT/p)==digest for p,digest in before.items()),
        limitations=['one 155s scripted traffic run, not independent opponent controllers',
            'official signal interpretation and 30cm camera are team fixture assumptions',
            'speed bump OFF; not dense grid, multi-lap or hardware proof'])
    with out.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({k:result[k] for k in ('report_passed','run_valid','failed_gates','progress',
        'runtime_contract','raw_unchanged','source_unchanged')},ensure_ascii=False))
