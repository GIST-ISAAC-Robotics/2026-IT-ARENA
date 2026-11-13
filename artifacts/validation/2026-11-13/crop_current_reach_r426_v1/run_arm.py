"""분리된 소스 arm의 current만 재생한다. 원 plan 상태 이력/ROS 재생 아님."""
import argparse
import copy
import json
from pathlib import Path
import signal
import time
import traceback

from audited_evaluator import evaluator_factory
from support import (changed, diagnostic_summary, difference, encoded, raw_recorded,
                     recorded_after_cap, write_new)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', choices=('baseline','candidate'), required=True)
    parser.add_argument('--phase', choices=('representative','remaining'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); root = Path(__file__).resolve().parent
    project = root/args.arm; out = args.output.resolve()
    assert out.is_relative_to(root/'results') and not out.exists(); out.mkdir(parents=True)
    manifest = json.loads((root/'bundle_manifest.json').read_bytes()); assert not changed(root,manifest)
    header = json.loads((root/'inputs/header.json').read_bytes())
    records = [json.loads(line) for line in (root/'inputs/currents.jsonl').open(encoding='utf8')]
    records = [r for r in records if (r['trace_seq']==1003) == (args.phase=='representative')]
    assert len(records) == (1 if args.phase=='representative' else 312)
    from arena_autonomy.runtime_dual_process import Lane, plan_reference
    if args.arm == 'candidate':
        from arena_autonomy.runtime_dual_state import current_request
    rows=[]; lane=None; cleanup=None; error=None; imports={}; source_changes=[]
    handlers={}
    def interrupted(number,_): raise InterruptedError('signal_'+str(number))
    for number in (signal.SIGINT,signal.SIGTERM): handlers[number]=signal.signal(number,interrupted)
    try:
        lane=Lane('current',header,capacity=1024*1024,timeout=20.,evaluator_factory=evaluator_factory)
        deadline=time.monotonic()+180.
        with (out/'rows.jsonl').open('x',encoding='utf8') as stream:
            for record in records:
                if time.monotonic()>=deadline: raise TimeoutError('bounded saved current replay')
                prior=copy.deepcopy(record); plan=record['plan_input']; current=record['current']
                original=dict(output=record['plan_output'],submitted=plan_reference(plan),current=current,retained=record['retained'])
                data=(original if args.arm=='baseline' else current_request(record['plan_output'],plan,current,
                    retained=record['retained'],crop_current_selection=(
                        header['follow_config']['crop_edge_component_trial'] is True
                        and header['current_corner_budget']=='r396_v1')))
                assert {k:v for k,v in data.items() if k!='crop_current_input'} == original
                lane.offer(data,record['generation'],record['sequence'])
                while True:
                    lane.connection.poll(.01); received=lane.collect()
                    if received is not None: break
                    if time.monotonic()>=deadline: raise TimeoutError('owned current Lane wait')
                audit=received['output']; actual=audit['output']; imports.update(audit['imports'])
                row=dict(index=record['index'],trace_seq=record['trace_seq'],generation=record['generation'],
                    sequence=record['sequence'],phase=args.phase,arm=args.arm,original_input_sha256=record['input_sha256'],
                    parent_input_unchanged=record==prior,request_sha256=__import__('hashlib').sha256(encoded(data)).hexdigest(),
                    parent_compact_evidence=data.get('crop_current_input'),diagnostics=diagnostic_summary(actual),**audit)
                if args.arm=='baseline':
                    row['raw_expected_diff']=difference(raw_recorded(record['expected']),actual)
                    row['recorded_parent_diff']=difference(record['expected'],recorded_after_cap(actual,record['expected']))
                stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush();rows.append(row)
                assert audit['input_unchanged'] and record==prior and audit['baseline_calls']==1
                assert not audit['external_project_imports']
                if args.arm=='baseline':
                    assert row['raw_expected_diff'] is None and row['recorded_parent_diff'] is None, ('baseline exact failed',record['trace_seq'])
    except BaseException as exc:
        error=type(exc).__name__+': '+str(exc);(out/'traceback.txt').write_text(traceback.format_exc(),encoding='utf8')
    finally:
        if lane is not None: cleanup=lane.close()
        for number,previous in handlers.items():signal.signal(number,previous)
        source_changes=changed(root,manifest)
        report=dict(arm=args.arm,phase=args.phase,completed=len(rows),expected=len(records),error=error,
            cleanup=cleanup,source_changes=source_changes,imports=imports,
            passed=error is None and len(rows)==len(records) and not source_changes and cleanup is not None
                and cleanup['exitcode']==0 and not cleanup['forced'] and not cleanup['alive'],
            real_time_or_command_expiry_test=False,plan_history_replayed=False,ros_started=False,device_opened=False)
        write_new(out/'report.json',report)
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
