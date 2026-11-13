"""현재 코드의 원 상태 shadow 및 독립 선택형 재생. 실제 주행/출력 발행 아님."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import candidate
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, replay_reset, freeze, equal_output, comparable_decision
from arena_autonomy.numeric_kernels import configure


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['python','native_v1'], default='python')
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args();out=args.output;out.mkdir(exist_ok=False)
    source=A.parent/'final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl'
    archive=source.parent/'source_snapshot.zip'
    manifest={str(p.relative_to(ROOT)):digest(p) for p in sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))}
    manifest.update({str(p.relative_to(ROOT)):digest(p) for p in [source,archive,A/'candidate.py',A/'replay.py']})
    (out/'input_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    events=[json.loads(line) for line in source.open()]
    header=events[0]['data'];cfg=header['follow_config']
    assert (cfg['wheelbase_m'],cfg['length_m'],cfg['width_m'])==(.128,.207,.15)
    spec=importlib.util.spec_from_file_location('r361_existing_boundary',
        A.parents[1]/'2026-11-12/terminal_boundary_r278_v1/candidate.py')
    boundary=importlib.util.module_from_spec(spec);spec.loader.exec_module(boundary)
    boundary.install();configure(args.backend)
    original,motion=make_controller(header);changed,_=make_controller(header)
    results=[];shadow_rows=[];sequence=None
    def observer(owner,scene,baseline,before,after,evidence,speed,previous,dt):
        shadow_rows.append(dict(seq=sequence,baseline_speed_mps=baseline[0],baseline_steering_rad=baseline[1],
            before=freeze(before[:2]),after=freeze(after[:2]),evidence=freeze(evidence)))
    began=time.monotonic()
    try:
        for event in events[1:]:
            kind,data=event['kind'],event['data'];sequence=event['seq']
            if kind=='reset':
                replay_reset(original,data);replay_reset(changed,data)
            elif kind=='call':
                before_count=len(shadow_rows)
                with candidate.enabled(shadow=True,observer=observer):
                    before=freeze(replay_call(original,data['input'],motion))
                with candidate.enabled():
                    after=freeze(replay_call(changed,data['input'],motion))
                expected=data['output'];normal=before[2].get('normal',{})
                expected_normal=expected[2].get('normal',{})
                new=after[2].get('normal',{});retry=new.get('selector_released_crawl_retry') or {}
                result=dict(seq=sequence,reference_s=data['input']['reference_s'],
                    raw_measured_speed_mps=data['input']['speed_mps'],
                    before_mps=before[0],after_mps=after[0],before_steering_rad=before[1],after_steering_rad=after[1],
                    original_command_equal=equal_output(before[:2],expected[:2]),
                    original_full_equal=equal_output(comparable_decision(before),comparable_decision(expected)),
                    original_region_equal=equal_output(normal.get('selector_region'),expected_normal.get('selector_region')),
                    candidate_changed=not equal_output(comparable_decision(before),comparable_decision(after)),
                    command_changed=not equal_output(before[:2],after[:2]),applied=retry.get('applied') is True,
                    before_reason=normal.get('selector_reason'),after_reason=new.get('selector_reason'),
                    final_stall=sequence>=804,
                    shadow=shadow_rows[before_count:],current_retry=retry)
                results.append(result)
                if len(results)%100==0:print(json.dumps(dict(calls=len(results),applied=sum(r['applied'] for r in results))),flush=True)
    finally:boundary.uninstall()
    drift=[name for name,h in manifest.items() if digest(ROOT/name)!=h]
    late=[r for r in results if r['final_stall']]
    diff=[r for r in results if not r['original_command_equal']]
    eligible=[s for s in shadow_rows if s['evidence']['eligible']]
    applied=[r for r in results if r['applied']]
    untriggered=[r for r in results if not r['applied']]
    report=dict(calls=len(results),backend=args.backend,
        original_command_equal=sum(r['original_command_equal'] for r in results),
        original_full_equal=sum(r['original_full_equal'] for r in results),
        original_command_differences=diff,final_stall_calls=len(late),
        final_stall_original_command_equal=sum(r['original_command_equal'] for r in late),
        final_stall_original_region_equal=sum(r['original_region_equal'] for r in late),
        shadow_eligible=len(eligible),shadow_accepted=sum(s['evidence']['applied'] for s in eligible),
        candidate_applied=len(applied),command_changes=sum(r['command_changed'] for r in results),
        untriggered=len(untriggered),untriggered_full_unchanged=sum(not r['candidate_changed'] for r in untriggered),
        applied_zero_to_point_one=all(r['before_mps']==0 and r['after_mps']==.1 for r in applied),
        applied_general_unobserved_zero=all(r['current_retry']['region']['unobserved']==0 for r in applied),
        applied_front_mask_zero=all(r['current_retry']['region']['masked_front_samples']==0 for r in applied),
        applied_tail_max_m=max((r['current_retry']['region']['masked_tail_extent_m'] for r in applied),default=None),
        measured_speed_preserved=all(r['raw_measured_speed_mps']==r['current_retry']['measured_speed_mps'] for r in applied),
        elapsed_s=time.monotonic()-began,changed_sources=drift,source_sha256=digest(source),
        scope='same recorded final-geometry inputs, no drive, no current stage or guard publication, no runtime deadline proof')
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (out/'decisions.json').write_text(json.dumps(results,separators=(',',':'))+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='original_command_differences'}),flush=True)
    assert not drift
    assert len(late)==173 and report['final_stall_original_command_equal']==173
    assert report['applied_zero_to_point_one'] and report['applied_general_unobserved_zero']
    assert report['applied_front_mask_zero'] and report['measured_speed_preserved']
    assert report['untriggered_full_unchanged']==report['untriggered']


if __name__=='__main__':main()
