"""동결 WSL 저장 검사만 실행. unit → 대표 → 양수일 때만 나머지312개."""
import argparse
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import zipfile

from support import changed, difference, digest, encoded, write_new


def rows(path): return [json.loads(s) for s in path.open(encoding='utf8')]


def compare_pair(before, after):
    assert before['index']==after['index'] and before['original_input_sha256']==after['original_input_sha256']
    old, new = before['output'], after['output']
    result = dict(index=before['index'],trace_seq=before['trace_seq'],crop_scope=after['crop_scope'],
                  baseline_speed_mps=old['speed_mps'],candidate_speed_mps=new['speed_mps'],
                  difference=difference(old,new),diagnostics=after['diagnostics'])
    target = after['crop_scope'] and old['speed_mps']==0 and old['reason']=='route_current_large_motion_region_unobserved'
    if not target:
        assert result['difference'] is None, ('non-target changed',result)
        result['outcome']='non_target_unchanged'
    elif 'current_crop_component_retry' not in new:
        assert result['difference'] is None, ('not-entered candidate changed output',result)
        result['outcome']='current_retry_not_entered'
    elif new['speed_mps']>0:
        assert 0<new['speed_mps']<=.2
        trial=new['current_crop_component_retry']
        assert trial['applied'] is True and trial['attempted_targets']==trial['final_full_checks']==1
        for key in ('steering_rad','reference_s','path_source_s','original_reference_s','path_points'):
            assert difference(old.get(key),new.get(key)) is None, ('identity changed',key)
        proof=new['current_route_proof']
        assert proof['accepted'] is True and proof['source_timestamp_renewed'] is False
        assert proof['old_occupancy_permission_reused'] is False
        assert new['independent_guard_required'] is True and new['runtime_authorized'] is False
        result['outcome']='recovered'
    else:
        retained={k:v for k,v in new.items() if k!='current_crop_component_retry'}
        assert difference(old,retained) is None, ('rejected candidate changed original fields',result)
        result['outcome']='current_proof_rejected'
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--stage',choices=('units','representative','conditional-full'),required=True)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--receipt',type=Path,required=True)
    args=parser.parse_args();here=Path(__file__).resolve().parent;root=args.root.resolve();receipt=args.receipt.resolve()
    assert sys.platform=='linux' and platform.machine()=='x86_64'
    assert root.parent==Path.home() and root.name.startswith('it-arena-r426-')
    assert receipt.parent==here
    prep=json.loads((here/'preparation_manifest.json').read_bytes());assert not changed(here,prep)
    if args.stage=='units':
        assert not root.exists() and not receipt.exists();root.mkdir();receipt.mkdir()
        with zipfile.ZipFile(here/'bundle_sources.zip') as z:
            for n in z.namelist():assert not Path(n).is_absolute() and '..' not in Path(n).parts and '\\' not in n
            z.extractall(root)
        (root/'results').mkdir()
    manifest=json.loads((root/'bundle_manifest.json').read_bytes());assert not changed(root,manifest)
    stage_out=root/'results'/args.stage;stage_out.mkdir()
    report=dict(stage=args.stage,root=str(root),steps=[],error=None,full_comparison_completed=False,
                simulation_started=False,jetson_accessed=False,physical_devices_opened=False)
    def execute(label,command,cwd,arm,timeout):
        isolated=[root/arm/'src/arena_autonomy',root/arm/'src/arena_vehicle_interface',root]
        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
            PYTHONPATH=os.pathsep.join([str(p) for p in isolated]+[p for p in os.environ.get('PYTHONPATH','').split(os.pathsep) if p]))
        with (stage_out/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            entry=dict(label=label,pid=child.pid,command=command,timeout_s=timeout,forced=False);report['steps'].append(entry)
            try: entry['returncode']=child.wait(timeout=timeout)
            except BaseException:
                entry['forced']=True;os.killpg(child.pid,signal.SIGTERM)
                try:entry['returncode']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);entry['returncode']=child.wait(timeout=5)
                raise
            finally:entry['reaped']=child.poll() is not None
        if entry['returncode']:raise RuntimeError((label,entry['returncode']))
    def run(arm,phase):
        destination=root/'results'/(phase+'_'+arm)
        execute(phase+'_'+arm,[sys.executable,'-B',str(root/'run_arm.py'),'--arm',arm,'--phase',phase,
            '--output',str(destination)],root/arm,arm,240)
        result=json.loads((destination/'report.json').read_bytes());assert result['passed']
        return rows(destination/'rows.jsonl')
    def checked_pairs(base, candidate):
        assert len(base)==len(candidate)
        with (stage_out/'comparisons.jsonl').open('x',encoding='utf8') as stream:
            output=[]
            for a,b in zip(base,candidate):
                # 생출력은 각 arm rows에 이미 기록됐다. 비교 예외도 원 결과를 지우지 않는다.
                value=compare_pair(a,b);stream.write(json.dumps(value,allow_nan=False)+'\n');stream.flush();output.append(value)
        return output
    old_handlers={}
    def interrupted(number,_):raise InterruptedError('signal_'+str(number))
    for number in (signal.SIGINT,signal.SIGTERM):old_handlers[number]=signal.signal(number,interrupted)
    try:
        if args.stage=='units':
            execute('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],root/'baseline/src/arena_autonomy','baseline',120)
            binaries=list((root/'baseline/src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'));assert len(binaries)==1
            target=root/'candidate/src/arena_autonomy/arena_autonomy'/binaries[0].name;shutil.copyfile(binaries[0],target)
            report['native_binaries']={p.relative_to(root).as_posix():digest(p) for p in [binaries[0],target]}
            assert len(set(report['native_binaries'].values()))==1
            reference=str(root/'tests/reference_r423_scope.py')
            selected_reference=[reference+'::'+name for name in (
                'test_same_scan_compact_contract_and_no_input_mutation',
                'test_actual_parent_offer_uses_owned_pair_and_leaves_inputs_unchanged',
                'test_original_timing_is_preserved_and_rejected',
                'test_false_or_other_plan_markers_do_not_select[false_allowed]',
                'test_false_or_other_plan_markers_do_not_select[latch]',
                'test_whole_scan_metadata_difference_recomputed_at_parent[acquired_at_s]')]
            execute('units',[sys.executable,'-B','-m','pytest','-q',str(root/'tests/test_crop_current_reach.py'),*selected_reference,
                '--junitxml='+str(stage_out/'junit.xml')],root,'candidate',120)
        else:
            unit=json.loads((root/'results/units/report.json').read_bytes());assert unit['verification_passed']
            assert all(digest(root/n)==h for n,h in unit['native_binaries'].items())
            if args.stage=='representative':
                pairs=checked_pairs(run('baseline','representative'),run('candidate','representative'))
                report.update(representative=pairs[0],representative_recovered=pairs[0]['outcome']=='recovered',
                    next_stage_allowed=pairs[0]['outcome']=='recovered')
                if not report['next_stage_allowed']: report['stop_reason']='representative_not_recovered_keep_original_rejection'
            else:
                first=json.loads((root/'results/representative/report.json').read_bytes());assert first['verification_passed']
                if not first['next_stage_allowed']:
                    report.update(skipped=True,stop_reason='representative_not_recovered_no_further_control_calls')
                else:
                    base=run('baseline','remaining');candidate=run('candidate','remaining')
                    pairs=checked_pairs(base,candidate);all_pairs=[first['representative'],*pairs]
                    assert len(all_pairs)==313 and len({p['index'] for p in all_pairs})==313
                    report.update(full_comparison_completed=True,total_unique_current=313,
                        representative_reused_without_repeat=True,remaining_current=312,
                        baseline_positive=sum(p['baseline_speed_mps']>0 for p in all_pairs),
                        candidate_positive=sum(p['candidate_speed_mps']>0 for p in all_pairs),
                        recovered_trace_seq=[p['trace_seq'] for p in all_pairs if p['outcome']=='recovered'],
                        non_target_unchanged=sum(p['outcome']=='non_target_unchanged' for p in all_pairs))
    except BaseException as exc:
        import traceback
        report['error']=type(exc).__name__+': '+str(exc)
        (stage_out/'traceback.txt').write_text(traceback.format_exc(),encoding='utf8')
    finally:
        for number,previous in old_handlers.items():signal.signal(number,previous)
        report['source_changes']=changed(root,manifest);report['preparation_changes']=changed(here,prep)
        report['verification_passed']=report['error'] is None and not report['source_changes'] and not report['preparation_changes']
        write_new(stage_out/'report.json',report);write_new(receipt/(args.stage+'.json'),report)
        with zipfile.ZipFile(receipt/(args.stage+'.zip'),'x',zipfile.ZIP_DEFLATED) as z:
            for p in (root/'results').rglob('*'):
                if p.is_file():z.write(p,p.relative_to(root).as_posix())
        print(json.dumps({k:v for k,v in report.items() if k not in ('representative','steps')},ensure_ascii=False))
    if not report['verification_passed']:raise SystemExit(1)


if __name__=='__main__':main()
