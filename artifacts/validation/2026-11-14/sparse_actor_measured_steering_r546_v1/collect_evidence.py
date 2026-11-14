"""r530/r534 저장 근거 수집. 실패/미완주도 처리하며 원 보고서는 수정하지 않는다."""
import argparse,collections,hashlib,json,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for data in iter(lambda:f.read(1024*1024),b''):h.update(data)
    return h.hexdigest()


def value_hash(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def trace_audit(path):
    kinds=collections.Counter();submitted={};completed={};calls=[];footer=None;header=None;last=None
    errors=[];duplicates=collections.Counter();reasons=collections.Counter();normal_positive=0;helpers=[];pubs=0;positive_pubs=0
    if not path.exists():return dict(present=False)
    with path.open(encoding='utf-8') as stream:
        for line_number,line in enumerate(stream,1):
            try:row=json.loads(line)
            except ValueError as error:errors.append(dict(line=line_number,error=str(error)));continue
            kind=row.get('kind');data=row.get('data') or {};kinds[kind]+=1;last=kind
            if kind=='header':header=data
            elif kind=='footer':footer=data
            elif kind in ('runtime_plan_submitted','runtime_plan_completed'):
                key=(data.get('generation'),data.get('sequence'));mapping=submitted if kind=='runtime_plan_submitted' else completed
                if key in mapping:duplicates[kind]+=1
                payload=data.get('input' if kind=='runtime_plan_submitted' else 'output');mapping[key]=value_hash(payload)
                if kind=='runtime_plan_completed' and isinstance(payload,list) and len(payload)>=3:
                    normal=(payload[2] or {}).get('normal') or {};reasons[normal.get('reason')]+=1
                    if payload[0]>0 and normal.get('selector_mode')=='FOLLOW':normal_positive+=1
                    if normal.get('current_wall_reacquire'):helpers.append(dict(seq=row.get('seq'),plan_id=key,speed_mps=payload[0],helper=normal['current_wall_reacquire']))
            elif kind=='call':
                compute=data.get('compute') or {};key=(compute.get('generation'),compute.get('sequence'))
                calls.append((row.get('seq'),key,value_hash(compute.get('input',data.get('input'))),value_hash(data.get('output'))))
            elif kind=='route_publication':pubs+=1;positive_pubs+=int(data.get('speed_mps',0)>0)
    return dict(present=True,kinds=dict(kinds),malformed_rows=errors,footer=footer,header=header,trace_closed=last=='footer',
        submitted=len(submitted),completed=len(completed),duplicate_records=dict(duplicates),
        unmatched_submitted=[list(k) for k in submitted.keys()-completed.keys()],unmatched_completed=[list(k) for k in completed.keys()-submitted.keys()],
        call_input_mismatches=[seq for seq,key,i,o in calls if submitted.get(key)!=i],call_output_mismatches=[seq for seq,key,i,o in calls if completed.get(key)!=o],
        plan_reasons=dict(reasons),positive_normal_follow_plans=normal_positive,low_reacquire_plan_records=helpers,
        route_publications=pubs,positive_route_publications=positive_pubs,
        scope='Plan helper occurrences and actual route publications counted separately; counts alone do not prove helper application or normal FOLLOW recovery')


def collect(artifact,kind):
    artifact=Path(artifact).resolve();artifact.relative_to(ROOT/'artifacts/validation');run=artifact/'run1';errors=[]
    output=artifact/'evidence_collection.json';assert not output.exists(),'preserve existing collection'
    def read(p):
        if not p.exists():return None
        try:return json.loads(p.read_bytes())
        except (ValueError,OSError) as error:errors.append(dict(path=str(p),error=str(error)));return None
    report=read(run/'report.json') or {};report_sha=sha(run/'report.json') if (run/'report.json').is_file() else None
    required=['report.json','source_snapshot.zip','runtime_world.sdf','runtime_vehicle.sdf','runtime_local_sensors.yaml',
        'simulation.log','readers.log','trace.json','trajectory.jsonl','scans_received.jsonl','autonomy_status.jsonl','safety_status.jsonl',
        'follow_decisions.jsonl','ego_poses.jsonl','actuation_trace.json','actuation_diagnostics.json','stop_trace.json','acquisition.json','native_sampling_profile.json']
    if kind=='solo':required+=['facilities_only.sdf','actor_poses.jsonl','actor_commands.json','actor_contacts.jsonl']
    inventory={n:dict(present=(run/n).is_file(),size_bytes=(run/n).stat().st_size if (run/n).is_file() else None,
        sha256=sha(run/n) if (run/n).is_file() else None) for n in required}
    source_map=report.get('source_sha256') or {};missing=[];mismatches=[]
    if (run/'source_snapshot.zip').is_file():
        try:
            with zipfile.ZipFile(run/'source_snapshot.zip') as archive:
                names=set(archive.namelist())
                for p,h in source_map.items():
                    if p not in names:missing.append(p)
                    elif hashlib.sha256(archive.read(p)).hexdigest()!=h:mismatches.append(p)
        except (OSError,ValueError,zipfile.BadZipFile) as error:errors.append(dict(stage='source_snapshot',error=str(error)))
    else:missing=list(source_map)
    trace=trace_audit(run/'follow_decisions.jsonl');comparison=None
    if trace.get('header') and (artifact/'expected_header.json').is_file():
        from header_comparison import compare_headers
        comparison=compare_headers(trace['header'],read(artifact/'expected_header.json'))
    safety=collections.Counter();first_latch=None
    if (run/'safety_status.jsonl').is_file():
        with (run/'safety_status.jsonl').open(encoding='utf-8') as stream:
            for n,line in enumerate(stream,1):
                try:row=json.loads(line);data=row.get('data') or {};safety[data.get('reason')]+=1
                except ValueError as error:errors.append(dict(stage='safety_status',line=n,error=str(error)));continue
                if first_latch is None and data.get('reason')=='known_obstacle_latched':first_latch=dict(line=n,**row)
    actuation=read(run/'actuation_trace.json') or [];prepared=read(artifact/'prepared.json') or {}
    changed=[p for p,h in prepared.get('frozen_inputs',{}).items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    evaluation=report.get('evaluation') or {};finish=report.get('solo_lap_finish_sim_s');green=report.get('green_observed_sim_s')
    result=dict(kind=kind,scope='Post-run stored evidence only; no replay or raw verdict change',original_report_sha256=report_sha,
        original_passed=report.get('passed'),run_valid=report.get('run_valid'),harness_valid=report.get('harness_valid'),
        original_abort=report.get('abort'),original_error=report.get('error'),gates=report.get('gates'),case_window=report.get('case_window'),
        progress=evaluation.get('progress'),full_course_progress=evaluation.get('full_course_progress'),case_metrics=evaluation.get('case_metrics'),
        lap_elapsed_since_green_sim_s=finish-green if finish is not None and green is not None else None,
        wall_elapsed_s=report.get('wall_elapsed_s'),stop_checks=report.get('stop_checks'),stop=report.get('stop'),
        normal_shutdown={k:report.get(k) for k in ('shutdown_clean','shutdown_audit','gazebo_stop_service','process_exit_codes','reader_forced_cleanup','remaining_test_pids')},
        owned_process_audit=read(artifact/'execution_v1/owned_processes_after.json'),
        run_exit=(artifact/'execution_v1/run.exit').read_text().strip() if (artifact/'execution_v1/run.exit').exists() else None,
        evidence_files=inventory,missing_required_files=[n for n,v in inventory.items() if not v['present']],
        source_snapshot=dict(entries_expected=len(source_map),missing=missing,mismatches=mismatches,source_map_nonempty=bool(source_map),
            derived_world_in_snapshot=(artifact/'field_world.sdf').relative_to(ROOT).as_posix() in source_map),
        frozen_input_integrity=dict(original_receipt=read(artifact/'integrity.json'),files=len(prepared.get('frozen_inputs',{})),changed_now=changed,
            archive_unchanged_now=sha(artifact/'sources.zip')==prepared.get('source_archive_sha256') if (artifact/'sources.zip').is_file() else None),
        runtime_resource_receipt=read(artifact/'runtime_resource_receipt.json'),evaluation_geometry=read(artifact/'evaluation_geometry.json'),
        field_runtime_receipt=read(artifact/'field_runtime_receipt.json'),controller_trace=trace,header_comparison=comparison,
        safety_reason_counts=dict(safety),first_known_obstacle_latch=first_latch,
        safety_scope='Whole existing validator subscriber receipt; not guard internal callback ordering',
        command_expired_samples=sum(row.get('reason')=='command_expired' for row in actuation),collection_errors=errors,
        raw_report_unchanged=report_sha is not None and report_sha==sha(run/'report.json'))
    with output.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(dict(kind=kind,original_passed=result['original_passed'],missing_files=result['missing_required_files'],
        shutdown_clean=report.get('shutdown_clean'),raw_report_unchanged=result['raw_report_unchanged'],errors=len(errors))))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--artifact',type=Path,default=A);parser.add_argument('--kind',choices=('solo','traffic'),default='solo');args=parser.parse_args();collect(args.artifact,args.kind)
