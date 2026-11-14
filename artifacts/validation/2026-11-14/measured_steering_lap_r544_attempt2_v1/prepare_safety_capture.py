"""승인된 r530 artifact에만 기존 safety 구독 원문 기록을 연결한다. 공통 수정 없음."""
import ast,hashlib,json,shutil
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];T=A.parent/'field_wall_resource_drive_r530_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def instrument(text):
    pairs=[("    status_stream = (output/'autonomy_status.jsonl').open('w', encoding='utf-8')",
        "    status_stream = (output/'autonomy_status.jsonl').open('w', encoding='utf-8')\n    safety_stream = (output/'safety_status.jsonl').open('w', encoding='utf-8')"),
        ('    def receive_dynamics(message):',
         "    def receive_safety(message):\n        status = json.loads(message.data)\n        remember(status, 'safety')\n        safety_stream.write(json.dumps(dict(receipt_monotonic_s=time.monotonic(),data=status))+'\\n')\n\n    def receive_dynamics(message):"),
        ("lambda m: remember(json.loads(m.data), 'safety'), 10)","receive_safety, 10)"),
        ('        status_stream.close()','        status_stream.close()\n        safety_stream.close()'),
        ("for name in ('scans_received', 'autonomy_status'):","for name in ('scans_received', 'autonomy_status', 'safety_status'):")]
    result=text
    for before,after in pairs:
        assert result.count(before)==1,before;result=result.replace(before,after)
    restored=result
    for before,after in reversed(pairs):assert restored.count(after)==1;restored=restored.replace(after,before)
    assert restored==text
    ast.parse(result)
    return result


def main():
    assert not any((T/n).exists() for n in ('prepared.json','sources.zip','registered.json','run1','traffic_runner.py'))
    original=ROOT/'scripts/validate_g1_traffic.py';world_hash=sha(T/'field_world.sdf');fixture_hash=sha(T/'facilities_only.sdf')
    before=T/'instrumentation_before';before.mkdir()
    for n in ('run.py','preflight.py','recipe_provenance.json','preparation_receipt.json'):shutil.copyfile(T/n,before/n)
    (T/'traffic_runner.py').write_text(instrument(original.read_text(encoding='utf-8')),encoding='utf-8',newline='\n')
    for name in ('run.py','preflight.py'):
        p=T/name;text=p.read_text(encoding='utf-8');assert text.count('import validate_g1_traffic as target')==1
        text=text.replace('import validate_g1_traffic as target','import traffic_runner as target')
        if name=='run.py':
            token='    verify_derived()\n'
            extra="""    verify_derived()
    # 실제 source_snapshot에도 실행 harness/파생월드/외부 자산을 포함한다.
    from world_resources import verify_resources
    source_paths = target.source_paths
    local_files = [*A.glob('*.py'), *(A/'launch').glob('*.py'),
        *[A/n for n in ('field_world.sdf','facilities_only.sdf','resource_manifest.json','field_geometry_audit.json')]]
    paths = [p.relative_to(ROOT).as_posix() for p in local_files]
    paths += [r['repository_path'] for r in verify_resources()['asset_references']]
    target.source_paths = lambda *args, **kwargs: list(dict.fromkeys([*source_paths(*args, **kwargs), *paths]))
"""
            assert text.count(token)==1;text=text.replace(token,extra)
        ast.parse(text);p.write_text(text,encoding='utf-8',newline='\n')
    receipt=dict(common_validator_path=original.relative_to(ROOT).as_posix(),common_validator_sha256=sha(original),
        local_runner_sha256=sha(T/'traffic_runner.py'),five_exact_text_insertions_reversible=True,
        existing_subscription_qos_depth=10,extra_subscription_count=0,whole_safety_status_json_preserved=True,
        clock_contract='receipt_monotonic_s is validator host receipt; status payload retains original simulation/reference fields',
        actual_source_snapshot_includes_local_runner_world_and_assets=True,
        original_world_unchanged=sha(T/'field_world.sdf')==world_hash,facilities_unchanged=sha(T/'facilities_only.sdf')==fixture_hash,
        controller_or_acceptance_changed=False,WSL_used=False,simulator_started=False)
    (T/'safety_capture_preparation.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
    p=T/'recipe_provenance.json';v=json.loads(p.read_bytes())
    for item in v['files']:
        if item['path'] in ('run.py','preflight.py'):
            item['after_sha256']=sha(T/item['path']);item['additional_change']='Artifact-only safety raw capture and actual snapshot inclusion'
    v['files'].append(dict(path='traffic_runner.py',source=original.relative_to(ROOT).as_posix(),before_sha256=sha(original),
        after_sha256=sha(T/'traffic_runner.py'),byte_exact=False,additional_change='Five reversible logger-only text additions'))
    p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(receipt))


if __name__=='__main__':main()
