"""r469의 runner/명령만 재사용한다. 공통 수집·동결·WSL·ROS 실행 없음."""
import ast,difflib,hashlib,json
from pathlib import Path
import shutil

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
BASE=A.parent/'solo_full_lap_r469_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert not (A/'baseline_copy.json').exists()
    oldrel=BASE.relative_to(ROOT).as_posix();newrel=A.relative_to(ROOT).as_posix()
    original_records=json.loads((BASE/'baseline_copy.json').read_bytes())
    records=[];patch=[]
    for row in original_records:
        source=BASE/row['path'];target=A/row['path']
        target.parent.mkdir(parents=True,exist_ok=True)
        if row['change']=='byte_exact_recipe_copy':
            shutil.copyfile(source,target)
            change='byte_exact_recipe_copy'
        elif row['path']=='config.json':
            data=json.loads(source.read_bytes())
            data['scope']='r475 solo full lap; r469 actor0 geometry/sensor/speed/physics/acceptance retained; r474 current-route priority with r466 observer OFF and r470 native'
            data['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
            target.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8',newline='\n')
            change='scope_and_artifact_entry_path_only'
        else:
            text=source.read_text(encoding='utf8').replace(oldrel,newrel).replace('r469','r475').replace('R469','R475')
            if row['path']=='run.py':text=text.replace("baseline='r465'","baseline='r469'")
            if row['path']=='execution_v1/execute_once.sh':text=text.replace('r465_plus_exact_root_reviewed_delta','r469_plus_exact_root_reviewed_delta')
            if row['path']=='prepare.py':
                text=text.replace('"""r465 recipe 재사용. 통합 후 실제 delta 전부를 주 담당 검토한 뒤에만 동결한다."""',
                    '"""r469 recipe 재사용. r474 최종 검사·실제 delta 검토 후에만 동결한다."""')
                text=text.replace("BASE=ROOT/'artifacts/validation/2026-11-13/solo_full_lap_r465_v1'", "BASE=ROOT/'artifacts/validation/2026-11-14/solo_full_lap_r469_v1'")
                start=text.index('INTEGRATION=');end=text.index('\n\ndef sha',start)
                text=text[:start]+text[end:]
                start=text.index("    assert {row['path'] for row in delta}==EXPECTED_DELTA")
                end=text.index("    return dict(baseline=",start)
                text=text[:start]+text[end:]
                text=text.replace("baseline='r465'","baseline='r469'").replace('r465과 같은 source 수집 범위','r469와 같은 source 수집 범위')
                text=text.replace("    assert review['root_reviewed'] is True", "    assert review['root_reviewed'] is True\n    assert review['r474_final_checks_passed'] is True\n    assert review['vehicle_marker_observer_enabled'] is False")
                text=text.replace("source_policy='r465_plus_exact_root_reviewed_delta'", "source_policy='r469_plus_exact_root_reviewed_delta'")
                change='same_freeze_workflow_actual_delta_and_r474_final_review_required'
            elif row['path']=='verify_recipe.py':
                text=text.replace(".replace('r465','r475').replace('R465','R475').replace(\"baseline='r457'\",\"baseline='r465'\")",
                    ".replace('r469','r475').replace('R469','R475').replace(\"baseline='r465'\",\"baseline='r469'\")")
                text=text.replace("assert 'r465_plus_exact_root_reviewed_delta' in wrapper", "assert 'r469_plus_exact_root_reviewed_delta' in wrapper")
                marker_check='''    # 보조 observer는 r469 명령에서 선택되지 않으며 공통 선언 기본값도 OFF다.
    marker_source=ROOT/'src/arena_autonomy/arena_autonomy/local_pursuit.py'
    marker_tree=ast.parse(marker_source.read_bytes())
    declarations=[node for node in ast.walk(marker_tree) if isinstance(node,ast.Call)
        and isinstance(node.func,ast.Attribute) and node.func.attr=='declare_parameter'
        and len(node.args)>=2 and isinstance(node.args[0],ast.Constant)
        and node.args[0].value=='vehicle_marker_observer_enabled']
    assert len(declarations)==1 and isinstance(declarations[0].args[1],ast.Constant)
    assert declarations[0].args[1].value is False
    assert not any('vehicle_marker_observer_enabled' in value for value in arguments())
'''
                text=text.replace('    paths=[*A.glob',marker_check+'    paths=[*A.glob',1)
                text=text.replace("pending=['exact approved five-file delta review'", "vehicle_marker_observer_enabled=False,\n        pending=['r474 final checks and exact actual common delta review'")
                change='windows_recipe_only_with_marker_off_assertion'
            elif row['path']=='build_v1/check_import_cli.py':
                text=text.replace("assert sha(native)==old['native']['sha256'] and _numeric_kernels.ABI_VERSION==1", "assert sha(native)==common[native.relative_to(ROOT).as_posix()] and _numeric_kernels.ABI_VERSION==1\nassert hasattr(_numeric_kernels,'current_mask_occupied')")
                text=text.replace("r465_archive_sha256=sha(BASE/'sources.zip')", "r469_archive_sha256=sha(BASE/'sources.zip')")
                text=text.replace('native_unchanged=True','native_matches_reviewed_source=True')
                change='actual_reviewed_native_hash_and_r470_entrypoint_required'
            else:change='own_path_label_or_baseline_link_only'
            target.write_text(text,encoding='utf8',newline='\n')
        records.append(dict(path=row['path'],baseline_sha256=sha(source),draft_sha256=sha(target),change=change))
        if source.read_bytes()!=target.read_bytes():
            patch.extend(difflib.unified_diff(source.read_text(encoding='utf8').splitlines(True),
                target.read_text(encoding='utf8').splitlines(True),fromfile='r469/'+row['path'],tofile='r475/'+row['path']))
    for file in A.rglob('*.py'):ast.parse(file.read_bytes())
    (A/'baseline_copy.json').write_text(json.dumps(records,indent=2),encoding='utf8')
    (A/'recipe_review.patch').write_text(''.join(patch),encoding='utf8',newline='\n')
    shutil.copyfile(BASE/'.gitignore',A/'.gitignore')
    print(json.dumps(dict(recipe_files_copied=len(records),byte_exact=sum(r['change']=='byte_exact_recipe_copy' for r in records),
        common_source_collected=False,source_frozen=False,build_executed=False,simulation_started=False)))

if __name__=='__main__':main()
