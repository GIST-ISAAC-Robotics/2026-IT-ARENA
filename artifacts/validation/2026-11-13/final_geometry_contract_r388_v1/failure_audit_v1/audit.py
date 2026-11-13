"""r388 원 결과/원문만 읽는 실패 감사. 재검사·native·ROS·장치 실행 없음."""
import ast
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import struct
import sys
import types
import zipfile

A=Path(__file__).resolve().parent
P=A.parent


def digest(raw):return hashlib.sha256(raw).hexdigest()


def main():
    output=A/'audit.json';assert not output.exists(),'preserve audit'
    saved=P/'remote_result/extracted';execution=json.loads((saved/'execution.json').read_bytes())
    reports={arm:json.loads((saved/'results_native_v1'/arm/'report.json').read_bytes()) for arm in ('frozen','legacy')}
    evidence=[P/'bundle_sources.zip',P/'remote_result/results.zip',saved/'execution.json',saved/'explicit.log',saved/'tests.log',
        *(saved/'results_native_v1'/arm/'report.json' for arm in reports),
        *(saved/'results_native_v1'/arm/'queries.npz' for arm in reports)]
    initial={p.relative_to(P).as_posix():digest(p.read_bytes()) for p in evidence}
    assert initial['bundle_sources.zip']==execution['archive_sha256']
    with zipfile.ZipFile(P/'bundle_sources.zip') as archive:
        names={key:'src/arena_vehicle_interface/arena_vehicle_interface/'+key+'.py'
            for key in ('steering_calibration','steering_cad_table')}
        names['launch']='src/arena_bringup/launch/simulation.launch.py'
        source={key:archive.read('frozen/'+name) for key,name in names.items()}
        assert all(archive.read('geometry/'+names[key])==source[key] for key in ('steering_calibration','steering_cad_table'))
        # 기존 순수 CAD 산술만 격리한 이름공간에서 실행. controller/native는 import하지 않는다.
        module=types.ModuleType('r388_frozen_cad_arithmetic');sys.modules[module.__name__]=module
        exec(compile(source['steering_calibration'],names['steering_calibration'],'exec'),module.__dict__)
        samples=next(ast.literal_eval(node.value) for node in ast.parse(source['steering_cad_table']).body
            if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='SAMPLES' for t in node.targets))
        fn=next(node for node in ast.parse(source['launch']).body
            if isinstance(node,ast.FunctionDef) and node.name=='_float32_limit_not_above')
        scope={'math':math,'struct':struct}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),names['launch'],'exec'),scope)
        vehicle_path='src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'
        yaml_raw=archive.read('frozen/'+vehicle_path)
        assert archive.read('geometry/'+vehicle_path)==yaml_raw
        # 이 감사는 YAML 설정 로더가 아니다. 보존 파일의 유일한 수치3행만 엄격히 읽는다.
        drive={}
        for key in ('wheelbase_m','track_width_m','max_steering_angle_rad'):
            matches=re.findall(r'^    '+key+r':\s*([^#\r\n]+)',yaml_raw.decode('utf8'),re.M)
            assert len(matches)==1
            drive[key]=ast.literal_eval(matches[0].strip())
        raw=module.CadMapping(samples).command_limit(wheelbase=drive['wheelbase_m'],track=drive['track_width_m'],
            wheel_limit=drive['max_steering_angle_rad'])
        sender=scope['_float32_limit_not_above'](raw)
        manifest=json.loads(archive.read('r388/source_manifest.json'))
        assert all(digest(archive.read(name))==h for name,h in manifest['files'].items())
    cfg=reports['legacy']['config'];saved_limit=cfg['max_steering_rad']
    bits=struct.unpack('<I',struct.pack('<f',sender))[0]
    above=struct.unpack('<f',struct.pack('<I',bits+1))[0]
    with zipfile.ZipFile(saved/'results_native_v1/frozen/queries.npz') as left,zipfile.ZipFile(saved/'results_native_v1/legacy/queries.npz') as right:
        arrays_exact=set(left.namelist())==set(right.namelist()) and all(left.read(name)==right.read(name) for name in left.namelist())
    result=dict(schema='r388_original_explicit_assert_failure_audit',original_pipeline_passed=False,
        original_execution_completed=execution['execution_completed'],original_failure=execution['error'],
        completed_stages={step['label']:step['returncode'] for step in execution['steps']},tests_passed=39,
        original_sources_unchanged=not execution['changed_sources'],bundle_inventory_verified=len(manifest['files']),
        original_r377_output_exact=all(r['recorded_exact_equal'] for r in reports.values()),
        frozen_legacy_output_exact=reports['frozen']['output']==reports['legacy']['output'],
        frozen_legacy_query_npy_bytes_exact=arrays_exact,
        wheelbase_saved_m=cfg['wheelbase_m'],wheelbase_yaml_m=drive['wheelbase_m'],
        wheelbase_exact_equal=cfg['wheelbase_m']==drive['wheelbase_m'],
        yaml_joint_limit_rad=drive['max_steering_angle_rad'],physical_equivalent_recomputed_rad=raw,
        sender_float32_floor_recomputed_rad=sender,saved_controller_limit_rad=saved_limit,
        sender_saved_exact_equal=sender==saved_limit,raw_saved_exact_equal=raw==saved_limit,
        difference_rad=raw-saved_limit,difference_degrees=math.degrees(raw-saved_limit),
        float32_next_above_rad=above,float32_ulp_rad=above-sender,
        difference_float32_ulp=(raw-saved_limit)/(above-sender),
        pure_arithmetic_host=platform.platform(),remote_raw_physical_limit_logged=False,
        diagnosis='원 launch의 안쪽 float32 송신 변환을 하네스 exact 비교에서 빠뜨렸다. 축거/선택 CAD source의 불일치 근거는 없다.',
        source_sha256={names[k]:digest(v) for k,v in source.items()},vehicle_yaml_sha256=digest(yaml_raw),
        evidence_sha256=initial,controller_calls=0,native_builds=0,remote_execution=False,existing_files_overwritten=False,
        proposed_scope=dict(harness='fresh r1에서 원 float32 변환까지 적용한 송신값과 exact 비교. 저장 cfg 조향/축거는 그대로 두고3치수만 변경.',
            staged_preview='현재 staged preview가 raw 물리한계를 사용하므로 simulation 송신 의미와 일치시키는 별도 선택형 보완 검토. 저장 대조와 분리.',
            original='원 실패 ZIP/실행/하네스/검증 결과 보존. 원 pipeline false 유지.'))
    assert initial=={p.relative_to(P).as_posix():digest(p.read_bytes()) for p in evidence}
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:result[k] for k in ('wheelbase_exact_equal','physical_equivalent_recomputed_rad',
        'saved_controller_limit_rad','sender_saved_exact_equal','difference_rad','frozen_legacy_output_exact','frozen_legacy_query_npy_bytes_exact')}))


if __name__=='__main__':main()
