"""r369와 같은 새 루트 ZIP+전파일 SHA 보존. 원격 접속/빌드/계측은 하지 않는다."""
import ast
import json
from pathlib import Path
import zipfile

from support import A, ROOT, POLICY, EXPECTED, CHAIN, digest


def main():
    archive = A/'bundle_sources.zip'
    manifest_path = A/'source_manifest.json'
    assert not archive.exists() and not manifest_path.exists(), 'existing bundle is preserved'
    expected = json.loads(EXPECTED.read_bytes())
    assert expected['passed'] and expected['groups']['r371'] == {'count': 217, 'applied': 93}
    files = set()
    # ROS/Gazebo 실행물 전체 대신 두 계산 패키지의 소스만 포함한다.
    # native .so는 PC 아키텍처 산출물이므로 넣지 않고 새 Jetson 루트에서 명시 빌드한다.
    for package in ('arena_autonomy', 'arena_vehicle_interface'):
        folder = ROOT/'src'/package
        files.update(p for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts
                     and p.suffix in ('.py', '.cpp', '.hpp', '.h', '.xml', '.cfg'))
        resource = folder/'resource'/package
        if resource.is_file(): files.add(resource)
    # 비교 실행의 등록 모듈과 명시적 날짜별 동적 candidate 계보를 모두 넣는다.
    # sys.modules에는 importlib로 만든 candidate가 없으므로 기록 manifest만 쓰지 않는다.
    for name, value in expected['source_hashes'].items():
        path = ROOT/name
        assert digest(path) == value, ('r375 source changed', name)
        if path.suffix == '.py': files.add(path)
    files.update(ROOT/name for name in CHAIN)
    files.update((ROOT/'artifacts/validation/2026-11-12/retained_route_execution_r266_v1/candidate_recheck').glob('*.py'))
    files.update(POLICY.glob('*.py'))
    files.add(EXPECTED)
    for name, value in expected['input_hashes'].items():
        path = ROOT/name
        assert digest(path) == value, ('r375 input changed', name)
        files.add(path)
    files.update(p for p in A.iterdir() if p.is_file() and p.suffix in ('.py', '.sh', '.md'))
    assert all(p.is_file() and p.is_relative_to(ROOT) for p in files)
    for path in files:
        if path.suffix == '.py': ast.parse(path.read_bytes(), filename=str(path))
    inventory = {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(files)}
    assert all(name in inventory for name in CHAIN)
    manifest = dict(schema='r380_current_cost_bundle_v1', files=inventory,
        explicit_dynamic_chain=list(CHAIN), native_binary_included=False,
        inputs=dict(expected['input_hashes']), reference_comparison_sha256=digest(EXPECTED),
        source_scope='두 계산 패키지 소스+명시 동적 계보+기존 비교/입력+계측 도구. 원시 자료 공개 금지.',
        historical_bundle_style='r369의 새 루트/원본 불변/SHA 보존을 따르며 r369 디렉터리를 수정하거나 재사용하지 않는다.')
    with manifest_path.open('x', encoding='utf8') as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2); stream.write('\n')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED, compresslevel=1) as zipped:
        for path in sorted(files): zipped.write(path, path.relative_to(ROOT).as_posix())
        zipped.write(manifest_path, manifest_path.relative_to(ROOT).as_posix())
    # 압축 내부의 개별 바이트와 실제 해시를 대조한다. 프로그램 import/실행은 별도다.
    with zipfile.ZipFile(archive) as zipped:
        import hashlib
        assert all(hashlib.sha256(zipped.read(name)).hexdigest() == value for name, value in inventory.items())
    record = dict(prepared_only=True, files=len(inventory), archive_bytes=archive.stat().st_size,
        archive_sha256=digest(archive), manifest_sha256=digest(manifest_path),
        candidate_chain_complete=True, source_bytes_verified=True,
        jetson_accessed=False, native_built=False, timed_rebase_calls=0,
        raw_in_git=False, archive=archive.name)
    with (A/'bundle_receipt.json').open('x', encoding='utf8') as stream:
        json.dump(record, stream, ensure_ascii=False, indent=2); stream.write('\n')
    print(json.dumps(record))


if __name__ == '__main__': main()
