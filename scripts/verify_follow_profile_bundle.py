#!/usr/bin/env python3
"""G1 비용 시험 묶음의 내용/해시 검사. 추출·전송·원격 실행·모터 연결은 하지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile

from python_dependency_closure import MissingInternalDependency, collect, third_party


def digest(stream):
    value = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024*1024), b''):
        value.update(chunk)
    return value.hexdigest()


def verify(archive, manifest):
    if manifest.get('bundle_mode') != 'follow-profile-only' or not manifest.get('includes_follow_traces'):
        raise ValueError('explicit follow-profile-only bundle required')
    if manifest.get('includes_recording'):
        raise ValueError('generic sensor bag is outside this minimal bundle')
    with Path(archive).open('rb') as stream:
        archive_matches = digest(stream) == manifest['archive_sha256']
    if Path(archive).stat().st_size != manifest['archive_size']:
        archive_matches = False
    expected = manifest['files']
    seen, problems, trace_headers, sources = set(), [], [], {}
    with tarfile.open(archive, 'r:gz') as bundle:
        for member in bundle:
            path = PurePosixPath(member.name)
            if (not member.isfile() or path.is_absolute() or '..' in path.parts or '\\' in member.name
                    or member.name in seen or ':' in member.name):
                raise ValueError('unsafe, duplicate or non-regular archive member')
            seen.add(member.name)
            if member.name not in expected:
                problems.append(dict(file=member.name, reason='unexpected_member'))
                continue
            with bundle.extractfile(member) as stream:
                checksum = digest(stream)
            if member.size != expected[member.name]['size'] or checksum != expected[member.name]['sha256']:
                problems.append(dict(file=member.name, reason='member_hash_or_size_mismatch'))
            if member.name.endswith('.py'):
                with bundle.extractfile(member) as stream:
                    sources[member.name] = stream.read()
            if member.name.startswith('artifacts/') and member.name.endswith('.jsonl'):
                with bundle.extractfile(member) as stream:
                    header = json.loads(stream.readline(65537))
                data = header.get('data', {})
                supported = data.get('scope') == 'g1_decision_stage_only' or (
                    data.get('schema') == 5 and data.get('scope') == 'g1_g2_handoff_decision_stage_only'
                    and data.get('controller_mode') == 'g1_g2_handoff_trial')
                if header.get('kind') != 'header' or not supported:
                    problems.append(dict(file=member.name, reason='not_decision_trace'))
                trace_headers.append(member.name)
    missing = sorted(set(expected)-seen)
    closure = closure_check(manifest, sources)
    if closure.get('problem'):
        problems.append(dict(file='<dependency_closure>', reason=closure['problem']))
    passed = archive_matches and not problems and not missing and bool(trace_headers)
    return dict(passed=passed, archive_hash_matches=archive_matches, members=len(seen),
                missing=missing, problems=problems, decision_traces=trace_headers, dependency_closure=closure,
                extracted=False, transferred=False, remote_execution_verified=False,
                scope='local_package_content_integrity_only_not_jetson_runtime_or_sensor_validation')


def closure_check(manifest, sources):
    """묶음 안의 .py만으로 진입점 import를 다시 닫는다. 기록 없는 이전 묶음은 미검사로 표시한다."""
    spec = manifest.get('dependency_closure')
    if spec is None:
        return dict(checked=False)
    try:
        closure = collect(spec['entries'], roots=spec['roots'], members=sources)
    except MissingInternalDependency as error:
        return dict(checked=True, problem=f'missing_internal_dependency: {error}')
    extra = sorted(set(third_party(closure))-set(spec['allowed_third_party']))
    if extra:
        return dict(checked=True, problem=f'unexpected_third_party: {extra}')
    # 표준 라이브러리 분류는 Python 판마다 다를 수 있으므로 외부 이름 집합만 대조한다.
    if set(closure['external']) != set(spec['external']) or len(closure['files']) != spec['files']:
        return dict(checked=True, problem='closure_differs_from_manifest')
    return dict(checked=True, files=len(closure['files']), third_party=third_party(closure))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('new output required')
    result = verify(args.archive, json.loads(args.manifest.read_text(encoding='utf-8')))
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
