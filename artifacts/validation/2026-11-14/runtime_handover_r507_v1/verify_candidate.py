"""r507 후보 소스 ZIP의 로컬 무결성/제외 경로/기준 대비 차이 확인. 장치 접근 없음."""
import hashlib
import importlib.util
import json
from pathlib import Path, PurePosixPath
import re
import zipfile

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
OLD = HERE.parent/'runtime_handover_r491_v1'


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def save(name, value):
    with (HERE/name).open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def main():
    provenance = load(HERE/'provenance.json')
    before = load(HERE/'sources_before.json')
    baseline = load(OLD/'final_v1/manifest.json')
    candidate = load(HERE/'candidate_v1/manifest.json')
    script = HERE/'stage/scripts/prepare_runtime_handover.py'
    spec = importlib.util.spec_from_file_location('original_handover_tool', script)
    prepare = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(prepare)
    current = prepare.inventory(prepare.collect(ROOT))

    added = sorted(candidate['files'].keys()-baseline['files'].keys())
    removed = sorted(baseline['files'].keys()-candidate['files'].keys())
    changed = sorted(name for name in baseline['files'].keys() & candidate['files'].keys()
                     if baseline['files'][name] != candidate['files'][name])
    delta = dict(
        baseline='r500 installed source: r491/final_v1',
        baseline_archive_sha256=provenance['old_archive_sha256'],
        baseline_manifest_sha256=provenance['old_manifest_sha256'],
        added=[dict(path=name, before_sha256=None, after_sha256=candidate['files'][name]) for name in added],
        removed=[dict(path=name, before_sha256=baseline['files'][name], after_sha256=None) for name in removed],
        changed=[dict(path=name, before_sha256=baseline['files'][name],
                      after_sha256=candidate['files'][name]) for name in changed])
    save('delta_from_r500.json', delta)

    zip_path = HERE/'candidate_v1/runtime_handover_sources.zip'
    prohibited_extensions = {'.so', '.dll', '.exe', '.elf', '.bin', '.a', '.o', '.pyd', '.pyc',
                             '.pem', '.key', '.p12', '.pfx', '.bag', '.db3', '.mcap'}
    prohibited_parts = prepare.SKIP | {'outputs', 'output', 'artifacts', 'node_modules', '.ssh'}
    prohibited_names = {'.env', '.netrc', 'id_rsa', 'id_ed25519', 'credentials.json'}
    # 실제 secret 내용을 출력하지 않는다. 일반 password/token 식별자나 빈 서식은 비밀로 판정하지 않는다.
    secret_pattern = re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|'
                                rb'\bsk-proj-[A-Za-z0-9_-]{30,}|'
                                rb'\bgh[pousr]_[A-Za-z0-9]{30,}|'
                                rb'\bAKIA[A-Z0-9]{16}\b')
    binary_magic = (b'\x7fELF', b'MZ', b'\xcf\xfa\xed\xfe', b'\xfe\xed\xfa\xcf')
    with zipfile.ZipFile(zip_path) as archive:
        names = archive.namelist()
        forbidden_paths = []
        binary_payloads = []
        secret_signature_paths = []
        mismatch = []
        for name in names:
            p = PurePosixPath(name)
            if (p.is_absolute() or '..' in p.parts or '\\' in name
                    or prohibited_parts.intersection(p.parts)
                    or p.suffix.lower() in prohibited_extensions or p.name in prohibited_names):
                forbidden_paths.append(name)
            raw = archive.read(name)
            if raw.startswith(binary_magic):
                binary_payloads.append(name)
            if secret_pattern.search(raw):
                secret_signature_paths.append(name)
            if name in candidate['files'] and sha(raw) != candidate['files'][name]:
                mismatch.append(name)
        embedded = json.loads(archive.read('source_manifest.json'))
        crc_error = archive.testzip()
    expected_names = set(candidate['files']) | {'source_manifest.json'}
    tool_source = OLD/'stage/scripts/prepare_runtime_handover.py'
    receipt = dict(
        source_only=True, install_pending=True, adoption_approved=False,
        r503_simulation_review_pending=True, device_io=False, firmware_written=False,
        archive_sha256=sha(zip_path.read_bytes()),
        manifest_sha256=sha((HERE/'candidate_v1/manifest.json').read_bytes()),
        archive_bytes=zip_path.stat().st_size, source_file_count=len(candidate['files']),
        zip_member_count=len(names), required_missing=candidate['required_missing'],
        zip_crc_error=crc_error, zip_sha_mismatch_paths=mismatch,
        zip_member_set_exact=set(names) == expected_names, duplicate_zip_members=len(names)-len(set(names)),
        embedded_manifest_matches={k:v for k,v in candidate.items() if k != 'archive_sha256'} == embedded,
        sources_before_equal=candidate['files'] == before['files'],
        sources_after_equal=candidate['files'] == current['files'],
        public_resource_count=len(candidate['public_resources']),
        public_resources_unmodified=all(candidate['files'][name] == baseline['files'][name]
                                      for name in candidate['public_resources']),
        installed_values=candidate['installed_values'],
        old_archive_unchanged=sha((OLD/'final_v1/runtime_handover_sources.zip').read_bytes()) == provenance['old_archive_sha256'],
        old_manifest_unchanged=sha((OLD/'final_v1/manifest.json').read_bytes()) == provenance['old_manifest_sha256'],
        tool_byte_identical=sha(script.read_bytes()) == sha(tool_source.read_bytes()) == provenance['tool_sha256'],
        forbidden_paths=forbidden_paths, binary_magic_paths=binary_payloads,
        known_secret_signature_paths=secret_signature_paths,
        delta_counts=dict(added=len(added), removed=len(removed), changed=len(changed)),
        scanner_scope='path exclusions, ELF/PE/Mach-O headers, private key and named provider key patterns; not a general secret proof')
    receipt['ok'] = (
        all(receipt[key] for key in ('zip_member_set_exact', 'embedded_manifest_matches',
            'sources_before_equal', 'sources_after_equal', 'public_resources_unmodified',
            'old_archive_unchanged', 'old_manifest_unchanged', 'tool_byte_identical'))
        and not any((crc_error, mismatch, forbidden_paths, binary_payloads, secret_signature_paths,
                     receipt['duplicate_zip_members'], receipt['required_missing']))
        and all(value is None for value in candidate['installed_values'].values())
        and receipt['archive_sha256'] == candidate['archive_sha256'])
    save('verification_receipt.json', receipt)
    print(json.dumps({key:receipt[key] for key in ('ok','source_file_count','archive_sha256',
        'manifest_sha256','delta_counts','install_pending')}, indent=2))
    print(json.dumps({'added':added,'removed':removed,'changed':changed}, indent=2))
    return 0 if receipt['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
