"""종료 후 회수한 r536 폴더를 새 폴더에 보존하고 status/sidecar를 함께 검증·복원한다."""
import argparse
from collections import Counter
import gzip
import json
from pathlib import Path, PurePosixPath
import shutil

from prepare import SIDECAR, sha
from restore_context import load_details, restore_status


def collect(selection_path, result_directory, destination):
    selection_path, result_directory, destination = map(Path, (selection_path, result_directory, destination))
    selection = json.loads(selection_path.read_bytes())
    if selection['schema'] != 'r547_guard_status_recipe_v1':
        raise ValueError('unknown selection schema')
    profile = selection_path.parent/'profile.json'
    if sha(profile) != selection['selected_profile_sha256']:
        raise ValueError('selected profile changed')
    destination = destination.resolve()
    source = result_directory.resolve(strict=True)
    if destination.is_relative_to(source):
        raise ValueError('collection destination must be outside the source result')
    destination.mkdir(parents=True, exist_ok=False)
    raw = destination/'runtime'
    shutil.copytree(source, raw)  # 기존 결과 폴더 전체 회수: status만 떼어내 상세를 잃지 않는다.
    shutil.copy2(selection_path, destination/'selection.json')
    shutil.copy2(profile, destination/'profile.json')
    receipt = dict(schema='r547_collection_v1', requested_guard_detail=selection['requested_guard_detail'],
                   restored=False, error=None, mode_counts={}, archive_ids=[])
    try:
        invocation = json.loads((raw/'invocation.json').read_bytes())
        if invocation['profile_sha256'] != selection['selected_profile_sha256']:
            raise ValueError('run used a different selected profile')
        if selection['requested_guard_detail'] == 'compact_v1':
            expected = PurePosixPath(selection['target_result'])/SIDECAR
            if str(expected) != selection['target_sidecar']:
                raise ValueError('sidecar is outside this selected run')
        sidecar = raw/SIDECAR
        records = load_details(sidecar) if sidecar.exists() else {}
        mode_counts, storage_counts = Counter(), Counter()
        restored_rows, archive_ids = [], set()
        references = 0
        with gzip.open(raw/'guard_status.jsonl.gz', 'rt', encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                value = row['value']
                metadata = value.get('status_detail')
                evidence = (value.get('partial_observation') or {}).get('latch_evidence') or {}
                ref = evidence.get('correspondence_context_ref')
                mode_counts['compact_v1' if metadata else 'full'] += 1
                if metadata:
                    archive = metadata['archive']
                    if archive['path'] != SIDECAR or not sidecar.exists():
                        raise ValueError('compact status archive was not collected')
                    archive_ids.add(archive['archive_id'])
                    storage_counts[metadata['context_storage']] += 1
                    if ref:
                        if ref['archive_id'] != archive['archive_id']:
                            raise ValueError('status archive ID mismatch')
                        references += 1
                elif ref:
                    raise ValueError('reference without compact metadata')
                restored_rows.append({**row, 'value':restore_status(value, records)})
        if len(archive_ids) > 1:
            raise ValueError('more than one guard archive in one bounded run')
        if records and {key[0] for key in records} != archive_ids:
            # 초기화 뒤 full fallback만 남으면 참조 없는 상세가 있을 수 있다. 성공으로 가장하지 않는다.
            if archive_ids:
                raise ValueError('sidecar contains another run archive')
        requested = selection['requested_guard_detail']
        if requested == 'full' and (mode_counts['compact_v1'] or sidecar.exists()):
            raise ValueError('full baseline unexpectedly enabled compact archive')
        if not restored_rows:
            raise ValueError('empty status capture')
        with gzip.open(destination/'guard_status.restored.jsonl.gz', 'xt', encoding='utf-8') as stream:
            for row in restored_rows:
                stream.write(json.dumps(row, allow_nan=False)+'\n')
        receipt.update(restored=True, rows=len(restored_rows), mode_counts=dict(mode_counts),
            storage_counts=dict(storage_counts), archive_ids=sorted(archive_ids), reference_rows=references,
            detail_records=len(records), sidecar_present=sidecar.exists(),
            compact_reduction_observed=references > 0,
            compact_with_full_rows=bool(mode_counts['compact_v1'] and mode_counts['full']),
            requested_compact_but_unobserved=requested == 'compact_v1' and not mode_counts['compact_v1'])
    except Exception as error:
        receipt['error'] = type(error).__name__+': '+str(error)
    receipt['files'] = {str(p.relative_to(destination)):sha(p) for p in destination.rglob('*') if p.is_file()}
    (destination/'collection.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    if receipt['error']:
        raise ValueError(receipt['error'])
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection', required=True, type=Path)
    parser.add_argument('--result-directory', required=True, type=Path)
    parser.add_argument('--destination', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(collect(args.selection, args.result_directory, args.destination), indent=2))


if __name__ == '__main__':
    main()
