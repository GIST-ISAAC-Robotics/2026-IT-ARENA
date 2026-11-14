"""root 인계의 정확한 SHA를 확인하고 r515 동결 준비를 마무리한다. 주행 없음."""
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')


def main():
    assert not any((A / n).exists() for n in ('prepared.json', 'registered.json', 'run1'))
    hashes = {
        'src/arena_autonomy/arena_autonomy/moving_pass.py': '699a9ebee9fca51bd730ab305f7b9c189b160e72f7b3c2e94c4aebad9e064c77',
        'tests/test_moving_path_preview.py': '3bd598055fff12130a2e42fb27832b9b16ed697e47ba4a5859a398df509c9371',
    }
    validation_path = A.parent / 'current_guide_extent_r514_v1/validation.json'
    validation = read(validation_path)
    assert validation['files'] == hashes and validation['tests_passed'] == 130
    for name, digest in hashes.items():
        assert sha(ROOT / name) == digest, name
    selection = read(A / 'source_selection.json')
    selection['control_changes'][0]['after_sha256'] = hashes[selection['control_changes'][0]['path']]
    selection['control_changes'][0]['role'] = 'r514 current-forward-guide coverage reviewed by root (GPT)'
    selection.update(freeze_authorized=True, wsl_exclusive_owner=True, drive_authorized=True,
        status='Root reviewed recipe and candidate SHA; exclusive WSL handed over; freeze then single run delegated',
        root_authorization='2026-11-14 root: after import/SHA/competing0 and freeze integrity checks, delegated runner records exact review scope and executes one representative; no repeat',
        reviewed_source_hashes=hashes, relevant_tests=130,
        test_evidence=validation_path.relative_to(ROOT).as_posix())
    selection['included_integrations'][-1] = 'r514 current-forward-guide coverage; 124 prior calls matched, 80 zero-to-positive, no positive-to-zero; 130 related tests passed'
    save(A / 'source_selection.json', selection)
    provenance = read(A / 'recipe_provenance.json')
    for name in ('RESULT.md', 'validation.json', 'candidate_replay.json', 'diagnostic.json'):
        p = validation_path.parent / name
        provenance['original_evidence'][p.relative_to(ROOT).as_posix()] = sha(p)
    save(A / 'recipe_provenance.json', provenance)
    print(json.dumps(dict(root_sha_verified=True, relevant_tests_evidence=130,
                         freeze_authorized=True, exclusive_WSL_handover=True, actual_run_count=0)))


if __name__ == '__main__':
    main()
