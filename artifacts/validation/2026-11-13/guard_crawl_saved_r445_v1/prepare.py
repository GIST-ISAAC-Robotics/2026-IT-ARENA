"""r441 동결 입력만 r445 독립 stage에 보존한다."""
import hashlib
import json
from pathlib import Path
import zipfile
import sys

B = Path(__file__).resolve().parent
R441 = B.parent / 'crop_crawl_drive_r441_v1'
version = sys.argv[1] if len(sys.argv) > 1 else 'v1'
assert version in ('v1', 'v2', 'v3')
S = B / ('stage' if version == 'v1' else 'stage_' + version)
S.mkdir(exist_ok=False)
def sha(data): return hashlib.sha256(data).hexdigest()
expected_archive = 'c032a4cf5b689bde9e150103a65847d783f9d64c56c2b15513840345d653ee5d'
assert sha((R441 / 'sources.zip').read_bytes()) == expected_archive
files = {}
with zipfile.ZipFile(R441 / 'sources.zip') as z:
    for name in z.namelist():
        if (name.startswith('src/arena_autonomy/arena_autonomy/') or name.startswith('src/arena_vehicle_interface/arena_vehicle_interface/')) and (name.endswith('.py') or name.endswith('.so')):
            files[name] = z.read(name)
files['inputs/follow_decisions.jsonl'] = (R441 / 'run1/follow_decisions.jsonl').read_bytes()
files['replay_guard.py'] = (B / 'replay_guard.py').read_bytes()
manifest = {name: sha(data) for name, data in files.items()}
for name, data in files.items():
    target = S / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
(S / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf8')
archive = B / ('sources_' + version + '.zip')
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as z:
    for name, data in files.items(): z.writestr(name, data)
    z.writestr('manifest.json', json.dumps(manifest, indent=2))
receipt = dict(source='r441 frozen sources.zip', source_archive_sha256=expected_archive,
    trace_sha256=manifest['inputs/follow_decisions.jsonl'], staged_files=len(files),
    stage_archive_sha256=sha(archive.read_bytes()), original_bytes_preserved=True,
    common_build_install_import=False, shared_files_modified=False)
(B / ('prepared.json' if version == 'v1' else 'prepared_' + version + '.json')).write_text(json.dumps(receipt, indent=2) + '\n', encoding='utf8')
print(json.dumps(receipt))
