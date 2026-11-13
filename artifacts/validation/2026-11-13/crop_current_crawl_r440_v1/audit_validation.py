"""r440 종료 후 동결자료/공통 target SHA/PID를 읽기 전용 확인한다."""
from pathlib import Path
import hashlib
import json
import os
import zipfile

bundle=Path(__file__).resolve().parent;repo=bundle.parents[3]
root=Path('/home/jinhyeong/it-arena-r440-verify-a')
manifest=json.loads((root/'manifest.json').read_bytes())
def sha(data):return hashlib.sha256(data).hexdigest()
changed=[name for name,want in manifest.items() if sha((root/name).read_bytes())!=want]
target='src/arena_autonomy/arena_autonomy/race_runtime_policy/crop_current_candidate.py'
with zipfile.ZipFile(bundle.parent/'applied_limit_drive_r437_v1/sources.zip') as z:old=z.read(target)
original_exact=old==(bundle/'original_crop_current_candidate.py').read_bytes()
common_exact=sha((repo/target).read_bytes())==manifest[target]=='886f4875a11cc5f7cc0af4924a99717a323464542ffb4fa48e45f218fe01d9e5'
owned=[]
for item in Path('/proc').glob('[0-9]*'):
    if int(item.name)==os.getpid():continue
    try:command=(item/'cmdline').read_bytes().replace(b'\x00',b' ').decode(errors='replace')
    except OSError:continue
    if 'it-arena-r440-verify-' in command or 'verify_saved.py' in command or 'run_validation.py' in command:
        owned.append(dict(pid=int(item.name),command=command))
result=dict(frozen_files=len(manifest),changed=changed,common_target_exact=common_exact,
    original_r437_archive_exact=original_exact,owned_processes=owned,
    passed=not changed and common_exact and original_exact and not owned)
(bundle/'handoff_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
