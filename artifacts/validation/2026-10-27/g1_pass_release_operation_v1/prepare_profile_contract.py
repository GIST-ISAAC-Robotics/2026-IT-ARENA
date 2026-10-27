"""프로파일러·최소 묶음 평가 스크립트의 기존 수정(run4 이전)을 새 후속 출처 계약으로 기록한다. 있으면 거절한다.

before 바이트는 git 0e6c28a(m5_publication 계약의 after와 같음)에서 꺼내 새 zip에 보존한다. 이전 계약·zip은 고치지 않는다.
제어기 변경은 없으며, 이 기록은 Jetson 실행 검증이 아니다.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
CONTRACT = ROOT/'config/tests/profile_bundle_selector_r0_sha256.json'
ZIP = HERE/'profile_contract_before.zip'
PARENT = 'config/tests/partial_observation_r0_sha256.json'
COMMIT = '0e6c28a'
REASONS = {
    'scripts/profile_follow_trace.py': '선택형 부분 관측·평행·통과 선택 모듈 4개와 원시 스캔 전달, 표면 정합 processing_ms만 비교 제외(10/27 GPT, run4 이전 반영)',
    'scripts/prepare_jetson_bundle.py': '최소 이관 묶음에 같은 새 모듈 의존성 보충(10/27 GPT, run4 이전 반영)'}


def sha(data):
    return hashlib.sha256(data).hexdigest()


if CONTRACT.exists() or ZIP.exists():
    raise FileExistsError('existing contract or snapshot is never overwritten')
parent = json.loads((ROOT/'config/tests/m5_publication_r0_sha256.json').read_text(encoding='utf-8'))
rows, blobs = {}, {}
for path, reason in REASONS.items():
    before = subprocess.run(['git', 'show', f'{COMMIT}:{path}'], cwd=ROOT, capture_output=True, check=True).stdout
    assert sha(before) == parent['approved_evaluation_scripts'][path]['after'], path
    after = sha((ROOT/path).read_bytes())
    assert after != sha(before), path
    rows[path] = {'before': sha(before), 'after': after, 'reason': reason}
    blobs[path] = before
with zipfile.ZipFile(ZIP, 'x', zipfile.ZIP_DEFLATED) as archive:
    for path, data in blobs.items():
        archive.writestr(path, data)
contract = {'schema_version': 1,
            'scope': '10/27 선택형 G1 모듈의 프로파일러·최소 이관 묶음 반영 출처. 제어기/기본값 변경 없음. Jetson 실행 검증 아님',
            'parent_contract': PARENT, 'parent_contract_sha256': sha((ROOT/PARENT).read_bytes()),
            'before_snapshot': ZIP.relative_to(ROOT).as_posix(), 'before_snapshot_sha256': sha(ZIP.read_bytes()),
            'before_source': f'git:{COMMIT}', 'approved_changes': {}, 'approved_evaluation_scripts': rows,
            'new_dependencies': {}}
CONTRACT.write_bytes((json.dumps(contract, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))
print(json.dumps(contract, ensure_ascii=False, indent=2))
