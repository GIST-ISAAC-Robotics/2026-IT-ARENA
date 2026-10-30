"""G2 통로 명시 연결의 후속 소스 계약을 만든다. 소스가 다시 바뀌면 파일을 지우고 다시 만든다."""
import hashlib
import json
from pathlib import Path
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
NAME=ROOT/'config/tests/g2_corridor_r1_sha256.json'
PARENT='config/tests/fresh_confirmation_r2_sha256.json'
SNAPSHOT=BASE/'before_connection.zip'
REASONS={'src/arena_autonomy/arena_autonomy/bypass_entry_trial.py':'선택형 padding_provider. 기본 None은 기존 함수 호출과 같다',
    'src/arena_autonomy/arena_autonomy/bypass_entry_follow.py':'명시 corridor_policy. 기본 off는 기존 EntryGuide를 만든다',
    'src/arena_autonomy/arena_autonomy/g2_entry_node.py':'g2_corridor_policy 파라미터. 기본 off'}


def sha(data):
    return hashlib.sha256(data).hexdigest()


changes={}
with zipfile.ZipFile(SNAPSHOT) as archive:
    for path,reason in REASONS.items():
        changes[path]=dict(before=sha(archive.read(path)),after=sha((ROOT/path).read_bytes()),reason=reason)
contract=dict(schema_version=1,
    scope='G2 중앙 통로 시험 후보의 명시 기본 OFF 연결. 기본 경로·경기 기본값·실차 위험 수용 변경이 아니다.',
    parent_contract=PARENT,parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
    before_snapshot=SNAPSHOT.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(SNAPSHOT.read_bytes()),
    approved_changes=changes,approved_evaluation_scripts={},
    new_dependencies={'src/arena_autonomy/arena_autonomy/bypass_corridor_trial.py':
        sha((ROOT/'src/arena_autonomy/arena_autonomy/bypass_corridor_trial.py').read_bytes())})
NAME.write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('written',NAME.name)
