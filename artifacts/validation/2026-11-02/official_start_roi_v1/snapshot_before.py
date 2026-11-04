"""검출 영역 수정 전 바이트를 보존한다. simulation.launch.py는 run1 당시 LF 바이트 그대로 묶고 저장소 형식(CRLF)으로 복원한다."""
from hashlib import sha256
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
FILES = ('src/arena_autonomy/arena_autonomy/race_start.py', 'src/arena_autonomy/arena_autonomy/race_start_local.py',
         'src/arena_bringup/launch/simulation.launch.py')
LAUNCH = ROOT/FILES[2]


def main():
    registered = json.loads((HERE.parent/'official_facility_v1/run1_registered.json').read_bytes())['source_sha256']
    data = LAUNCH.read_bytes()
    assert sha256(data).hexdigest() == registered[FILES[2]], 'run1 바이트가 아니다'
    with zipfile.ZipFile(HERE/'before.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            archive.writestr(zipfile.ZipInfo(path, (2026, 11, 2, 0, 0, 0)), (ROOT/path).read_bytes())
    assert b'\r\n' not in data
    LAUNCH.write_bytes(data.replace(b'\n', b'\r\n'))


if __name__ == '__main__':
    main()
