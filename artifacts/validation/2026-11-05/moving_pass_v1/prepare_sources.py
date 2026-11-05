"""이동차 후보 연결 직전의 변경 대상 보존. 현재 파일이 달라지면 재실행하지 않는다."""
from pathlib import Path
import zipfile
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
PATHS=('src/arena_autonomy/arena_autonomy/bypass_handoff.py',
       'src/arena_autonomy/arena_autonomy/g1_g2_node.py',
       'src/arena_bringup/launch/simulation.launch.py',
       'scripts/g1_handoff_eval.py','scripts/validate_g1_traffic.py',
       'tests/test_source_provenance_r0_plus_b3.py')
with zipfile.ZipFile(HERE/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
    for name in PATHS: z.writestr(name,(ROOT/name).read_bytes())
print('preserved',len(PATHS),'files')
