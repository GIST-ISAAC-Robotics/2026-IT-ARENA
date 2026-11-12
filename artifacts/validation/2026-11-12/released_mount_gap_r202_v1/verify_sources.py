"""직전 전체 계보202검사 이후 변경한 계약과 원 M3 기준 연결만 대조한다."""
import hashlib
from pathlib import Path
import test_source_provenance_r0_plus_b3 as source
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
assert source.M3_SUCCESSORS[-1]=='config/tests/released_mount_gap_r202_sha256.json'
source.test_registered_oct26_and_nov01_successors_have_preserved_source_chain(len(source.M3_SUCCESSORS)-1)
source.test_m3_successor_records_current_bytes_without_rewriting_previous_contract()
assert (ROOT/'src/arena_autonomy/arena_autonomy/released_body_gap.py').read_bytes()==(A/'candidate.py').read_bytes()
print('latest source contract, original M3 chain, and tested candidate bytes: PASS')
