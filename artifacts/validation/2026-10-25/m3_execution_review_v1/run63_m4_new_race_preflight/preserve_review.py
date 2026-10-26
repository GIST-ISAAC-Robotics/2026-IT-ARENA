from pathlib import Path
import zipfile
out=Path('artifacts/validation/2026-10-25/m3_execution_review_v1/run63_m4_new_race_preflight/source_before_review.zip')
assert not out.exists()
with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
    for p in ['scripts/m4_operation_trial.py','scripts/m4_operation_eval.py','scripts/validate_g1_traffic.py','config/tests/m4_new_race_r0_sha256.json','tests/test_m4_operation_trial.py']:
        z.write(p,p)