"""현행 native/import/선택 입력 사전 검사. evaluator 계산은 호출하지 않는다."""
import json
import platform
import sys
from support import HERE, ROOT, digest, import_receipt, verify_manifest, write_new


def main():
    manifest=verify_manifest()
    from arena_autonomy.race_runtime_policy.evaluator import Evaluator
    from arena_autonomy.runtime_dual_process import Lane
    from arena_autonomy.numeric_kernels import configure
    from arena_autonomy import _numeric_kernels
    configure('native_v1')
    assert _numeric_kernels.ABI_VERSION==1
    assert all(hasattr(_numeric_kernels,n) for n in ('depth','masked','classes','polygon'))
    selection=json.loads((HERE/'inputs/selection.json').read_bytes())
    assert selection['plan_count']==37 and selection['current_count']==153
    assert selection['recorded_candidate_applied_indices']==[137]
    imported=import_receipt()
    report=dict(passed=True,machine=platform.machine(),python=sys.version,backend='native_v1',
        native_abi=1,imports=imported,manifest_sha256=digest(ROOT/'source_manifest.json'),
        source_files=len(manifest['files']),control_calls=0,devices_opened=False)
    write_new(HERE/'import_preflight.json',report)
    print(json.dumps({k:v for k,v in report.items() if k!='imports'}))


if __name__=='__main__':main()
