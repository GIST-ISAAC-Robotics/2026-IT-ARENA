"""현재 단독 판단 비용을 한 번 측정한다. 기존 비용 도구에 새 의존 소스만 보존한다."""
from hashlib import sha256
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
import profile_follow_trace as profiler

EXTRA = ('bypass_yield_progress', 'front_patch_motion', 'released_rear_sweep', 'turn_preview')


if __name__ == '__main__':
    if (HERE/'run1').exists():
        raise FileExistsError('existing timing run')
    source = Path(__file__).read_bytes()
    profiler.SNAPSHOT_MODULES['arena_autonomy'] = (*profiler.SNAPSHOT_MODULES['arena_autonomy'], *EXTRA)
    sys.argv = [str(ROOT/'scripts/profile_follow_trace.py'),
                str(HERE.parent/'solo_route_v1/run1/follow_decisions.jsonl'),
                '--input-mode','stream','--repeats','1','--warmups','0','--output',str(HERE/'run1')]
    result = profiler.main()
    manifest = dict(wrapper_sha256=sha256(source).hexdigest(),
                    wrapper_unchanged=source==Path(__file__).read_bytes(),
                    extra_snapshot_modules=EXTRA, exit_code=result,
                    scope='기록된 단독 한 랩의 판단 단계 비용. 기준 판단 전체 일치 후 측정 1회, 워밍업 0회. ROS/영상/보호/MCU/Jetson/실시간 마감 검증은 아니다.')
    with (HERE/'wrapper_manifest.json').open('x', encoding='utf-8') as f:
        json.dump(manifest,f,ensure_ascii=False,indent=2)
    raise SystemExit(result)
