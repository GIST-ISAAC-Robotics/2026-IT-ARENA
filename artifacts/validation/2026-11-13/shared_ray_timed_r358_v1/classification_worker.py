"""시간 비교 전용: r353 정책과 분리/공유 계산만 선택한다."""
from pathlib import Path
import sys
A=Path(__file__).resolve().parent
sys.path.insert(0,str(A.parent/'final_geometry_crawl_drive_r353_v1'))

def worker(stage, *args):
    import low_solo_worker
    from arena_autonomy import pass_release_trial
    # overlap._worker 인자의 header 위치. 시험용 필드이며 센서/판정에는 쓰지 않는다.
    header=args[3]
    mode=header['r358_classification']
    if mode not in ('split','shared'):
        raise ValueError('explicit classification comparison required')
    original=pass_release_trial.depth_and_masked_support
    if mode=='split':
        pass_release_trial.depth_and_masked_support=lambda *a,**k:None
    try:
        low_solo_worker.worker(stage,*args)
    finally:
        pass_release_trial.depth_and_masked_support=original

def install():
    import overlap
    overlap._worker=worker
