"""호환 import 경로. 같은 마스크에 0.10m/s 재시도를 추가하지 않는다."""
from . import solo_gap as prior


def rebase(output, submitted, current, scene, history, cfg, *, require_current_region=False):
    return prior.rebase(output, submitted, current, scene, history, cfg,
                        require_current_region=require_current_region)
