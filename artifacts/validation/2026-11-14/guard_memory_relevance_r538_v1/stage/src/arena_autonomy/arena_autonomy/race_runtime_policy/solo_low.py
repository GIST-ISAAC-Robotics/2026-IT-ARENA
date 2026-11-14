"""호환 import 경로. SOLO 마스크 저속 재시도는 현재 구조 그림자 판정으로 대체했다."""
from . import current_region as base


def rebase(output, submitted, current, scene, history, cfg, *, require_current_region=False):
    return base.rebase(output, submitted, current, scene, history, cfg,
                       require_current_region=require_current_region)
