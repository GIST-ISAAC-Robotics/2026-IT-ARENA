"""기존 시설 평가의 기준·fixture는 유지하고 출력 폴더만 명시한다."""
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'),str(ROOT/'scripts')]
from traffic_posts_assess import assess

if __name__=='__main__':
    raise SystemExit(assess(HERE/'two_actor_run1'))
