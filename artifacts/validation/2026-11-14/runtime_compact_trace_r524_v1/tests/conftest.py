"""stage 두 모듈만 우선하고 나머지 의존성은 현재 저장소를 읽는다."""
from pathlib import Path
import sys

ARTIFACT = Path(__file__).resolve().parents[1]
ROOT = ARTIFACT.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'),
                str(ROOT/'scripts'), str(ROOT/'tests')]
import arena_autonomy
arena_autonomy.__path__.insert(0, str(ARTIFACT/'stage/arena_autonomy'))
