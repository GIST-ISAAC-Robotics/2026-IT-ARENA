"""Windows spawn용 실제 파일 진입점. 장치/성능 실행이 아닌 선택 pytest만 호출."""
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
             str(ROOT/'scripts'),str(ROOT/'tests')]
import arena_autonomy
arena_autonomy.__path__.insert(0,str(HERE/'stage/arena_autonomy'))

if __name__=='__main__':
    import pytest
    raise SystemExit(pytest.main(['-q','-p','no:cacheprovider',*sys.argv[1:]]))
