"""새 배포본의 실제 Gazebo 렌더링 확인. 주행시험은 하지 않는다."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
import capture_official_track_views as capture

tree = ET.parse(ROOT / 'src/arena_gazebo/worlds/it_arena_official/world.sdf')
pose = [float(x) for x in tree.findtext("./world/model/link[@name='safety_bump_0']/pose").split()]
x, y, z = pose[:3]
capture.VIEWS = (capture.VIEWS[0], capture.VIEWS[1],
    dict(name='bump_detail', label='공개 2분할 방지턱', position=(x+.4, y-.7, z+.7), target=(x,y,z+.005)))
raise SystemExit(capture.main())
