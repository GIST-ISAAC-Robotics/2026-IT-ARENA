"""기존 희소 상부와 마커 셀 생성기를 재사용하는 평가 전용 SDF 어댑터."""
from pathlib import Path
import math
import sys
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'scripts'))
from experimental_facilities import box,marker_cells,WHITE,BLACK

ACTOR_OVERRIDES=dict(visual_outline='sparse_upper_posts',height_m=.22,bottom_m=.02)
MARKER=dict(dictionary='DICT_4X4_50',id=1,board_m=.05,black_m=.0348,
            world_center_z_m=.20,rear_x_m=-.10,white_face_x_m=-.0996)


def attach_marker(actor_text):
    """actor 자세는 평가기만 소유한다. 센서/제어 토픽·반환은 생성하지 않는다."""
    root=ET.fromstring(actor_text)
    model=root.find("model[@name='eval_g1b_actor']")
    if model is None:raise ValueError('one named evaluation actor required')
    link=model.find("link[@name='body']")
    size=list(map(float,link.findtext('collision/geometry/box/size').split()))
    if size!=[.2,.15,.22]:raise ValueError('registered 200x150x220mm envelope required')
    expected={'visual_lower_body','visual_top',*[f'visual_post_{i}' for i in range(4)]}
    if {v.get('name') for v in link.findall('visual')}!=expected:
        raise ValueError('existing sparse_upper_posts geometry required; no duplicate marker')
    model_z=float(model.findtext('pose').split()[2])
    if not math.isclose(model_z,.13,abs_tol=1e-9):raise ValueError('registered 20mm bottom required')
    z=MARKER['world_center_z_m']-model_z
    # 코드 바깥 면을 실제 차체 후단 x=-.1에 맞춘다. 흰 판은0.4mm 안쪽에 둔다.
    # 모든 마커 형상은 기존 보수 collision 외곽 안에 있어 평가 외곽이 변하지 않는다.
    box(link,'r537_marker_white_board',(-.0991,0.,z),(.001,.05,.05),WHITE,rpy=(0,0,math.pi))
    cells=marker_cells(MARKER['id']);cell=MARKER['black_m']/6
    for row in range(6):
        for col in range(6):
            if cells[row,col]!=0:continue
            local_y=-MARKER['black_m']/2+(col+.5)*cell
            box(link,f'r537_marker_ink_{row}_{col}',(-.0998,-local_y,z+MARKER['black_m']/2-(row+.5)*cell),
                (.0004,cell,cell),BLACK,rpy=(0,0,math.pi))
    return ET.tostring(root,encoding='unicode')


def wrap_actor_factory(factory):
    """다음 실행 사본의 actor_sdf에만 설치한다. 공통 생성기는 수정하지 않는다."""
    def wrapped(config,pose):
        actor=config['actor']
        if any(actor.get(k)!=v for k,v in ACTOR_OVERRIDES.items()):
            raise ValueError('explicit sparse actor config required')
        return attach_marker(factory(config,pose))
    return wrapped
