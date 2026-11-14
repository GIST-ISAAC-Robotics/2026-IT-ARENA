"""희소 상부는 보이는 면만 변경하며 실제 차체 평가 외곽은 줄이지 않는다."""
from copy import deepcopy
import xml.etree.ElementTree as ET

import pytest
from shapely.geometry import box
from shapely.ops import unary_union

import validate_g1_traffic as harness
from test_traffic_actor_outline import ROOT


def config(outline):
    cfg = harness.load_config(ROOT/'config/tests/official_traffic_v1.json')
    cfg['actor'].update(visual_outline=outline, height_m=.22, bottom_m=.02)
    return cfg


def sections(root, world_z):
    link = root.find('model/link')
    model_z = float(root.findtext('model/pose').split()[2])
    shapes = []
    for v in link.findall('visual'):
        x, y, z, _, _, _ = map(float, v.findtext('pose', '0 0 0 0 0 0').split())
        length, width, height = map(float, v.findtext('geometry/box/size').split())
        if abs(world_z-model_z-z) < height/2:
            shapes.append(box(x-length/2, y-width/2, x+length/2, y+width/2))
    return unary_union(shapes)


@pytest.mark.parametrize('outline,components,area', [('thin_upper_blade', 1, .004*.12),
                                                    ('sparse_upper_posts', 4, 4*.008**2)])
def test_plane_is_sparse_but_lower_body_and_collision_are_full(outline, components, area):
    cfg = config(outline)
    root = ET.fromstring(harness.actor_sdf(cfg, (0, 0, 0)))
    collision = root.findtext('model/link/collision/geometry/box/size')
    assert list(map(float, collision.split())) == [.2, .15, .22]
    cut = sections(root, .1346)
    assert cut.area == pytest.approx(area)
    assert (len(cut.geoms) if hasattr(cut, 'geoms') else 1) == components
    assert sections(root, .05).area == pytest.approx(.2*.15)
    assert box(-.1, -.075, .1, .075).covers(cut)


def test_sparse_fixture_is_not_silently_applied_to_existing_shapes():
    cfg = config('box')
    assert len(ET.fromstring(harness.actor_sdf(cfg, (0, 0, 0))).findall('model/link/visual')) == 1
    cfg['actor'].update(visual_outline='sparse_upper_posts', height_m=.12)
    with pytest.raises(ValueError, match='height'):
        harness.actor_sdf(cfg, (0, 0, 0))
