"""11/9 신호등 양지주 시설: 실행 사본에만 추가하고 기본값·공식 링크·물성은 그대로 둔다."""
import copy
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import ast

import pytest
import yaml
from build_experimental_track import obstacle_polygons, rectangle
from test_g1_solo_candidate_validation import helpers

ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT/'src/arena_bringup/launch/simulation.launch.py'
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
SENSORS = ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'
spec = importlib.util.spec_from_file_location('signal_post_launch', PATH)
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)
TRACK = 'official_v2026_11_06'
MODE = 'stl_61cm_provisional'


def evaluator():
    """검증기의 지주 평가 함수만 읽는다(ROS·영상 의존성 없이)."""
    source = ROOT/'scripts/validate_basic_autonomy.py'
    names = ('hidden_course_markers', 'load_simulation_launch', 'signal_post_obstacles', 'signal_post_clearance')
    nodes = [n for n in ast.parse(source.read_text(encoding='utf-8')).body
             if isinstance(n, ast.FunctionDef) and n.name in names]
    scope = dict(Path=Path, REPO=ROOT, math=math, rectangle=rectangle, obstacle_polygons=obstacle_polygons)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), scope)
    return scope


def canonical(element):
    return ET.canonicalize(ET.tostring(element, encoding='unicode'), strip_text=True)


def links(root):
    return {link.get('name'): link for link in root.iter('link')}


def test_posts_keep_stl_sections_and_widen_only_the_span():
    root = ET.fromstring(WORLD.read_bytes())
    before = {name: canonical(link) for name, link in links(root).items()}
    report = launch._signal_posts(root.find('world'), MODE, TRACK)
    after = links(root)
    added = sorted(set(after) - set(before))
    assert added == ['tl_foot_left', 'tl_foot_right', 'tl_post_left', 'tl_post_right']
    assert all(canonical(after[name]) == text for name, text in before.items())
    placed = {item['name']: item for item in report['links']}
    # 공식 가로대 중심 x=9.80에서 STL 0.2925 m + (0.61-0.45)/2 = 0.3725 m
    assert placed['tl_post_left']['center_xy_m'] == [9.4275, 6.145]
    assert placed['tl_post_right']['center_xy_m'] == [10.1725, 6.145]
    for side in ('left', 'right'):
        post, foot = placed[f'tl_post_{side}'], placed[f'tl_foot_{side}']
        assert (post['across_m'], post['along_m'], post['bottom_m'], post['top_m']) == (.015, .01462, .01, .465)
        assert (foot['across_m'], foot['along_m'], foot['bottom_m'], foot['top_m']) == (.04, .05, 0., .01)
        for tag in ('collision', 'visual'):
            shape = after[f'tl_post_{side}'].find(tag)
            assert shape.findtext('geometry/box/size') == '0.01462 0.0150 0.4550'
        # 표면 마찰 등 새 물성은 넣지 않는다.
        assert after[f'tl_post_{side}'].find('collision/surface') is None


def test_post_faces_sit_in_grass_and_cross_the_c1_plane():
    report = launch._signal_posts(ET.fromstring(WORLD.read_bytes()).find('world'), MODE, TRACK)
    placed = {item['name']: item for item in report['links']}
    road_half, wall = .305, .505
    for side in ('left', 'right'):
        post, foot = placed[f'tl_post_{side}'], placed[f'tl_foot_{side}']
        offset = abs(post['center_xy_m'][0]-9.80)
        assert math.isclose(offset-post['across_m']/2-road_half, .06, abs_tol=1e-9)   # STL 45 cm 당시와 같은 6 cm
        assert offset+foot['across_m']/2 < wall-.10
    lidar_z = yaml.safe_load(SENSORS.read_text(encoding='utf-8'))['lidar']['xyz_m'][2]
    assert placed['tl_post_left']['bottom_m'] < lidar_z < placed['tl_post_left']['top_m']
    assert placed['tl_foot_left']['top_m'] < lidar_z


def test_none_is_noop_and_registration_is_strict():
    world = ET.fromstring(WORLD.read_bytes()).find('world')
    original = ET.tostring(world)
    assert launch._signal_posts(world, 'none', TRACK) is None
    assert ET.tostring(world) == original
    with pytest.raises(ValueError):
        launch._signal_posts(world, MODE, 'official')
    with pytest.raises(ValueError):
        launch._signal_posts(world, 'stl_45cm', TRACK)
    launch._signal_posts(world, MODE, TRACK)
    with pytest.raises(ValueError):
        launch._signal_posts(world, MODE, TRACK)
    legacy = ET.fromstring(WORLD.read_bytes()).find('world')
    ET.SubElement(legacy.find("model[@name='it_arena_track_static']"), 'link', name='tl_post')
    with pytest.raises(ValueError):
        launch._signal_posts(legacy, MODE, TRACK)


def test_runtime_copy_default_bytes_and_posts_copy(tmp_path):
    source_before = WORLD.read_bytes()
    launch._runtime_world_overrides(WORLD, tmp_path/'plain.sdf', 'configured', True, False)
    launch._runtime_world_overrides(WORLD, tmp_path/'none.sdf', 'configured', True, False, signal_posts='none', track=TRACK)
    assert (tmp_path/'plain.sdf').read_bytes() == (tmp_path/'none.sdf').read_bytes()
    report = launch._runtime_world_overrides(WORLD, tmp_path/'posts.sdf', 'configured', True, False,
                                             signal_posts=MODE, track=TRACK)
    assert report['signal_posts']['mode'] == MODE
    assert 'tl_post_left' in links(ET.parse(tmp_path/'posts.sdf').getroot())
    assert WORLD.read_bytes() == source_before


def test_evaluator_uses_same_posts_and_keeps_low_feet_out_of_body_obstacles(tmp_path):
    h = evaluator()
    base = obstacle_polygons(WORLD)
    obstacles, posts, feet, report = h['signal_post_obstacles'](WORLD, MODE, TRACK, tmp_path)
    assert len(obstacles) == len(base)+2 and len(posts) == 2 and len(feet) == 2
    assert report['links'][0]['name'] == 'tl_post_left'
    centered = [dict(x=9.80, y=y, yaw=math.pi/2, time=y) for y in (5.9, 6.145, 6.4)]
    clear = h['signal_post_clearance'](centered, posts, feet)
    assert clear['post_overlap_samples'] == 0 and math.isclose(clear['min_body_to_post_m'], .29, abs_tol=1e-6)
    # 차체 중심이 도로 끝에서 1.5 cm 안쪽(0.29 m)을 넘으면 지주에 닿는다.
    edge = [dict(x=9.80+.30, y=6.145, yaw=math.pi/2, time=0.)]
    hit = h['signal_post_clearance'](edge, posts, feet)
    assert hit['post_overlap_samples'] == 1


def test_route_completion_requires_only_unhidden_junction_markers():
    assert evaluator()['hidden_course_markers']('blank_20_30') == {20, 30}
    route = helpers()['solo_route_complete']
    assert route(49., 48., True, set(), 'blank_20_30')
    assert not route(40., 48., True, {0, 45}, 'blank_20_30')
    assert not route(49., 48., True, {20}, 'visible')
    assert route(49., 48., True, {20}, 'blank_30')


def test_course_marker_mode_hides_only_junction_markers():
    root = ET.fromstring(WORLD.read_bytes())
    original = copy.deepcopy(root)
    result = launch._evaluation_course_markers(root.find('world'), 'blank_20_30')
    assert result['hidden_ids'] == [20, 30]
    for marker in (0, 45):
        selector = f".//link[@name='aruco_{marker}']"
        assert canonical(root.find(selector)) == canonical(original.find(selector))
    for marker in (20, 30):
        assert root.find(f".//link[@name='aruco_{marker}']/visual/material").find('.//albedo_map') is None


@pytest.mark.parametrize('entry', ['local_pursuit.launch.py', 'demo.launch.py', 'simulation.launch.py'])
def test_junction_marker_drop_is_accepted_through_each_launch_entry(entry):
    # 실제 실패는 최상위 launch의 선택지 누락이었다. 노드를 시작하지 않고
    # ROS launch 자체의 인자 검증을 실행해 중첩 진입점 어느 곳에서도 거부하지 않음을 확인한다.
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    source = ROOT/'src/arena_bringup/launch'/entry
    spec = importlib.util.spec_from_file_location('marker_launch_'+entry.split('.')[0], source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declarations = [action for action in module.generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument)
                    and action.name == 'evaluation_course_markers']
    assert len(declarations) == 1
    context = LaunchContext()
    context.launch_configurations['evaluation_course_markers'] = 'blank_20_30'
    declarations[0].execute(context)
    assert context.launch_configurations['evaluation_course_markers'] == 'blank_20_30'


def test_solo_runner_adds_facility_flags_only_when_selected():
    spec = importlib.util.spec_from_file_location('new_course_solo', ROOT/'scripts/run_new_course_solo.py')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)

    class Opts:
        signal_posts, course_marker_visibility = 'none', 'visible'
    assert runner.facility_args(Opts) == []
    Opts.signal_posts, Opts.course_marker_visibility = MODE, 'blank_20_30'
    assert runner.facility_args(Opts) == ['--signal-posts', MODE, '--course-marker-visibility', 'blank_20_30']
    Opts.narrow_occluder_bridge = Opts.junction_continuity = True
    assert runner.facility_args(Opts)[-2:] == ['--g1-narrow-occluder-bridge', '--g1-junction-continuity']


def wall_scan(occluder=None, step=math.radians(.72)):
    """원점 라이다에서 왼쪽 벽 y=0.505 (x -0.3~3.0)와 선택 상자 가림체를 본 반환점(방위 순서)."""
    import numpy as np
    points = []
    for angle in np.arange(math.radians(5), math.radians(170), step):
        dx, dy = math.cos(angle), math.sin(angle)
        best = .505/dy if dy > 1e-6 else math.inf
        if not -.3 <= best*dx <= 3.0:
            best = math.inf
        if occluder is not None:
            (x0, x1), (y0, y1) = occluder
            lo, hi = -math.inf, math.inf
            for p0, p1, d in ((x0, x1, dx), (y0, y1, dy)):
                if abs(d) < 1e-12:
                    continue
                t0, t1 = sorted((p0/d, p1/d))
                lo, hi = max(lo, t0), min(hi, t1)
            if lo <= hi and lo > 0:
                best = min(best, lo)
        if math.isfinite(best):
            points.append((best*dx, best*dy))
    return np.asarray(points)


def test_narrow_occluder_keeps_wall_length_but_wide_object_still_breaks():
    from arena_autonomy.local_path import local_path
    post = ((1.0-.0073, 1.0+.0073), (.3725-.0075, .3725+.0075))
    car = ((1.0, 1.2), (.30, .45))
    _, clear = local_path(wall_scan(), 'left', .505, horizon=2.5)
    assert clear['boundary_observed_ahead_m'] == pytest.approx(2.5)
    plain_path, plain = local_path(wall_scan(post), 'left', .505, horizon=2.5)
    path, bridged = local_path(wall_scan(post), 'left', .505, horizon=2.5, narrow_occluder_bridge=True)
    # 지주 그림자가 벽 x≈1.36 m에 떨어진다. 끊김 없는 벽은 탐색 한도 2.5 m까지 이어진다.
    assert plain['boundary_observed_ahead_m'] < 1.4 and bridged['boundary_observed_ahead_m'] == pytest.approx(2.5)
    assert bridged['narrow_occluders_bridged'] == 1 and 'narrow_occluders_bridged' not in plain
    _, wide = local_path(wall_scan(car), 'left', .505, horizon=2.5, narrow_occluder_bridge=True)
    assert 'narrow_occluders_bridged' not in wide and wide['boundary_observed_ahead_m'] < 1.6
    # 기본값은 이전 호출과 같은 결과다.
    again_path, again = local_path(wall_scan(post), 'left', .505, horizon=2.5, narrow_occluder_bridge=False)
    assert again == plain


def test_candidates_are_default_off_and_bool_only():
    from arena_autonomy.local_follow import FollowConfig
    assert FollowConfig().narrow_occluder_bridge is False and FollowConfig().junction_continuity is False
    for key in ('narrow_occluder_bridge', 'junction_continuity'):
        with pytest.raises(ValueError):
            FollowConfig(**{key: 1})
        assert getattr(FollowConfig(**{key: True}), key) is True
