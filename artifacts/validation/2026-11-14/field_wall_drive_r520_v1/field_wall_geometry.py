"""r520 두 벽의 국소 파생 형상. 공통 월드/센서/제어 입력은 수정하지 않는다."""
from __future__ import annotations
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
SOURCE = ROOT / 'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
DERIVED = A / 'field_world.sdf'
SOURCE_SHA = '4c2136c7eec1615293db9f0cd8a5acb5133fc5f433c4a16f292b527bca851b3b'
SELECTED = {'walls_main_left_5': 1, 'walls_main_left_25': -1}
Y0, Y1, AMPLITUDE, PERIOD = 3.0, 11.5, .030, 1.0
THICKNESS, CHORD, JOIN_PERIOD, GAP = .005, .050, .600, .020
INTERNAL_OVERLAP, END_CAP = .001, .001
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
read = lambda p: json.loads(Path(p).read_bytes())


def canonical(element):
    return ET.canonicalize(ET.tostring(element, encoding='unicode'), strip_text=True)


def walls_element(path):
    return ET.parse(path).getroot().find("world/model[@name='it_arena_track_static']/link[@name='walls']")


def collision_digest(path):
    return hashlib.sha256('\n'.join(canonical(c) for c in walls_element(path).findall('collision')).encode()).hexdigest()


def verify_derived():
    value = read(A / 'field_geometry_audit.json')
    assert sha(SOURCE) == value['source_sha256'] == SOURCE_SHA
    assert sha(DERIVED) == value['derived_sha256']
    assert collision_digest(DERIVED) == value['walls_collision_sha256']
    assert value['unchanged_original_walls'] == 127 and value['non_wall_subtrees_unchanged']
    return value


def paired_geometry(node):
    return (node.findtext('pose'), canonical(node.find('geometry')))


def values(node):
    return tuple(map(float, node.findtext('pose').split())), tuple(map(float, node.findtext('geometry/box/size').split()))


def nominal_face_x(collision, outward, y):
    pose, size = values(collision)
    nx = abs(math.sin(pose[5]))
    slope = math.cos(pose[5]) / math.sin(pose[5])
    return pose[0] + slope * (y - pose[1]) - outward * size[1] / (2 * nx)


def wave(y):
    return AMPLITUDE * math.sin(2 * math.pi * (y - Y0) / PERIOD)


def segment(collision, outward, lo, hi, extend_lo=0., extend_hi=0., clip_lo=None, clip_hi=None, flat=False):
    pose, size = values(collision)
    p = (nominal_face_x(collision, outward, lo) + (0 if flat else wave(lo)), lo)
    q = (nominal_face_x(collision, outward, hi) + (0 if flat else wave(hi)), hi)
    length = math.hypot(q[0]-p[0], q[1]-p[1])
    tx, ty = (q[0]-p[0])/length, (q[1]-p[1])/length
    nx, ny = outward*ty, -outward*tx
    cx = (p[0]+q[0])/2 + THICKNESS*nx/2 + tx*(extend_hi-extend_lo)/2
    cy = (p[1]+q[1])/2 + THICKNESS*ny/2 + ty*(extend_hi-extend_lo)/2
    length += extend_lo + extend_hi
    # 접합 틈은 world-y 폭20mm로 유지한다. 기울어진 상자의 바깥 꼭짓점도 침범하지 않는다.
    lower = cy - ty*length/2 - abs(ny)*THICKNESS/2
    if clip_lo is not None and lower < clip_lo:
        cut = (clip_lo-lower)/ty
        length -= cut; cx += tx*cut/2; cy += ty*cut/2
    upper = cy + ty*length/2 + abs(ny)*THICKNESS/2
    if clip_hi is not None and upper > clip_hi:
        cut = (upper-clip_hi)/ty
        length -= cut; cx -= tx*cut/2; cy -= ty*cut/2
    assert length > 0
    return (cx, cy, pose[2], 0., 0., math.atan2(ty, tx)), (length, THICKNESS, size[2])


def retained(collision, lo, hi):
    pose, size = values(collision)
    slope = math.cos(pose[5])/math.sin(pose[5])
    cy = (lo+hi)/2
    return (pose[0]+slope*(cy-pose[1]), cy, *pose[2:]), ((hi-lo)/abs(math.sin(pose[5])), size[1], size[2])


def add_pair(link, old_collision, old_visual, stem, pose, size):
    for tag, original, suffix in (('collision', old_collision, 'col'), ('visual', old_visual, 'vis')):
        item = copy.deepcopy(original)
        item.set('name', stem+'_'+suffix)
        item.find('pose').text = ' '.join(f'{v:.12f}' for v in pose)
        item.find('geometry/box/size').text = ' '.join(f'{v:.12f}' for v in size)
        link.append(item)


def generate():
    assert sha(SOURCE) == SOURCE_SHA
    assert not DERIVED.exists() and not (A/'field_geometry_audit.json').exists()
    before = ET.parse(SOURCE).getroot()
    after = copy.deepcopy(before)
    old_walls = before.find("world/model[@name='it_arena_track_static']/link[@name='walls']")
    walls = after.find("world/model[@name='it_arena_track_static']/link[@name='walls']")
    assert len(old_walls.findall('collision')) == len(old_walls.findall('visual')) == 129
    seam_centers = [Y0+j*JOIN_PERIOD for j in range(1, math.floor((Y1-Y0)/JOIN_PERIOD)+1)]
    gaps = [(y-GAP/2, y+GAP/2) for y in seam_centers]
    bounds = [Y0, *[v for pair in gaps for v in pair], Y1]
    panels = list(zip(bounds[0::2], bounds[1::2]))
    details = []
    for name, outward in SELECTED.items():
        original = old_walls.find(f"collision[@name='{name}_col']")
        visual = old_walls.find(f"visual[@name='{name}_vis']")
        assert paired_geometry(original) == paired_geometry(visual)
        pose, size = values(original)
        assert size[1:] == (.05, .4) and abs(abs(pose[5])-math.pi/2) < 1e-5
        for suffix, tag in (('col','collision'),('vis','visual')):
            walls.remove(walls.find(f"{tag}[@name='{name}_{suffix}']"))
        low = pose[1] - size[0]*abs(math.sin(pose[5]))/2
        high = pose[1] + size[0]*abs(math.sin(pose[5]))/2
        generated = []
        for side, lo, hi in (('before', low, Y0), ('after', Y1, high)):
            stem = f'field_{name}_{side}'
            add_pair(walls, original, visual, stem, *retained(original,lo,hi))
            generated.append(dict(name=stem,kind='retained_original_thickness',panel=None))
        for panel_id, (lo,hi) in enumerate(panels):
            grid = [lo, *[Y0+j*CHORD for j in range(1,math.ceil((Y1-Y0)/CHORD)) if lo+1e-9 < Y0+j*CHORD < hi-1e-9], hi]
            for index, (left,right) in enumerate(zip(grid,grid[1:])):
                stem=f'field_{name}_p{panel_id:02d}_s{index:03d}'
                pp,ss=segment(original,outward,left,right,
                    INTERNAL_OVERLAP if index else 0., INTERNAL_OVERLAP if index<len(grid)-2 else 0.,
                    (lo+END_CAP/2 if panel_id==0 else lo) if not index else None,
                    (hi-END_CAP/2 if panel_id==len(panels)-1 else hi) if index==len(grid)-2 else None)
                add_pair(walls,original,visual,stem,pp,ss)
                generated.append(dict(name=stem,kind='curve_chord',panel=panel_id,face_sample_y=[left,right]))
        # 두 국소 경계에서만1mm 평탄 연결부. 정해진 접합 틈에는 추가하지 않는다.
        for side, lo, hi, panel_id in (('lower_cap',Y0-1e-6,Y0+END_CAP,0),('upper_cap',Y1-END_CAP,Y1+1e-6,len(panels)-1)):
            stem=f'field_{name}_{side}'
            add_pair(walls,original,visual,stem,*segment(original,outward,lo,hi,flat=True))
            generated.append(dict(name=stem,kind='boundary_flat_connector',panel=panel_id))
        details.append(dict(source_name=name,source_pose=pose,source_size=size,outward_x_sign=outward,
            nominal_face_x_at_y0=nominal_face_x(original,outward,Y0),
            generated=generated,generated_collision_count=len(generated)))
    untouched=[]
    for c in old_walls.findall('collision'):
        if c.get('name')[:-4] in SELECTED:continue
        name=c.get('name');new=walls.find(f"collision[@name='{name}']")
        assert canonical(c)==canonical(new),name
        vname=name[:-4]+'_vis'
        assert canonical(old_walls.find(f"visual[@name='{vname}']"))==canonical(walls.find(f"visual[@name='{vname}']"))
        untouched.append(name)
    before_without=copy.deepcopy(before);after_without=copy.deepcopy(after)
    for root in (before_without,after_without):
        model=root.find("world/model[@name='it_arena_track_static']")
        model.remove(model.find("link[@name='walls']"))
    assert canonical(before_without)==canonical(after_without)
    assert len(untouched)==127
    for c in walls.findall('collision'):
        assert paired_geometry(c)==paired_geometry(walls.find(f"visual[@name='{c.get('name')[:-4]}_vis']"))
    data=ET.tostring(after,encoding='utf-8',xml_declaration=True)
    with DERIVED.open('xb') as f:f.write(data)
    assert sha(SOURCE)==SOURCE_SHA
    result=dict(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=SOURCE_SHA,
        derived_world=DERIVED.relative_to(ROOT).as_posix(),derived_sha256=sha(DERIVED),
        walls_collision_sha256=collision_digest(DERIVED),
        scope='Unmeasured field-wall sensitivity assumption; not measured as-built geometry',
        parameters=dict(y_range_m=[Y0,Y1],amplitude_m=AMPLITUDE,period_m=PERIOD,same_global_x_phase_both_walls=True,
            wall_thickness_m=THICKNESS,chord_grid_step_m=CHORD,joint_pitch_m=JOIN_PERIOD,joint_gap_world_y_m=GAP,
            internal_chord_overlap_each_end_m=INTERNAL_OVERLAP,boundary_flat_connector_length_m=END_CAP,
            boundary_connector_overlap_into_original_m=1e-6,boundary_chord_inset_m=END_CAP/2,
            endpoint_wave_offsets_m=[wave(Y0),wave(Y1)],nominal_road_side_plane_preserved=True),
        original_walls=129,unchanged_original_walls=len(untouched),unchanged_wall_names=untouched,
        derived_wall_collision_count=len(walls.findall('collision')),selected_walls=details,
        joint_gap_intervals_m=gaps,continuous_panel_intervals_m=panels,
        non_wall_subtrees_unchanged=True,collision_visual_geometry_identical=True,
        friction_material_height_inherited=True,source_unchanged=True,
        curve_linear_interpolation_error_bound_m=AMPLITUDE*(2*math.pi/PERIOD)**2*CHORD**2/8,
        static_geometry_check_pending=True,execution_authorized=False,simulator_started=False)
    with (A/'field_geometry_audit.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps({k:result[k] for k in ('derived_sha256','original_walls','unchanged_original_walls','derived_wall_collision_count','curve_linear_interpolation_error_bound_m')}))


if __name__=='__main__':generate()
