"""평가 사본의 시작점/원 XML만 변형한다. ROS 노드/제어 파라미터는 건드리지 않는다."""
import ast
import hashlib
import inspect
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from prepare import WORLD, CASES, spawn_variant, world_variant

URI_TAGS = ('uri', 'albedo_map', 'normal_map', 'roughness_map', 'metalness_map',
            'environment_map', 'emissive_map')
SPAWN_LINE = 'spawn_x, spawn_y, spawn_yaw = slot["x"], slot["y"], slot["yaw_rad"]'


def write_new(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def raw_variant(source, name, *, absolute_uris):
    """같은 원본 디렉터리 기준 URI 해석을 유지한 뒤 임시 source로 전달한다."""
    tree, edits = world_variant(ET.parse(source).getroot(), name)
    changed = []
    if absolute_uris:
        for tag in URI_TAGS:
            for value in tree.find('world').findall(f'.//{tag}'):
                if value.text and '://' not in value.text and not Path(value.text).is_absolute():
                    before = value.text
                    value.text = str((source.parent / before).resolve())
                    changed.append(dict(tag=tag, before=before, after=value.text))
    return tree, dict(world_edits=edits, uri_edits=changed)


def runtime_wrapper(original, name, evidence):
    assert name in CASES
    def isolated(*args, **kwargs):
        bound = inspect.signature(original).bind(*args, **kwargs)
        bound.apply_defaults()
        source, destination = Path(bound.arguments['source']), Path(bound.arguments['destination'])
        assert source.read_bytes() == WORLD.read_bytes(), 'only the registered original world'
        tree, edits = raw_variant(source, name, absolute_uris=True)
        # common도 이 URI 변환을 한다. 임시 source.parent로 기준이 바뀌지 않게 먼저 처리한다.
        with tempfile.TemporaryDirectory(prefix='r492_raw_', dir=destination.parent) as temporary:
            derived = Path(temporary)/'world.sdf'
            ET.ElementTree(tree).write(derived, encoding='utf8', xml_declaration=True)
            bound.arguments['source'] = derived
            report = original(*bound.args, **bound.kwargs)
        output = ET.parse(destination).getroot()
        model = output.find("world/model[@name='it_arena_track_static']")
        beam = model.find("link[@name='tl_beam']")
        assert beam is not None, 'same duplicate old610 beam removal as r475'
        removed = ET.tostring(beam, encoding='unicode')
        model.remove(beam)
        factors = output.findall('world/physics/real_time_factor')
        assert len(factors) == 1 and float(factors[0].text) == 1.0
        factors[0].text = '0.25'
        ET.ElementTree(output).write(destination, encoding='utf8', xml_declaration=True)
        write_new(evidence/'world_variation.json', dict(case=name, source=str(source),
            source_sha256=sha(source), runtime_sha256=sha(destination), **edits,
            original_override_arguments={k: str(v) if isinstance(v, Path) else v
                for k, v in bound.arguments.items() if k not in ('source', 'destination')},
            removed_duplicate_beam=removed, real_time_factor=.25,
            max_step_size=output.findtext('world/physics/max_step_size'),
            scope='evaluation world only; raw variant before common batch; no world_override'))
        return report
    return isolated


def spawn_clone(original, name, evidence):
    """선택한 slot의 지역 변수 세 개만 수정한다. 원 launch 함수/기본값은 보존한다."""
    assert name in CASES
    source = inspect.getsource(original)
    assert source.count(SPAWN_LINE) == 1, 'review changed spawn entry before reuse'
    insertion = SPAWN_LINE + '\n    spawn_x, spawn_y, spawn_yaw = _r492_spawn(spawn_x, spawn_y, spawn_yaw)'
    updated = source.replace(SPAWN_LINE, insertion)
    assert updated.replace(insertion, SPAWN_LINE) == source
    def select(x, y, yaw):
        pose = spawn_variant(x, y, yaw, name)
        write_new(evidence/'spawn_variation.json', dict(case=name, before=[x,y,yaw], after=list(pose),
            selected_after_slot=True, controller_parameters_changed=False))
        return pose
    # globals must remain the clone module dictionary: runtime overrides are set in that module.
    original.__globals__['_r492_spawn'] = select
    local = {}
    exec(compile(ast.parse(updated), inspect.getsourcefile(original), 'exec'), original.__globals__, local)
    return local[original.__name__]
