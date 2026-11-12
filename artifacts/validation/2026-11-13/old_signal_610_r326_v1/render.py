"""r307의 같은 RGB/검출기를 사용하고 구형610 지지 구조만 선택한다."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
BASE = A.parent/'camera_grid_r307_v1/audit_v3.py'
spec = importlib.util.spec_from_file_location('old610_grid_audit', BASE)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def main():
    root = ET.Element('sdf', version='1.10')
    model = ET.parse(A/'supports_trial.sdf').getroot().find("model[@name='trial_old610_signal_support']")
    assert model is not None
    model = copy.deepcopy(model)
    model.set('name', 'eval_20261112_signal_support')
    root.append(model)
    camera_support = A/'camera_support.sdf'
    ET.ElementTree(root).write(camera_support, encoding='utf-8', xml_declaration=True)
    audit.HERE = A
    audit.SUPPORT = camera_support
    original = audit.build_fixture

    def build(path, shift):
        assert shift == 0.
        views = original(path, shift)
        tree = ET.parse(path)
        track = tree.getroot().find("world/model[@name='it_arena_track_static']")
        beam = track.find("link[@name='tl_beam']")
        assert beam is not None
        track.remove(beam)
        ET.indent(tree)
        tree.write(path, encoding='utf-8', xml_declaration=True)
        return views

    audit.build_fixture = build
    args = SimpleNamespace(output='render_current', head_shift=0.,
        levels='empty,same224,partial230', slots='', timeout=900.)
    result = audit.render(args)
    report = A/'render_current/report.json'
    if report.is_file():
        data = json.loads(report.read_text())
        data['facility_selection'] = 'provisional user old610; exact STL supports; dynamic optical panel unchanged'
        report.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    raise SystemExit(main())
