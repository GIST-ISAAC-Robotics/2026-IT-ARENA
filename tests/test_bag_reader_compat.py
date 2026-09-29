"""새 metadata를 읽지 못하는 Humble의 단일 MCAP 호환 경로만 허용한다."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('a3_audit', ROOT/'scripts/audit_sensor_bag.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def reader_double(monkeypatch, failure='yaml-cpp: bad conversion', topics=None):
    calls = []
    class Reader:
        def open(self, options, converter):
            calls.append(options.uri)
            if len(calls) == 1 and failure:
                raise RuntimeError(failure)
        def get_all_topics_and_types(self):
            values = audit.TOPICS if topics is None else topics
            return [SimpleNamespace(name=k, type=v) for k, v in values.items()]
    monkeypatch.setitem(sys.modules, 'rosbag2_py', SimpleNamespace(SequentialReader=Reader,
        StorageOptions=lambda **kwargs: SimpleNamespace(**kwargs), ConverterOptions=lambda *_: None))
    return calls


def recording(root, version=9, names=('bag_0.mcap',)):
    bag = root/'bag'
    bag.mkdir()
    for name in names:
        (bag/name).write_bytes(b'placeholder')
    (bag/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':
        {'version': version, 'relative_file_paths': list(names)}}))


def test_normal_directory_path_is_unchanged(tmp_path, monkeypatch):
    calls = reader_double(monkeypatch, failure=None)
    mode = {}
    audit.reader_for(tmp_path, mode)
    assert calls == [str(tmp_path/'bag')]
    assert mode['mode'] == 'directory_metadata'


def test_v9_single_mcap_fallback_preserves_original(tmp_path, monkeypatch):
    recording(tmp_path)
    path = tmp_path/'bag/metadata.yaml'
    before = path.read_bytes()
    calls = reader_double(monkeypatch)
    mode = {}
    audit.reader_for(tmp_path, mode)
    assert calls[-1] == str(tmp_path/'bag/bag_0.mcap')
    assert mode['mode'] == 'single_mcap_metadata_v9_compat'
    assert path.read_bytes() == before


@pytest.mark.parametrize('version,names,failure', [
    (5, ('bag_0.mcap',), 'yaml-cpp: bad conversion'),
    (9, ('bag_0.mcap', 'bag_1.mcap'), 'yaml-cpp: bad conversion'),
    (9, ('bag_0.mcap',), 'disk read failure')])
def test_other_errors_are_not_hidden(tmp_path, monkeypatch, version, names, failure):
    recording(tmp_path, version, names)
    calls = reader_double(monkeypatch, failure)
    with pytest.raises(RuntimeError, match=failure):
        audit.reader_for(tmp_path)
    assert len(calls) == 1


def test_fallback_still_checks_topic_contract(tmp_path, monkeypatch):
    recording(tmp_path)
    reader_double(monkeypatch, topics={'/drive': 'unexpected'})
    with pytest.raises(ValueError, match='topic/type'):
        audit.reader_for(tmp_path)
