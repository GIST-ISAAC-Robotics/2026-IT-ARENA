"""ROS 없이 기록 파일 무결성 및 재생 발행 경계를 검사한다."""
import hashlib
import json
import math
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import (
    INPUTS, OBSERVATIONS, TOPICS, REQUIRED, load_manifest, replay_topic, sha256_file,
    update_digest, quantiles, eof_stopped, compare_commands, dds_metadata,
)


@pytest.fixture
def recording(tmp_path):
    (tmp_path/'bag').mkdir()
    for path in ('receipt.jsonl', 'parameters.json', 'bag/metadata.yaml', 'bag/data.mcap'):
        (tmp_path/path).write_bytes(b'test')
    manifest = dict(schema=1, complete=True, topics=TOPICS,
        stats={t: dict(count=1) for t in REQUIRED},
        files_sha256={str(p.relative_to(tmp_path)): sha256_file(p)
                      for p in tmp_path.rglob('*') if p.is_file()})
    def save():
        (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    save()
    return tmp_path, manifest, save


def test_manifest_inventory(recording):
    root, manifest, _ = recording
    assert load_manifest(root) == manifest


def test_dds_metadata_accepts_mapping_and_attributes():
    from types import SimpleNamespace
    values = dict(source_timestamp=12, received_timestamp=15, publication_sequence_number=3,
                  reception_sequence_number=2, publisher_gid=bytes([1, 2]))
    assert dds_metadata(values) == dds_metadata(SimpleNamespace(**values))
    assert dds_metadata(values)['publisher_gid'] == '0102'
    assert dds_metadata({})['publication_sequence_number'] is None


@pytest.mark.parametrize('field,value', [('complete', False), ('schema', 2), ('queue_drops', {'/scan': 1}),
    ('error', ['disk failure']), ('clock_regressions', 1)])
def test_reject_partial_and_loss(recording, field, value):
    root, manifest, save = recording
    manifest[field] = value
    save()
    with pytest.raises(ValueError):
        load_manifest(root)


def test_corrupt_bytes_rejected(recording):
    root, _, _ = recording
    (root/'bag/data.mcap').write_bytes(b'changed')
    with pytest.raises(ValueError, match='integrity'):
        load_manifest(root)


def test_path_escape_rejected(recording, tmp_path):
    root, manifest, save = recording
    manifest['files_sha256']['../outside.mcap'] = '00'
    save()
    with pytest.raises(ValueError, match='integrity'):
        load_manifest(root)


@pytest.mark.parametrize('path', ['receipt.jsonl', 'parameters.json', 'bag/metadata.yaml', 'bag/data.mcap'])
def test_missing_inventory_rejected(recording, path):
    root, manifest, save = recording
    del manifest['files_sha256'][path]
    save()
    with pytest.raises(ValueError):
        load_manifest(root)


def test_missing_sensor_rejected(recording):
    root, manifest, save = recording
    manifest['stats']['/scan']['count'] = 0
    save()
    with pytest.raises(ValueError, match='Required topics'):
        load_manifest(root)


@pytest.mark.parametrize('topic', [*OBSERVATIONS, '/cmd_vel', '/odom', '/clock', '/ground_truth'])
def test_commands_truth_and_clock_never_sensor_replayed(topic):
    with pytest.raises(ValueError):
        replay_topic(topic)


@pytest.mark.parametrize('topic', INPUTS)
def test_only_sensors_are_namespaced(topic):
    assert replay_topic(topic) == '/replay'+topic


def test_topic_contract_not_extensible_through_manifest(recording):
    root, manifest, save = recording
    manifest['topics'] = dict(TOPICS, **{'/odom': 'nav_msgs/msg/Odometry'})
    save()
    with pytest.raises(ValueError, match='contract'):
        load_manifest(root)


def test_digest_includes_stamp_length_and_order():
    def digest(rows):
        h = hashlib.sha256()
        for data, stamp in rows:
            update_digest(h, data, stamp)
        return h.hexdigest()
    assert len({digest(rows) for rows in [
        [(b'a', 1), (b'bc', 2)], [(b'ab', 1), (b'c', 2)],
        [(b'a', 2), (b'bc', 2)], [(b'bc', 2), (b'a', 1)]]}) == 4


def test_quantiles_empty_singleton_and_order():
    assert quantiles([])['p95'] is None
    assert quantiles([4]) == dict(count=1, p50=4, p95=4, p99=4, max=4)
    assert quantiles(list(reversed(range(101))))['p95'] == 95


def test_eof_stop_requires_new_tail_commands():
    assert not eof_stopped([[1_000_000_000, 0., 0.]]*20, 1_000_000_000)
    tail = [[2_250_000_000+i*20_000_000, 0., 0.] for i in range(10)]
    assert eof_stopped(tail, 1_000_000_000)
    tail[-1][1] = .1
    assert not eof_stopped(tail, 1_000_000_000)
    tail[-1][1] = math.nan
    assert not eof_stopped(tail, 1_000_000_000)


def test_command_comparison_excludes_eof_and_prestart():
    result = compare_commands([[10, 1., .2], [20, 2., .4]],
        [[5, 100., 1.], [15, 1.5, .3], [25, 0., 0.]], 20)
    assert result['samples'] == 1
    assert result['speed_absolute_error_mps']['max'] == .5
