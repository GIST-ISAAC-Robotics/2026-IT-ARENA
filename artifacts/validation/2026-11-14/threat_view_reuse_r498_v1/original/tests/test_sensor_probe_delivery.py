import hashlib

import pytest

from sensor_probe_delivery import DeliveryInventory


def entry(data, topic='/camera/color/camera_info'):
    return dict(topic=topic, bytes=len(data), sha256=hashlib.sha256(data).hexdigest())


@pytest.mark.parametrize('size', [4, 5, 6, 7, 8, 333, 336])
def test_exact_and_explicit_end_padding_are_distinct(size):
    raw = b'a'*size
    exact = DeliveryInventory([entry(raw)])
    exact.observe('/camera/color/camera_info', raw)
    assert exact.result()['full_wire_bytes_equal']
    assert exact.result()['all_sent_observed_exactly_once']
    pad = (-size) % 4
    aligned = DeliveryInventory([entry(raw)])
    aligned.observe('/camera/color/camera_info', raw+b'\0'*pad)
    assert aligned.result()['all_sent_observed_exactly_once']
    assert aligned.result()['full_wire_bytes_equal'] is (pad == 0)


@pytest.mark.parametrize('bad', [b'abcde\0\0x', b'abxde\0\0\0', b'abcde\0', b'abcde\0\0\0\0\0\0\0', b'abcd'])
def test_not_arbitrary_padding_removal_or_tolerance(bad):
    inventory = DeliveryInventory([entry(b'abcde')])
    inventory.observe('/camera/color/camera_info', bad)
    assert not inventory.result()['every_received_payload_matched']
    assert inventory.result()['missing']


def test_ambiguous_prefix_is_not_chosen():
    inventory = DeliveryInventory([entry(b'abcde'), entry(b'abcde\0')])
    inventory.observe('/camera/color/camera_info', b'abcde\0\0\0')
    assert not inventory.result()['every_received_payload_matched']


def test_missing_and_duplicate_and_wrong_topic_remain_failures():
    inventory = DeliveryInventory([entry(b'aaaa'), entry(b'bbbb')])
    inventory.observe('/camera/color/camera_info', b'aaaa')
    inventory.observe('/camera/color/camera_info', b'aaaa')
    inventory.observe('/scan', b'bbbb')
    r = inventory.result()
    assert r['missing'] and r['excess'] and r['unexpected']
    assert not r['all_sent_observed_exactly_once']


def test_identical_messages_count_individually():
    inventory = DeliveryInventory([entry(b'aaaa')]*2)
    inventory.observe('/camera/color/camera_info', b'aaaa')
    assert not inventory.result()['all_sent_observed_exactly_once']
    inventory.observe('/camera/color/camera_info', b'aaaa')
    assert inventory.result()['all_sent_observed_exactly_once']


@pytest.mark.parametrize('bad', [[], [dict(topic='/scan', bytes=True, sha256='0'*64)],
                                [dict(topic='/scan', bytes=4, sha256='z'*64)]])
def test_invalid_sender_inventory_rejected(bad):
    with pytest.raises(ValueError):
        DeliveryInventory(bad)
