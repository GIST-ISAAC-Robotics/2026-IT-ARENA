from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_braking_ab import normalized_world, sampled_difference


def test_comparison_retains_wall_shape_and_visual_content(tmp_path):
    a, b = tmp_path/'a', tmp_path/'b'
    a.mkdir()
    b.mkdir()
    def make(folder, surface):
        mesh = folder/'batched_walls_0.obj'
        mesh.write_text('same visual vertices')
        (folder/'world.sdf').write_text('<sdf><world name="test"><model name="track"><link name="walls">'
            '<collision name="wall"><geometry><box><size>1 1 1</size></box></geometry></collision>'
            f'<visual><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual>{surface}'
            '</link></model></world></sdf>')
    make(a, '<collision name="surface_a_col"/>')
    make(b, '')
    assert normalized_world(a/'world.sdf')[0] == normalized_world(b/'world.sdf')[0]
    assert normalized_world(a/'world.sdf')[1] == 1
    (b/'batched_walls_0.obj').write_text('changed visual vertices')
    assert normalized_world(a/'world.sdf')[0] != normalized_world(b/'world.sdf')[0]


def test_command_alignment_exposes_delay_and_refuses_stale():
    a = [{'t': i*.02, 'v': 1. if i < 10 else 0.} for i in range(25)]
    b = [{'t': i*.02, 'v': 1. if i < 12 else 0.} for i in range(25)]
    assert sampled_difference(a, a, 'v')['max_abs'] == 0.
    assert sampled_difference(a, b, 'v')['max_abs'] == 1.
    assert sampled_difference([], b, 'v')['max_abs'] is None
    stale = sampled_difference([{'t': 0., 'v': 1.}, {'t': 1., 'v': 0.}], a, 'v')
    assert stale['samples'] <= 3
