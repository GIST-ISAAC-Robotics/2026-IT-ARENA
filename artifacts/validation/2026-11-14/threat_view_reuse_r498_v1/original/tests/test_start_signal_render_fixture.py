import math
from types import SimpleNamespace
import pytest
import validate_start_signal_render as render
from validate_start_signal_render import build_fixture, replay_pixels
from audit_start_signal_layout import light_image


def test_render_fixture_isolated_views_and_correct_colors():
    scene = {'starting_grid': {'slots': [dict(index=i, x=0., y=-1., yaw_rad=math.pi/2) for i in (2, 3)]}}
    lamps = [dict(name=f'lamp_red_{i+1}' if i<4 else 'lamp_green', xyz=[0., i*.03, 1.08], radius=.0075) for i in range(5)]
    cam = dict(xyz_m=[.055, 0., .145], rpy_rad=[0., 0., 0.], width_px=1280, height_px=720, horizontal_fov_deg=75.)
    root, views = build_fixture(scene, lamps, cam)
    assert len(views) == 6
    assert len(root.findall('.//sensor')) == 6
    assert not root.findall('.//collision')
    for name, view in views.items():
        materials = root.findall(f"world/model[@name='lamps_{name}']/link/visual/material")
        emit = [m.findtext('emissive') for m in materials]
        assert emit.count('1 0 0 1') == (4 if view['stage'] == 'red4' else 0)
        assert emit.count('0 1 0 1') == (1 if view['stage'] == 'green' else 0)


def test_static_pixel_replay_labels_synthetic_time_separately():
    frames = {}
    for index in (2, 3):
        for stage in ('off', 'red4', 'green'):
            rgb = light_image(4 if stage == 'red4' else 0, stage == 'green', diameter=12, paired=False)
            frames[f'grid{index}_{stage}'] = [dict(stamp=10.+i*.1, rgb=rgb) for i in range(3)]
    result = replay_pixels(frames)
    assert all(r['green_detected'] and not r['early_start'] and r['off_keeps_start'] for r in result.values())
    assert result['2']['rows'][0]['assigned_replay_stamp'] == 0
    assert result['2']['rows'][0]['captured_stamp'] == 10.


def test_native_bridge_must_belong_to_this_launch():
    log = '[parameter_bridge-2]: process started with pid [123]'
    assert render.owned_bridge(100, log, {123: dict(ppid=100, start='42')})[0] == 123
    with pytest.raises(ValueError, match='owned child'):
        render.owned_bridge(100, log, {123: dict(ppid=101, start='42')})
    with pytest.raises(ValueError, match='unique'):
        render.owned_bridge(100, log+'\n'+log, {})


def test_bridge_pid_reuse_is_not_signaled(monkeypatch):
    rows = [{123: dict(ppid=100, start='42')}, {123: dict(ppid=100, start='43')}]
    monkeypatch.setattr(render, 'group_processes', lambda pid: rows.pop(0))
    actions = []
    monkeypatch.setattr(render.os, 'kill', lambda *a: actions.append(a))
    result = render.stop_bridge_first(SimpleNamespace(pid=100, poll=lambda: None),
                                     '[parameter_bridge-2]: process started with pid [123]')
    assert not result['verified'] and not actions


def test_residue_does_not_kill_unknown_or_reused_pid(monkeypatch):
    rows = {123: dict(ppid=1, start='43'), 124: dict(ppid=1, start='42')}
    monkeypatch.setattr(render, 'group_processes', lambda pid: rows)
    actions = []
    monkeypatch.setattr(render.os, 'kill', lambda *a: actions.append(a))
    result = render.cleanup_owned_residue(100, {123: dict(ppid=100, start='42')})
    assert not actions and len(result['remaining']) == 2
