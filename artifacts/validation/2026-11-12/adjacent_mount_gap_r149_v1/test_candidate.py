import copy
import json
from pathlib import Path

import numpy as np
import pytest
import candidate
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.known_structure import parse_sectors
from arena_autonomy.scene_motion import motion_scene

A=Path(__file__).resolve().parent


def run(*,modify=None,speed=0.,target=.15,sectors=None,padding_factor=1.):
    f=json.loads((A/'fixture.json').read_bytes())
    owner,motion=make_controller(f['header'])
    if modify:modify(f['input']['scan']['measurements'])
    data,scan,history=restore_call(f['input'],motion)
    scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                       epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
    actual_sectors=parse_sectors(owner.config.known_structure_sectors_json) if sectors is None else sectors
    result=candidate.bounded_adjacent_structure(scene,scan,f['geometry']['queries'],
        np.asarray(f['geometry']['padding'])*padding_factor,owner.config.length_m,owner.config.width_m,
        speed,target,actual_sectors)
    return result


def test_saved_group_is_bounded_without_relabeling_unknown():
    out=run()
    assert out['accepted'] and not out['unknown_filled'] and out['residual_hidden_object_risk']
    assert out['maximum_chord_m']<=.05
    assert len(out['gaps'])==1 and out['gaps'][0]['unknown_gap_rad']<=np.deg2rad(6)
    assert out['gaps'][0]['masked_indices']
    assert min(out['gaps'][0]['outer_return_ranges_m'])>=.6


@pytest.mark.parametrize('speed,target',[(.151,.15),(.2,.15),(0.,.151),(0.,.35),(-.01,.15)])
def test_only_actual_and_target_crawl_speed(speed,target):
    assert not run(speed=speed,target=target)['accepted']


@pytest.mark.parametrize('value',[{'nonfinite':'nan'},{'nonfinite':'-inf'},.01,.15])
def test_invalid_or_near_return_in_gap_still_rejects(value):
    assert not run(modify=lambda m:m['ranges_m'].__setitem__(89,copy.deepcopy(value)))['accepted']


def test_finite_value_in_declared_mask_is_not_excused():
    assert not run(modify=lambda m:m['ranges_m'].__setitem__(87,.08))['accepted']


@pytest.mark.parametrize('sectors',[(),((180.,30.),),((122.8,1.),)])
def test_missing_partial_or_rear_mask_is_not_a_mount_sector(sectors):
    assert not run(sectors=sectors)['accepted']


def test_close_outer_flank_is_not_hidden():
    assert not run(modify=lambda m:m['ranges_m'].__setitem__(91,.2))['accepted']


def test_wider_missing_part_rejects():
    def modify(m):
        for i in range(91,100):m['ranges_m'][i]={'nonfinite':'inf'}
    assert not run(modify=modify)['accepted']


def test_combined_width_counts_padding_too():
    out=run(padding_factor=3.)
    assert not out['accepted']
