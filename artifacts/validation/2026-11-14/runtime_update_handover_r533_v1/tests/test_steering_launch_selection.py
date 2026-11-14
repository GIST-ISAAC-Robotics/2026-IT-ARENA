import json
from pathlib import Path
import tempfile

import pytest
import steering_launch_selection as subject
from test_runtime_installed_limits import synthetic_bundle,BASE

SELECTION=Path(__file__).resolve().parents[1]/'steering_selection.assembled_2026_11_14.json'


def test_source_selection_produces_arguments_without_claiming_runtime_installation():
    result=subject.selected_arguments(SELECTION)
    assert result['argument_fragments']['preview']==['--vehicle-config','src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml']
    assert result['argument_fragments']['host']==['--max-steering-rad','0.28475436568260193']
    assert result['argument_fragments']['race_ops'][-1]=='manual_steering_limit_rad:=0.2'
    assert result['argument_fragments']['planner'] is None and result['argument_fragments']['guard'] is None
    assert not result['installed_profile_link_verified'] and not result['complete_launch_command']
    assert not result['nodes_started'] and not result['device_io']


def test_existing_selected_bundle_links_both_nodes_and_rejects_widening_base():
    with tempfile.TemporaryDirectory(prefix='r533-launch-synthetic-') as temp:
        root=Path(temp);bundle=synthetic_bundle(root)
        board=subject.installed.read_json(bundle/'runtime_board.json')
        selection=subject.installed.read_json(SELECTION)
        board['assembly_id']=selection['assembly_id'];board['servo']['command_limit']=selection['command_limit_rad']
        bp=root/'synthetic-board.json';bp.write_text(json.dumps(board),encoding='utf-8')
        selected=root/'installed_selected'
        subject.installed.prepare(bp,subject.ROOT/selection['steering_profile']['path'],selection['assembly_id'],selected)
        base=root/'base.yaml';base.write_text(subject.limits.yaml.safe_dump(BASE),encoding='utf-8')
        runtime=root/'runtime';subject.limits.prepare(selected,base,runtime)
        result=subject.selected_arguments(SELECTION,installed_profile=selected,runtime_selection=runtime)
        assert result['installed_profile_link_verified'] and result['runtime_params_selected']
        assert result['argument_fragments']['planner']==result['argument_fragments']['guard']==['--ros-args','--params-file',str(runtime/'runtime_profile.yaml')]
        assert result['effective_command_limit_rad']==selection['command_limit_rad']
        smaller=subject.limits.yaml.safe_load(base.read_text())
        for node in subject.limits.NODES:smaller[node]['ros__parameters']['max_steering_angle_rad']=.15
        base.write_text(subject.limits.yaml.safe_dump(smaller),encoding='utf-8')
        with pytest.raises(ValueError,match='would widen existing steering limit'):
            subject.limits.prepare(selected,base,root/'runtime_small')
        with pytest.raises(ValueError,match='supplied together'):subject.selected_arguments(SELECTION,installed_profile=selected)
        with pytest.raises(ValueError,match='differs from selected'):subject.selected_arguments(SELECTION,installed_profile=bundle,runtime_selection=runtime)


def test_launcher_preview_and_panel_accept_selected_arguments_without_running():
    from types import SimpleNamespace
    from probe_g1_hardware_shadow import vehicle_geometry_override,ros_arguments
    from arena_autonomy.race_ops_manual import ManualPanel
    value=subject.selected_arguments(SELECTION)
    config=subject.ROOT/value['vehicle_config']['path']
    args=SimpleNamespace(vehicle_config=config,output=Path('NO_DEVICE_OUTPUT'),estimated_time=True,camera=False)
    geometry=vehicle_geometry_override(args)
    assert geometry['parameters']['max_steering_angle_rad']==value['source_command_limit_rad']
    assert 'max_steering_angle_rad:=0.28475436568260193' in ros_arguments(args,subject.ROOT)
    panel=ManualPanel(steering_limit_rad=.2,spawn=lambda *a,**k: (_ for _ in ()).throw(AssertionError('no process allowed')))
    assert panel.steering_limit_rad<=value['source_command_limit_rad']
