from copy import deepcopy
import xml.etree.ElementTree as ET
import pytest
from level_surface import level_tree


def fixture():
    root=ET.fromstring('<sdf><world><model name="track"/></world></sdf>')
    model=root.find('.//model')
    for name,center,thickness in [('grass',.001,.002),('track_surface',.0015,.003)]:
        link=ET.SubElement(model,'link',name=name)
        for tag in ('collision','visual'):
            item=ET.SubElement(link,tag,name=name+'_'+tag)
            ET.SubElement(item,'pose').text=f'1 2 {center} 0 0 .7'
            box=ET.SubElement(ET.SubElement(item,'geometry'),'box')
            ET.SubElement(box,'size').text=f'.2 .2 {thickness}'
    return root


def test_only_grass_translation_changes():
    root=fixture();before=deepcopy(root)
    result=level_tree(root)
    assert result['variant_grass_top_m']==result['road_top_m']==.003
    grass=root.find(".//link[@name='grass']")
    assert grass.findtext('pose')=='0 0 0.001 0 0 0'
    grass.remove(grass.find('pose'))
    assert ET.tostring(root)==ET.tostring(before)


def test_unregistered_height_or_transform_is_rejected():
    root=fixture();root.find(".//link[@name='grass']/collision/pose").text='1 2 .002 0 0 .7'
    with pytest.raises(ValueError):level_tree(root)
    root=fixture();ET.SubElement(root.find(".//link[@name='grass']"),'pose').text='0 0 0 0 0 0'
    with pytest.raises(ValueError):level_tree(root)


def test_sloped_or_missing_geometry_is_rejected():
    root=fixture();root.find(".//link[@name='grass']/collision/pose").text='1 2 .001 .1 0 .7'
    with pytest.raises(ValueError):level_tree(root)
    root=fixture();item=root.find(".//link[@name='grass']/collision")
    item.remove(item.find('geometry'))
    with pytest.raises(ValueError):level_tree(root)


def test_actor_schedules_cover_registered_window_and_shortening_must_include_both():
    import json
    from pathlib import Path
    from traffic_actors import actor_specs,overlay_case
    b=Path(__file__).resolve().parent
    config=json.loads((b/'traffic_config.json').read_text())
    case=overlay_case(config['cases'][0],b/'actor_overlay.json')
    assert len(actor_specs(config,case))==2
    case['duration_s']=40.
    case['schedule'][-1]['to_s']=40.
    with pytest.raises(ValueError,match='schedule must cover'):actor_specs(config,case)
