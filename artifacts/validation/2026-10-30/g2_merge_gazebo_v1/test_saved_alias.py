import json
from pathlib import Path
import pytest
from reevaluate_saved import add_legacy_alias


def test_only_new_legacy_alias_changes_and_input_is_unchanged():
    cfg=json.loads(Path(__file__).with_name('g2_center_config_v1.json').read_text())
    before=json.dumps(cfg,sort_keys=True)
    fixed=add_legacy_alias(cfg)
    assert json.dumps(cfg,sort_keys=True)==before
    assert fixed['acceptance'].pop('g2_center_s_merge')==cfg['acceptance']['stopped_center']
    assert fixed==cfg


def test_unexpected_alias_or_reference_rejected():
    cfg=json.loads(Path(__file__).with_name('g2_center_config_v1.json').read_text())
    with pytest.raises(ValueError):add_legacy_alias(add_legacy_alias(cfg))
    cfg['acceptance']['g2_center_stopped']={'minimum_continuous_hold_s':1}
    with pytest.raises(ValueError):add_legacy_alias(cfg)
