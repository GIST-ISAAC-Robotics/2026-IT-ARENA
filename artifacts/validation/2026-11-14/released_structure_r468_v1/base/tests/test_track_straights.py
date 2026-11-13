"""직선거리 분석의 폐회로 처리와 두 지도 간 동일성을 검사합니다."""

import importlib.util
import math
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("analyze_track_straights", REPO / "scripts/analyze_track_straights.py")
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)


def test_straight_crossing_csv_origin_is_not_split():
    points = [(1., 0.), (4., 0.), (4., 1.), (2., 1.), (2., 2.), (0., 2.), (0., 0.)]
    result = ANALYSIS.longest_straight(points, 1.0)
    assert result["length_m"] == pytest.approx(4.0)
    assert result["crosses_csv_origin"] is True
    assert result["maximum_chord_deviation_m"] == pytest.approx(0.0)
    angle = math.radians(179.8)
    rotated = [(x * math.cos(angle) - y * math.sin(angle), x * math.sin(angle) + y * math.cos(angle))
               for x, y in points]
    assert ANALYSIS.longest_straight(rotated, 1.0)["length_m"] == pytest.approx(4.0)


def test_original_and_experimental_share_long_straight_geometry():
    original = ANALYSIS.analyze_track(REPO, "original")
    experimental = ANALYSIS.analyze_track(REPO, "experimental")
    assert original["longest_segments"] == experimental["longest_segments"]
    assert original["lap_length_from_csv_m"] == pytest.approx(46.6329, abs=1e-4)
    strict, moderate, planning = original["longest_segments"]
    assert strict["length_m"] == pytest.approx(5.44, abs=.02)
    assert moderate["length_m"] == pytest.approx(10.28, abs=.02)
    assert planning["length_m"] == pytest.approx(10.30, abs=.02)
    assert planning["maximum_chord_deviation_m"] < .03
    assert planning["crosses_csv_origin"] is True


def test_degenerate_or_invalid_inputs_are_rejected():
    with pytest.raises(ValueError):
        ANALYSIS.longest_straight([(0., 0.), (1., 0.)], 1.0)
    with pytest.raises(ValueError):
        ANALYSIS.longest_straight([(0., 0.), (1., 0.), (1., 0.)], 1.0)
