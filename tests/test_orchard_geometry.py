"""Tests for the calibrated orchard geometry and the shipped calibration."""
import math
import os

import pytest

from harbot_orchard_picker.orchard_geometry import OrchardGeometry

REPO = os.path.dirname(os.path.dirname(__file__))
TREE_POINTS = os.path.join(REPO, "config", "tree_points.jsonl")


@pytest.fixture(scope="module")
def geometry():
    return OrchardGeometry(TREE_POINTS, num_rows=4, num_indices=8)


def station_xy(g, passage, index):
    """Forward model: passage centre lines sit half a row spacing outside /
    between tree rows; stations sit level with the trees."""
    along = (index - 1) * g.tree_spacing
    across = (passage - 0.5) * g.row_spacing
    return (g.t11[0] + along * g.length_unit[0] + across * g.width_unit[0],
            g.t11[1] + along * g.length_unit[1] + across * g.width_unit[1])


def test_calibration_matches_thesis_geometry(geometry):
    # Thesis Table 4.1.1: 3.61 m between stations, 4.02 m between passages,
    # 25.29 m row length (I1 to I8).
    assert geometry.tree_spacing == pytest.approx(3.61, abs=0.01)
    assert geometry.row_spacing == pytest.approx(4.02, abs=0.01)
    assert 7 * geometry.tree_spacing == pytest.approx(25.29, abs=0.02)


# The calibrated rows are not exactly perpendicular to the passages, while
# the inverse uses orthogonal projections; the resulting round-trip error is
# bounded well below a centimetre (0.002 of a 3.6 m station spacing).
ROUND_TRIP_TOL = 2e-3


@pytest.mark.parametrize("passage", range(5))
@pytest.mark.parametrize("index", [1, 4, 5, 8])
def test_round_trip_station(geometry, passage, index):
    x, y = station_xy(geometry, passage, index)
    p, frac = geometry.xy_to_passage_index(x, y)
    assert p == passage
    assert frac == pytest.approx(index, abs=ROUND_TRIP_TOL)


def test_midpoint_between_trees(geometry):
    x, y = station_xy(geometry, 2, 3.5)
    p, frac = geometry.xy_to_passage_index(x, y)
    assert p == 2
    assert frac == pytest.approx(3.5, abs=ROUND_TRIP_TOL)


def test_passage_is_clamped_to_the_orchard(geometry):
    far_out = station_xy(geometry, 9, 4)
    assert geometry.xy_to_passage_index(*far_out)[0] == 4
    far_in = station_xy(geometry, -5, 4)
    assert geometry.xy_to_passage_index(*far_in)[0] == 0


def test_units_are_orthonormal(geometry):
    lx, ly = geometry.length_unit
    wx, wy = geometry.width_unit
    assert math.hypot(lx, ly) == pytest.approx(1.0)
    assert math.hypot(wx, wy) == pytest.approx(1.0)
    assert abs(lx * wx + ly * wy) < 0.01  # rows are (near) perpendicular to passages
