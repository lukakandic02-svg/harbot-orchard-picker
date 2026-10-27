"""Tests for the orchard workspace graph (harbot_orchard_picker.orchard_topology).

Two kinds of checks:
  * regression: without shortcuts, the shared builder reproduces exactly the
    edge sets the planner and motion layers used to build by hand;
  * properties of the five experiment topologies, matching Table 4.7.1 of the
    thesis (node/edge counts, station diameter, bypassable row ends).
"""
from collections import deque

import pytest

from harbot_orchard_picker import orchard_topology as ot

NUM_PASSAGES, NUM_INDICES = 5, 8
STATIONS = ot.base_locations(NUM_PASSAGES, NUM_INDICES)
ROW_ENDS = ["P%d_I%d" % (p, i) for p in range(NUM_PASSAGES) for i in (1, NUM_INDICES)]


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def adjacency(locs, edges, removed=()):
    adj = {loc: [] for loc in locs if loc not in removed}
    for a, b in edges:
        if a in adj and b in adj:
            adj[a].append(b)
    return adj


def bfs_hops(adj, start):
    dist = {start: 0}
    queue = deque([start])
    while queue:
        cur = queue.popleft()
        for nxt in adj[cur]:
            if nxt not in dist:
                dist[nxt] = dist[cur] + 1
                queue.append(nxt)
    return dist


def legacy_ladder_edges(loc_set):
    """The hand-written edge loop the planner/motion layers used before the
    shared module existed (both endcaps of every passage pair)."""
    edges = []
    for p in range(5):
        for i in range(1, 8):
            a, b = "P%d_I%d" % (p, i), "P%d_I%d" % (p, i + 1)
            if a in loc_set and b in loc_set:
                edges += [(a, b), (b, a)]
    for p in range(4):
        for i in (1, 8):
            a, b = "P%d_I%d" % (p, i), "P%d_I%d" % (p + 1, i)
            if a in loc_set and b in loc_set:
                edges += [(a, b), (b, a)]
    return edges


# --------------------------------------------------------------------------
# regression: shared builder == legacy hand-written graphs
# --------------------------------------------------------------------------
def test_base_locations():
    assert len(STATIONS) == 40
    assert STATIONS[0] == "P0_I1" and STATIONS[-1] == "P4_I8"


def test_base_edges_match_legacy_full_set():
    loc_set = set(STATIONS)
    assert sorted(ot.base_edges(loc_set)) == sorted(legacy_ladder_edges(loc_set))


def test_base_edges_match_legacy_restricted_zone():
    zone = {loc for loc in STATIONS if loc.split("_")[0] in ("P0", "P1", "P2")}
    assert sorted(ot.base_edges(zone)) == sorted(legacy_ladder_edges(zone))


def test_no_shortcuts_is_a_no_op():
    assert ot.shortcut_locations([]) == []
    assert ot.shortcut_edges([], loc_set=set(STATIONS)) == []


# --------------------------------------------------------------------------
# shortcuts and edge classification
# --------------------------------------------------------------------------
def test_parse_shortcut_pairs():
    assert ot.parse_shortcut_pairs(["P1-P2:2", " P3-P4:7 "]) == [(1, 2, 2), (3, 4, 7)]


@pytest.mark.parametrize("bad", ["P1-P3:2", "P1P2:2", "1-2:3", "P1-P2"])
def test_parse_shortcut_pairs_rejects_bad_specs(bad):
    with pytest.raises(ValueError):
        ot.parse_shortcut_pairs([bad])


def test_shortcut_adds_two_nodes_and_ten_edges():
    locs = ot.shortcut_locations(["P1-P2:2"])
    assert locs == ["P1_M2_2", "P2_M2_1"]
    edges = ot.shortcut_edges(["P1-P2:2"], loc_set=set(STATIONS))
    assert len(edges) == len(set(edges)) == 10
    assert ("P1_M2_2", "P2_M2_1") in edges and ("P2_M2_1", "P1_M2_2") in edges


def test_midpoints_toward_both_neighbours_do_not_collide():
    # An inner passage borders two neighbours; shortcuts to both at the same
    # gap index must produce two distinct midpoint nodes.
    locs = ot.shortcut_locations(["P0-P1:3", "P1-P2:3"])
    assert len(locs) == len(set(locs)) == 4


def test_classify_edge_kinds():
    for a, b in ot.base_edges(set(STATIONS)):
        assert ot.classify_edge(a, b)[0] in ("passage", "transfer")
    assert ot.classify_edge("P1_I3", "P1_I4") == ("passage", 1, 3, 4)
    assert ot.classify_edge("P2_I8", "P3_I8") == ("transfer", 2, 3, 8)
    m12, m21 = ot.midpoint_node(1, 2, 2), ot.midpoint_node(2, 2, 1)
    assert ot.classify_edge(m12, m21)[0] == "crossing"
    assert ot.classify_edge("P1_I2", m12)[0] == "feeder"
    assert ot.classify_edge(m12, "P1_I3")[0] == "feeder"
    # same gap, adjacent passages, but the midpoints do not face each other
    assert ot.classify_edge(ot.midpoint_node(1, 2, 0), m21) == (None,)
    # an edge straight through a tree is not an adjacency
    assert ot.classify_edge("P1_I4", "P2_I4") == (None,)


def test_fractional_index():
    assert ot.loc_fractional_index("P3_I5") == (3, 5.0)
    assert ot.loc_fractional_index("P3_M5_2") == (3, 5.5)


# --------------------------------------------------------------------------
# the five experiment topologies (thesis Table 4.7.1)
# --------------------------------------------------------------------------
TABLE_4_7_1 = {
    #        nodes, directed edges, station diameter, bypassable row ends
    "line": (40, 78, 39, 2),
    "m1": (54, 150, 25, 4),
    "m2": (68, 222, 25, 6),
    "m3": (82, 294, 18, 8),
    "full": (96, 366, 11, 10),
}


@pytest.mark.parametrize("name", list(TABLE_4_7_1))
def test_topology_size(name):
    nodes, n_edges, _, _ = TABLE_4_7_1[name]
    locs, edges = ot.build_topology(name)
    assert len(locs) == len(set(locs)) == nodes
    assert len(edges) == len(set(edges)) == n_edges


@pytest.mark.parametrize("name", list(TABLE_4_7_1))
def test_topology_is_symmetric(name):
    _, edges = ot.build_topology(name)
    edge_set = set(edges)
    assert all((b, a) in edge_set for a, b in edge_set)


@pytest.mark.parametrize("name", list(TABLE_4_7_1))
def test_topology_station_diameter(name):
    locs, edges = ot.build_topology(name)
    adj = adjacency(locs, edges)
    diameter = max(bfs_hops(adj, s)[t] for s in STATIONS for t in STATIONS)
    assert diameter == TABLE_4_7_1[name][2]


@pytest.mark.parametrize("name", list(TABLE_4_7_1))
def test_bypassable_row_ends(name):
    """A row end is bypassable if every other station stays reachable when
    that node is blocked - i.e. a peer parked there can be routed around."""
    locs, edges = ot.build_topology(name)
    bypassable = 0
    for end in ROW_ENDS:
        adj = adjacency(locs, edges, removed={end})
        others = [s for s in STATIONS if s != end]
        reached = bfs_hops(adj, others[0])
        bypassable += all(s in reached for s in others)
    assert bypassable == TABLE_4_7_1[name][3]


def test_depots_are_bypassable_in_every_topology():
    for name in TABLE_4_7_1:
        locs, edges = ot.build_topology(name)
        for depot in ("P0_I1", "P4_I8"):
            adj = adjacency(locs, edges, removed={depot})
            others = [s for s in STATIONS if s != depot]
            reached = bfs_hops(adj, others[0])
            assert all(s in reached for s in others), (name, depot)


def test_line_is_a_single_serpentine_path():
    locs, edges = ot.build_topology("line")
    adj = adjacency(locs, edges)
    degrees = sorted(len(v) for v in adj.values())
    assert degrees.count(1) == 2 and degrees.count(2) == 38


def test_connectivity_grows_monotonically():
    order = ["line", "m1", "m2", "m3", "full"]
    edge_sets = [set(ot.build_topology(n)[1]) for n in order]
    station_edges = [{e for e in s if e[0] in STATIONS and e[1] in STATIONS}
                     for s in edge_sets]
    for smaller, larger in zip(station_edges, station_edges[1:]):
        assert smaller <= larger
