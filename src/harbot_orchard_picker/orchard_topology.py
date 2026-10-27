#!/usr/bin/env python3

"""Orchard workspace graph, shared by the planner and the motion layer.

The orchard has 5 passages (P0..P4) between 4 tree rows and 8 stations per
passage (I1..I8), named ``P{p}_I{i}``. Mid-row shortcut crossings add
helper nodes ``P{p}_M{k}_{q}`` between two trees (see ``midpoint_node``).

Used by online_orchard_manager.py (write_instance() for the RDDL
``adjacent`` non-fluents and build_graph_edges() for routing) and by
orchard_map_picker.py (build_map()), so PROST and the physical layer
always use the same edge set.

The five topologies used in the experiments (line, m1, m2, m3, full) are
defined in ``TOPOLOGIES`` / ``build_topology()``, see docs/topologies.md.
No ROS dependency.
"""

import re

_LOC_RE = re.compile(r"^P(\d+)_I(\d+)$")
_MID_RE = re.compile(r"^P(\d+)_M(\d+)_(\d+)$")


def base_locations(num_passages=5, num_indices=8):
    """All P{p}_I{i} locations, p in [0, num_passages), i in [1, num_indices]."""
    return [
        "P%d_I%d" % (p, i)
        for p in range(num_passages)
        for i in range(1, num_indices + 1)
    ]


def base_edges(loc_set, num_passages=5, num_indices=8):
    """Directed edges for passage travel and endcap transfers between neighbouring
    passages, limited to locations in loc_set.
    """
    edges = []

    for p in range(num_passages):
        for i in range(1, num_indices):
            a, b = "P%d_I%d" % (p, i), "P%d_I%d" % (p, i + 1)
            if a in loc_set and b in loc_set:
                edges.append((a, b))
                edges.append((b, a))

    for p in range(num_passages - 1):
        for i in (1, num_indices):
            a, b = "P%d_I%d" % (p, i), "P%d_I%d" % (p + 1, i)
            if a in loc_set and b in loc_set:
                edges.append((a, b))
                edges.append((b, a))

    return edges


def line_edges(loc_set, num_passages=5, num_indices=8, redundant_pairs=None):
    """Directed edges for the "line" topology: one serpentine path through all stations.

    Passages are connected at only one endcap, alternating:

    P0 (I1->I8) -I8- P1 (I8->I1) -I1- P2 (I1->I8) -I8-
    P3 (I8->I1) -I1- P4 (I1->I8)

    Every station has 2 neighbours except the two ends, so a blocked node
    cannot be routed around.

    `redundant_pairs`: optional passage-pair indices p (P{p}-P{p+1}) that also
    get the second endcap, giving the intermediate topologies between line
    and ladder.
    """
    edges = []

    for p in range(num_passages):
        for i in range(1, num_indices):
            a, b = "P%d_I%d" % (p, i), "P%d_I%d" % (p, i + 1)
            if a in loc_set and b in loc_set:
                edges.append((a, b))
                edges.append((b, a))

    redundant = set(redundant_pairs) if redundant_pairs else set()
    for p in range(num_passages - 1):
        i = num_indices if p % 2 == 0 else 1
        a, b = "P%d_I%d" % (p, i), "P%d_I%d" % (p + 1, i)
        if a in loc_set and b in loc_set:
            edges.append((a, b))
            edges.append((b, a))
        if p in redundant:
            other_i = 1 if i == num_indices else num_indices
            a2, b2 = "P%d_I%d" % (p, other_i), "P%d_I%d" % (p + 1, other_i)
            if a2 in loc_set and b2 in loc_set:
                edges.append((a2, b2))
                edges.append((b2, a2))

    return edges


def parse_shortcut_pairs(shortcut_pairs):
    """Parse ["P1-P2:2", "P2-P3:6"] into (p, p+1, gap_index) tuples.

    gap_index k means the crossing lies between stations I{k} and I{k+1}.
    """
    parsed = []
    for spec in shortcut_pairs or []:
        m = re.match(r"^P(\d+)-P(\d+):(\d+)$", str(spec).strip())
        if not m:
            raise ValueError("Bad shortcut_pair spec: %r" % (spec,))
        p, q, k = int(m.group(1)), int(m.group(2)), int(m.group(3))
        if q != p + 1:
            raise ValueError(
                "shortcut_pair must connect adjacent passages P{p}-P{p+1}: %r"
                % (spec,)
            )
        parsed.append((p, q, k))
    return parsed


def midpoint_node(passage, gap_index, neighbor):
    """Location of a mid-row crossing on one side of a shortcut.

    Station nodes sit at the trees, so a crossing needs its own node between
    two trees. `neighbor` (the passage the crossing leads to) is required
    because inner passages have two neighbours.
    """
    return "P%d_M%d_%d" % (passage, gap_index, neighbor)


def shortcut_locations(shortcut_pairs):
    """The 2 new location objects per configured shortcut."""
    locs = []
    for p, q, k in parse_shortcut_pairs(shortcut_pairs):
        locs.append(midpoint_node(p, k, q))
        locs.append(midpoint_node(q, k, p))
    return locs


def shortcut_edges(shortcut_pairs, loc_set=None):
    """10 directed edges per configured shortcut: on each side, feeder
    edges in both directions between the midpoint node and the two
    stations it sits between (I{k} and I{k+1}; gated on the station
    existing in loc_set, same convention as base_edges), plus the actual
    mid-row crossing edge between the two midpoint nodes, both ways."""
    edges = []
    for p, q, k in parse_shortcut_pairs(shortcut_pairs):
        mp, mq = midpoint_node(p, k, q), midpoint_node(q, k, p)
        for passage, mid in ((p, mp), (q, mq)):
            lo = "P%d_I%d" % (passage, k)
            hi = "P%d_I%d" % (passage, k + 1)
            if loc_set is None or lo in loc_set:
                edges.append((lo, mid))
                edges.append((mid, lo))
            if loc_set is None or hi in loc_set:
                edges.append((hi, mid))
                edges.append((mid, hi))
        edges.append((mp, mq))
        edges.append((mq, mp))
    return edges


def parse_loc(loc):
    """Return ("I", passage, index) or ("M", passage, gap_index, neighbor)."""
    m = _LOC_RE.match(loc)
    if m:
        return ("I", int(m.group(1)), int(m.group(2)))
    m = _MID_RE.match(loc)
    if m:
        return ("M", int(m.group(1)), int(m.group(2)), int(m.group(3)))
    raise ValueError("Bad loc: %r" % (loc,))


def loc_fractional_index(loc):
    """Return (passage, fractional_index) for a station or a midpoint.

    Midpoints get index k + 0.5, which passage_pose() accepts directly.
    """
    parsed = parse_loc(loc)
    kind, passage, idx = parsed[0], parsed[1], parsed[2]
    if kind == "I":
        return passage, float(idx)
    return passage, idx + 0.5


def classify_edge(a, b, num_indices=8):
    """Classify a directed edge between two locations. The first element is one of:

    "passage"  (kind, passage, from_index, to_index): move between adjacent
    stations in one passage.
    "transfer" (kind, from_passage, to_passage, index): endcap transfer at
    I1 or I{num_indices}.
    "feeder"   (kind, passage, station_index, gap_index): station <-> shortcut
    midpoint in the same passage.
    "crossing" (kind, from_passage, to_passage, gap_index): mid-row crossing
    between two midpoint nodes.
    None       (None,): not an adjacency.
    """
    # Midpoint tuples have an extra `neighbor` element, so index by position.
    parsed_a, parsed_b = parse_loc(a), parse_loc(b)
    ta, pa, ia = parsed_a[0], parsed_a[1], parsed_a[2]
    tb, pb, ib = parsed_b[0], parsed_b[1], parsed_b[2]

    if ta == "I" and tb == "I":
        if pa == pb and abs(ia - ib) == 1:
            return ("passage", pa, ia, ib)
        if abs(pa - pb) == 1 and ia == ib and ia in (1, num_indices):
            return ("transfer", pa, pb, ia)
        return (None,)

    if ta == "I" and tb == "M" and pa == pb and ia in (ib, ib + 1):
        return ("feeder", pa, ia, ib)

    if ta == "M" and tb == "I" and pa == pb and ib in (ia, ia + 1):
        return ("feeder", pa, ib, ia)

    if ta == "M" and tb == "M" and abs(pa - pb) == 1 and ia == ib:
        # The two midpoints must face each other.
        na, nb = parsed_a[3], parsed_b[3]
        if na == pb and nb == pa:
            return ("crossing", pa, pb, ia)
        return (None,)

    return (None,)


# The five workspace topologies of the experiment matrix, from least to most
# connected. Each entry mirrors the robot parameters experiments/
# run_experiment.sh passes for that topology:
#   mode            -> ~topology_mode ("line" = serpentine base, "ladder" =
#                      every neighbouring passage pair joined at BOTH endcaps)
#   redundant_pairs -> ~topology_redundant_pairs (line mode only)
#   shortcut_pairs  -> passage pairs that get all 7 mid-row shortcuts
# m1..m3 upgrade one passage pair at a time, centre-outward, from
# line-minimal to fully connected (second endcap + 7 shortcuts).
TOPOLOGIES = {
    "line": {"mode": "line", "redundant_pairs": [], "shortcut_pairs": []},
    "m1": {"mode": "line", "redundant_pairs": [1], "shortcut_pairs": [1]},
    "m2": {"mode": "line", "redundant_pairs": [1, 2], "shortcut_pairs": [1, 2]},
    "m3": {"mode": "line", "redundant_pairs": [0, 1, 2],
           "shortcut_pairs": [0, 1, 2]},
    "full": {"mode": "ladder", "redundant_pairs": [],
             "shortcut_pairs": [0, 1, 2, 3]},
}


def shortcut_specs(passage_pairs, num_indices=8):
    """All mid-row shortcut specs ("P{p}-P{p+1}:{k}") for the given passage
    pairs, one per tree gap k in [1, num_indices - 1]."""
    return [
        "P%d-P%d:%d" % (p, p + 1, k)
        for p in passage_pairs
        for k in range(1, num_indices)
    ]


def build_topology(name, num_passages=5, num_indices=8):
    """Return (locations, directed_edges) for one of TOPOLOGIES.

    Composes the graph exactly the way OnlineOrchardManager.write_instance()
    does for the same parameters, so it can be used offline (tests,
    analysis, figures) without ROS.
    """
    spec = TOPOLOGIES[name]
    shortcuts = shortcut_specs(spec["shortcut_pairs"], num_indices)
    locs = base_locations(num_passages, num_indices) + shortcut_locations(shortcuts)
    loc_set = set(locs)
    if spec["mode"] == "line":
        edges = line_edges(loc_set, num_passages, num_indices,
                           redundant_pairs=spec["redundant_pairs"])
    else:
        edges = base_edges(loc_set, num_passages, num_indices)
    edges = edges + shortcut_edges(shortcuts, loc_set=loc_set)
    return locs, edges
